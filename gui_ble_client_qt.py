#!/usr/bin/env python3
# gui_ble_client_qt.py
# macOS-compatible Qt (PySide6) GUI for ESP32 Load Cell BLE Server.
# Same functionality as gui_ble_client.py: scan, connect, commands, live values,
# time-series plot, and Center of Pressure (CoP) visualization.
#
# Run:   pip install PySide6 pyqtgraph bleak
#        python gui_ble_client_qt.py
# Dependencies: PySide6, pyqtgraph, bleak

import asyncio
import struct
import threading
import queue
import sys
import time
from collections import deque
from typing import Optional, List, Dict

from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QLineEdit, QComboBox, QCheckBox, QTextEdit, QGroupBox,
    QFrame, QScrollArea, QMessageBox, QDialog, QDialogButtonBox,
)
from PySide6.QtGui import QFont

from bleak import BleakClient, BleakScanner, BleakError

import pyqtgraph as pg

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False

# ==== UUIDs ====
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
DATA_CHAR_UUID = "87654321-4321-4321-4321-cba987654321"
COMMAND_CHAR_UUID = "11111111-2222-3333-4444-555555555555"
CALIBRATION_LOG_CHAR_UUID: Optional[str] = None

DEFAULT_DEVICE_NAME = "LoadCell_BLE_Server"

COMMAND_GROUPS = {
    "Local": ["START", "STOP", "RESTART", "RESET"],
    "Remote": ["REMOTE_START", "REMOTE_STOP", "REMOTE_RESTART", "REMOTE_RESET"],
    "Dual": ["ALL_START", "ALL_STOP", "ALL_RESTART", "ALL_RESET"],
    "Ping": ["LOCAL_PING", "REMOTE_PING"],
    "System": ["STATS", "RESET_STATS"],
}

CHANNEL_NAMES = ["L1", "L2", "L3", "L4", "R5", "R6", "R7", "R8"]

PLATE_WIDTH_CM = 22.0
PLATE_LENGTH_CM = 35.0


def calculate_cop(f1: float, f2: float, f3: float, f4: float,
                  width: float = PLATE_WIDTH_CM, length: float = PLATE_LENGTH_CM):
    total = f1 + f2 + f3 + f4
    if total < 10:
        return (0.0, 0.0, total, False)
    cop_x = (width / 2.0) * ((f2 + f3) - (f1 + f4)) / total
    cop_y = (length / 2.0) * ((f1 + f2) - (f4 + f3)) / total
    return (cop_x, cop_y, total, True)


def parse_packet(data: bytes):
    if not data or len(data) < 1:
        return {"sample_count": 0, "samples": []}
    sample_count = data[0]
    if sample_count > 10:
        return {"sample_count": 0, "samples": []}
    expected_len = 1 + sample_count * 16
    if len(data) < expected_len or sample_count == 0:
        return {"sample_count": 0, "samples": []}
    samples: List[List[int]] = []
    for i in range(sample_count):
        off = 1 + i * 16
        local4 = struct.unpack_from("<4h", data, off)
        remote4 = struct.unpack_from("<4h", data, off + 8)
        samples.append(list(local4) + list(remote4))
    return {"sample_count": sample_count, "samples": samples}


# ===== BLE Worker (queue-based; connection state fixed for macOS) =====
class BLEWorker:
    def __init__(self, ui_q: queue.Queue):
        self.client: Optional[BleakClient] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        self.ui_q = ui_q
        self.selected_address: Optional[str] = None
        self.packet_counter = 0
        self._subscribed_notify_uuids = set()
        # Explicit connection flag (is_connected can be unreliable on macOS)
        self._connected = False
        # Cache BLEDevice from last scan so we can pass device object on connect (more reliable on macOS)
        self._device_cache: Dict = {}

    def send_ui(self, kind: str, payload):
        self.ui_q.put({"kind": kind, "payload": payload})

    def start(self):
        if self.thread and self.thread.is_alive():
            return
        self.thread = threading.Thread(target=self._thread_main, daemon=True)
        self.thread.start()

    def stop(self):
        if self.loop:
            asyncio.run_coroutine_threadsafe(self._shutdown(), self.loop)

    async def _shutdown(self):
        try:
            if self.client:
                try:
                    if self.client.is_connected:
                        await self.client.disconnect()
                except Exception:
                    pass
                self.send_ui("status", "Disconnected.")
        except Exception as e:
            self.send_ui("log", f"Error during disconnect: {e}")
        finally:
            self._connected = False
            self.client = None
            await asyncio.sleep(0.05)
            if self.loop.is_running():
                self.loop.stop()

    def _thread_main(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.send_ui("status", "BLE thread started.")
        try:
            self.loop.run_forever()
        except Exception as e:
            self.send_ui("log", f"Loop error: {e}")
        finally:
            self.send_ui("status", "BLE thread stopped.")

    def scan(self, use_service_filter: bool, name_hint: str):
        asyncio.run_coroutine_threadsafe(
            self._scan_task(use_service_filter, name_hint), self.loop
        )

    def connect(self, address: Optional[str]):
        self.selected_address = address
        asyncio.run_coroutine_threadsafe(self._connect_task(), self.loop)

    def disconnect(self):
        if self.loop:
            asyncio.run_coroutine_threadsafe(self._disconnect_task(), self.loop)

    def send_command(self, cmd: str):
        asyncio.run_coroutine_threadsafe(self._send_command_task(cmd), self.loop)

    async def _scan_task(self, use_service_filter: bool, name_hint: str):
        self.send_ui("status", "Scanning for devices...")
        try:
            devices = await BleakScanner.discover(timeout=5.0)
        except Exception as e:
            self.send_ui("status", f"Scan error: {e}")
            self.send_ui("devices", [])
            return
        results = []
        name_hint_l = (name_hint or "").strip().lower()
        for d in devices:
            nm = d.name or ""
            show = True
            if name_hint_l and name_hint_l not in nm.lower():
                show = False
            if use_service_filter:
                adv_uuids = set()
                try:
                    if hasattr(d, "metadata") and d.metadata and "uuids" in d.metadata:
                        adv_uuids = set(d.metadata.get("uuids") or [])
                except Exception:
                    adv_uuids = set()
                if SERVICE_UUID.lower() not in [u.lower() for u in adv_uuids]:
                    if not name_hint_l:
                        show = False
            if show:
                results.append({
                    "name": nm if nm else "(unknown)",
                    "address": d.address,
                    "rssi": getattr(d, "rssi", None),
                })
                # Cache BLEDevice for connect (macOS works better with device object than address string)
                self._device_cache[d.address] = d
        results.sort(key=lambda x: (x["rssi"] is None, -(x["rssi"] or -999), x["name"]))
        self.send_ui("devices", results)
        self.send_ui("status", f"Scan complete. Found {len(results)} device(s).")

    async def _connect_task(self):
        if not self.selected_address:
            self.send_ui("status", "No device selected.")
            return
        if self._connected and self.client:
            self.send_ui("status", "Already connected.")
            return
        # On macOS, passing BLEDevice from scanner is more reliable than address string
        device = self._device_cache.get(self.selected_address)
        self.client = BleakClient(device if device is not None else self.selected_address)
        try:
            await self.client.connect()
            self.send_ui("status", f"Connected: {self.selected_address}")
            self._subscribed_notify_uuids.clear()
            # Bleak 1.x: services are discovered during connect(); use .services property
            services = self.client.services
            available = set()
            try:
                for svc in services:
                    for ch in getattr(svc, "characteristics", []) or []:
                        if getattr(ch, "uuid", None):
                            available.add(ch.uuid.lower())
            except Exception:
                available = set()
            if not available or DATA_CHAR_UUID.lower() in available:
                await self.client.start_notify(DATA_CHAR_UUID, self._notification_handler)
                self._subscribed_notify_uuids.add(DATA_CHAR_UUID)
            else:
                raise BleakError(f"Device missing DATA characteristic {DATA_CHAR_UUID}")
            if CALIBRATION_LOG_CHAR_UUID and (
                (not available) or (CALIBRATION_LOG_CHAR_UUID.lower() in available)
            ):
                await self.client.start_notify(
                    CALIBRATION_LOG_CHAR_UUID, self._calibration_log_handler
                )
                self._subscribed_notify_uuids.add(CALIBRATION_LOG_CHAR_UUID)
            self.packet_counter = 0
            self._connected = True
            self.send_ui("status", "Notifications enabled.")
        except BleakError as e:
            self.send_ui("status", f"Connect error: {e}")
            self._connected = False
            self.client = None
        except Exception as e:
            self.send_ui("status", f"Connect error: {e}")
            self._connected = False
            self.client = None

    async def _disconnect_task(self):
        try:
            if self.client:
                for uuid in list(self._subscribed_notify_uuids):
                    try:
                        await self.client.stop_notify(uuid)
                    except Exception:
                        pass
                try:
                    if self.client.is_connected:
                        await self.client.disconnect()
                except Exception:
                    pass
                self.send_ui("status", "Disconnected.")
        except Exception as e:
            self.send_ui("log", f"Disconnect error: {e}")
        finally:
            self._connected = False
            self._subscribed_notify_uuids.clear()
            self.client = None

    async def _send_command_task(self, cmd: str):
        cmd = (cmd or "").strip()
        if not cmd:
            self.send_ui("log", "Empty command.")
            return
        # Use explicit _connected flag; is_connected can be unreliable on macOS
        if not self._connected or not self.client:
            self.send_ui("log", "Not connected.")
            return
        try:
            await self.client.write_gatt_char(
                COMMAND_CHAR_UUID, cmd.encode("utf-8"), response=False
            )
            self.send_ui("log", f">> {cmd}")
        except Exception as e:
            self.send_ui("log", f"Command error: {e}")
            # If write failed (e.g. device dropped), clear connection state so UI reflects it
            if "disconnect" in str(e).lower() or "not connected" in str(e).lower():
                self._connected = False
                self.client = None
                self.send_ui("status", "Disconnected (send failed).")

    def _notification_handler(self, sender, data: bytearray):
        parsed = parse_packet(bytes(data))
        if not parsed["sample_count"]:
            return
        self.packet_counter += 1
        samples = parsed["samples"]
        self.send_ui("samples", samples)
        mins = [min(s[ch] for s in samples) for ch in range(8)]
        maxs = [max(s[ch] for s in samples) for ch in range(8)]
        first, last = samples[0], samples[-1]
        line1 = (f"[pkt {self.packet_counter:06d}] samples={parsed['sample_count']}  "
                 f"first L={first[0:4]} R={first[4:8]}  last L={last[0:4]} R={last[4:8]}")
        line2 = (f"           mins L={mins[0:4]} R={mins[4:8]}  maxs L={maxs[0:4]} R={maxs[4:8]}")
        self.send_ui("data", line1)
        self.send_ui("data", line2)

    def _calibration_log_handler(self, sender, data: bytearray):
        try:
            message = data.decode("utf-8").strip()
            if message:
                self.send_ui("calibration_log", message)
        except Exception as e:
            self.send_ui("log", f"Calibration log decode error: {e}")


# ===== Main Window =====
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Load Cell BLE Monitor (Qt)")
        self.setMinimumSize(1100, 750)
        self.resize(1200, 820)

        self.q = queue.Queue()
        self.ble = BLEWorker(self.q)
        self.ble.start()

        self.device_map: Dict[str, str] = {}
        self.sample_rate_hz = 1000
        self.window_seconds = 10
        self.max_points = self.sample_rate_hz * self.window_seconds
        self.buffers = [deque(maxlen=self.max_points) for _ in range(8)]
        self.latest_vals = [0] * 8
        self.paused = False

        self.cop_enabled = False
        self.cop_trail_length = 100
        self.local_cop_trail = deque(maxlen=self.cop_trail_length)
        self.remote_cop_trail = deque(maxlen=self.cop_trail_length)
        self.latest_local_cop = (0.0, 0.0, 0.0, False)
        self.latest_remote_cop = (0.0, 0.0, 0.0, False)
        self.show_local_cop = True
        self.show_remote_cop = True
        self.show_cop_trail = True

        self.selected_channel_index = 0
        self.calibration_window = None

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(8)

        # Top bar
        top = QHBoxLayout()
        self.status_label = QLabel("Idle.")
        self.status_label.setStyleSheet("font-weight: bold;")
        top.addWidget(self.status_label)
        top.addStretch()
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self._connect_selected)
        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.clicked.connect(self.ble.disconnect)
        top.addWidget(self.btn_connect)
        top.addWidget(self.btn_disconnect)
        main_layout.addLayout(top)

        # Scan & select
        scan_grp = QGroupBox("Scan & Select Device")
        scan_layout = QHBoxLayout(scan_grp)
        scan_layout.addWidget(QLabel("Name filter (optional):"))
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText(DEFAULT_DEVICE_NAME)
        self.filter_edit.setMinimumWidth(200)
        scan_layout.addWidget(self.filter_edit)
        self.use_service_filter = QCheckBox("Filter by Service UUID")
        scan_layout.addWidget(self.use_service_filter)
        scan_layout.addStretch()
        self.btn_scan = QPushButton("Scan")
        self.btn_scan.clicked.connect(self._scan_devices)
        scan_layout.addWidget(self.btn_scan)
        main_layout.addWidget(scan_grp)

        sel_layout = QHBoxLayout()
        sel_layout.addWidget(QLabel("Select device:"))
        self.device_combo = QComboBox()
        self.device_combo.setMinimumWidth(400)
        self.device_combo.setEditable(False)
        sel_layout.addWidget(self.device_combo, 1)
        main_layout.addLayout(sel_layout)

        # Commands
        cmd_grp = QGroupBox("Commands")
        cmd_layout = QVBoxLayout(cmd_grp)
        for grp, cmds in COMMAND_GROUPS.items():
            row = QHBoxLayout()
            row.addWidget(QLabel(grp + ":"))
            for c in cmds:
                btn = QPushButton(c)
                btn.clicked.connect(lambda checked=False, cc=c: self.ble.send_command(cc))
                row.addWidget(btn)
            row.addStretch()
            cmd_layout.addLayout(row)
        main_layout.addWidget(cmd_grp)

        custom_layout = QHBoxLayout()
        custom_layout.addWidget(QLabel("Custom Command:"))
        self.custom_edit = QLineEdit()
        self.custom_edit.setPlaceholderText("Type command and press Send")
        custom_layout.addWidget(self.custom_edit, 1)
        btn_send = QPushButton("Send")
        btn_send.clicked.connect(self._send_custom)
        custom_layout.addWidget(btn_send)
        main_layout.addLayout(custom_layout)

        # Middle: values + plots
        self.middle_layout = QHBoxLayout()

        # Left: values panel
        values_grp = QGroupBox("Calibrated Values (10g units)")
        values_layout = QVBoxLayout(values_grp)
        self.val_labels: List[QLabel] = []
        for i, name in enumerate(CHANNEL_NAMES):
            row = QHBoxLayout()
            row.addWidget(QLabel(f"{name}:"))
            lbl = QLabel("0")
            lbl.setMinimumWidth(60)
            self.val_labels.append(lbl)
            row.addWidget(lbl)
            row.addStretch()
            values_layout.addLayout(row)

        values_layout.addWidget(self._hline())
        for label_text, key in [
            ("Local:", "local_sum"), ("Remote:", "remote_sum"), ("TOTAL:", "total_sum")
        ]:
            row = QHBoxLayout()
            row.addWidget(QLabel(label_text))
            l = QLabel("0")
            l.setMinimumWidth(60)
            setattr(self, key + "_label", l)
            row.addWidget(l)
            l2 = QLabel("(0.00 kg)")
            setattr(self, key.replace("_sum", "_kg") + "_label", l2)
            row.addWidget(l2)
            row.addStretch()
            values_layout.addLayout(row)
        self.total_sum_label.setStyleSheet("font-weight: bold; color: #0066cc;")
        self.total_kg_label.setStyleSheet("font-weight: bold; color: #0066cc;")

        values_layout.addWidget(self._hline())
        self.cop_check = QCheckBox("Enable CoP Chart")
        self.cop_check.toggled.connect(self._toggle_cop)
        values_layout.addWidget(self.cop_check)

        self.cop_info_grp = QGroupBox("Center of Pressure")
        cop_info_layout = QVBoxLayout(self.cop_info_grp)
        self.local_cop_label = QLabel("Local: X: 0.0, Y: 0.0 cm")
        self.remote_cop_label = QLabel("Remote: X: 0.0, Y: 0.0 cm")
        cop_info_layout.addWidget(self.local_cop_label)
        cop_info_layout.addWidget(self.remote_cop_label)
        values_layout.addWidget(self.cop_info_grp)

        self.middle_layout.addWidget(values_grp)

        # Time series plot (PyQtGraph)
        plot_grp = QGroupBox("Live Plot")
        plot_layout = QVBoxLayout(plot_grp)
        control_row = QHBoxLayout()
        control_row.addWidget(QLabel("Channel:"))
        self.channel_combo = QComboBox()
        self.channel_combo.addItems(CHANNEL_NAMES)
        self.channel_combo.currentIndexChanged.connect(self._on_channel_change)
        control_row.addWidget(self.channel_combo)
        control_row.addWidget(QLabel("Window (s):"))
        self.window_combo = QComboBox()
        self.window_combo.addItems(["5", "10", "30"])
        self.window_combo.setCurrentText("10")
        self.window_combo.currentTextChanged.connect(self._on_window_change)
        control_row.addWidget(self.window_combo)
        self.pause_check = QCheckBox("Pause")
        self.pause_check.toggled.connect(lambda v: setattr(self, "paused", v))
        control_row.addWidget(self.pause_check)
        control_row.addStretch()
        plot_layout.addLayout(control_row)

        pg.setConfigOptions(antialias=True, background="#f8f8f8", foreground="#333")
        self.plot_widget = pg.PlotWidget(title="Selected channel")
        self.plot_widget.setLabel("left", "Raw int16")
        self.plot_widget.setLabel("bottom", "Sample")
        # Default Y range for int16 so chart is usable before data arrives
        self.plot_widget.setYRange(-2000, 2000)
        self.plot_widget.enableAutoRange()
        self.curve = self.plot_widget.plot(pen=pg.mkPen(color="#1f77b4", width=2))
        plot_layout.addWidget(self.plot_widget)
        self.middle_layout.addWidget(plot_grp, 1)

        # CoP plot (optional, created when enabled)
        self.cop_plot_widget: Optional[pg.PlotWidget] = None
        self.cop_plot_grp: Optional[QGroupBox] = None
        self.cop_curve_local = None
        self.cop_curve_remote = None
        self.cop_trail_local = None
        self.cop_trail_remote = None
        self.cop_scatter_local = None
        self.cop_scatter_remote = None

        main_layout.addLayout(self.middle_layout)

        # Log
        log_grp = QGroupBox("Log / Output")
        log_layout = QVBoxLayout(log_grp)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(180)
        self.log_text.setFont(QFont("Monospace", 9))
        log_layout.addWidget(self.log_text)
        main_layout.addWidget(log_grp)

        # Timers
        self._queue_timer = QTimer(self)
        self._queue_timer.timeout.connect(self._pump_queue)
        self._queue_timer.start(50)
        self._plot_timer = QTimer(self)
        self._plot_timer.timeout.connect(self._update_plot)
        self._plot_timer.start(100)

        self._log("Ready. Scan, select a device, Connect, then use commands.")

    def _hline(self):
        f = QFrame()
        f.setFrameShape(QFrame.Shape.HLine)
        f.setFrameShadow(QFrame.Shadow.Sunken)
        return f

    def _toggle_cop(self, checked: bool):
        self.cop_enabled = checked
        if checked:
            self._create_cop_plot()
            self.cop_info_grp.setVisible(True)
            self._log("CoP chart enabled.")
        else:
            self._destroy_cop_plot()
            self.cop_info_grp.setVisible(False)
            self._log("CoP chart disabled.")

    def _create_cop_plot(self):
        if self.cop_plot_grp is not None:
            return
        self.cop_plot_grp = QGroupBox("Center of Pressure (CoP)")
        layout = QVBoxLayout(self.cop_plot_grp)
        ctrl = QHBoxLayout()
        cb_local = QCheckBox("Local")
        cb_local.setChecked(True)
        cb_local.toggled.connect(lambda v: setattr(self, "show_local_cop", v))
        ctrl.addWidget(cb_local)
        cb_remote = QCheckBox("Remote")
        cb_remote.setChecked(True)
        cb_remote.toggled.connect(lambda v: setattr(self, "show_remote_cop", v))
        ctrl.addWidget(cb_remote)
        cb_trail = QCheckBox("Show Trail")
        cb_trail.setChecked(True)
        cb_trail.toggled.connect(lambda v: setattr(self, "show_cop_trail", v))
        ctrl.addWidget(cb_trail)
        btn_clear = QPushButton("Clear Trail")
        btn_clear.clicked.connect(self._clear_cop_trail)
        ctrl.addWidget(btn_clear)
        ctrl.addStretch()
        layout.addLayout(ctrl)
        self.cop_plot_widget = pg.PlotWidget(title="Center of Pressure")
        self.cop_plot_widget.setAspectLocked(True)
        half_w = PLATE_WIDTH_CM / 2
        half_l = PLATE_LENGTH_CM / 2
        margin = 2.0
        self.cop_plot_widget.setXRange(-half_w - margin, half_w + margin)
        self.cop_plot_widget.setYRange(-half_l - margin, half_l + margin)
        self.cop_plot_widget.setLabel("left", "Y (cm)")
        self.cop_plot_widget.setLabel("bottom", "X (cm)")
        # Plate outline
        plate_x = [-half_w, half_w, half_w, -half_w, -half_w]
        plate_y = [half_l, half_l, -half_l, -half_l, half_l]
        self.cop_plot_widget.plot(plate_x, plate_y, pen=pg.mkPen("k", width=2))
        self.cop_trail_local = self.cop_plot_widget.plot(pen=pg.mkPen("g", width=1))
        self.cop_trail_remote = self.cop_plot_widget.plot(pen=pg.mkPen("r", width=1))
        self.cop_scatter_local = self.cop_plot_widget.plot(
            symbol="o", symbolSize=12, pen=None, brush="g"
        )
        self.cop_scatter_remote = self.cop_plot_widget.plot(
            symbol="o", symbolSize=12, pen=None, brush="r"
        )
        layout.addWidget(self.cop_plot_widget)
        self.middle_layout.addWidget(self.cop_plot_grp, 1)

    def _destroy_cop_plot(self):
        if self.cop_plot_grp is not None:
            self.cop_plot_grp.setParent(None)
            self.cop_plot_grp.deleteLater()
            self.cop_plot_grp = None
            self.cop_plot_widget = None
            self.cop_curve_local = None
            self.cop_curve_remote = None
            self.cop_trail_local = None
            self.cop_trail_remote = None
            self.cop_scatter_local = None
            self.cop_scatter_remote = None
            self.local_cop_trail.clear()
            self.remote_cop_trail.clear()

    def _clear_cop_trail(self):
        self.local_cop_trail.clear()
        self.remote_cop_trail.clear()
        self._log("CoP trail cleared.")

    def _on_channel_change(self, index: int):
        self.selected_channel_index = index

    def _on_window_change(self, s: str):
        try:
            self.window_seconds = int(s)
            self.max_points = self.sample_rate_hz * self.window_seconds
            for ch in range(8):
                old = list(self.buffers[ch])
                self.buffers[ch] = deque(old[-self.max_points:], maxlen=self.max_points)
            self._log(f"Plot window set to {self.window_seconds}s ({self.max_points} samples).")
        except ValueError:
            pass

    def _scan_devices(self):
        self._log("Scanning for 5 seconds...")
        self.ble.scan(
            self.use_service_filter.isChecked(),
            self.filter_edit.text().strip() or "",
        )

    def _connect_selected(self):
        disp = self.device_combo.currentText().strip()
        if not disp or disp == "<no devices>":
            QMessageBox.information(
                self, "Select device", "Please scan and select a device first."
            )
            return
        addr = self.device_map.get(disp)
        if not addr:
            QMessageBox.critical(self, "Error", "Selected entry has no address.")
            return
        self._log(f"Connecting to {disp} ...")
        self.ble.connect(addr)

    def _send_custom(self):
        cmd = self.custom_edit.text().strip()
        if cmd:
            self.ble.send_command(cmd)

    def _pump_queue(self):
        try:
            while True:
                msg = self.q.get_nowait()
                kind = msg.get("kind")
                payload = msg.get("payload")
                if kind == "status":
                    self.status_label.setText(payload)
                    self._log(f"[status] {payload}")
                elif kind == "log":
                    self._log(str(payload))
                elif kind == "data":
                    self._print_line(str(payload))
                elif kind == "devices":
                    self._populate_devices(payload or [])
                elif kind == "samples":
                    self._ingest_samples(payload or [])
                elif kind == "calibration_log":
                    self._handle_calibration_log(str(payload))
        except queue.Empty:
            pass

    def _populate_devices(self, items: List[Dict]):
        self.device_map.clear()
        self.device_combo.clear()
        display_list = []
        for it in items:
            name = it.get("name") or "(unknown)"
            addr = it.get("address") or "?"
            rssi = it.get("rssi")
            disp = f"{name} ({addr})" + (f"  [RSSI {rssi}]" if rssi is not None else "")
            self.device_map[disp] = addr
            display_list.append(disp)
        if not display_list:
            display_list = ["<no devices>"]
            self.device_map["<no devices>"] = ""
        self.device_combo.addItems(display_list)
        if display_list:
            self.device_combo.setCurrentIndex(0)

    def _ingest_samples(self, samples: List[List[int]]):
        for row in samples:
            for ch in range(8):
                self.buffers[ch].append(row[ch])
                self.latest_vals[ch] = row[ch]
            if self.cop_enabled:
                local_cop = calculate_cop(
                    max(0, row[0]), max(0, row[1]), max(0, row[2]), max(0, row[3])
                )
                if local_cop[3]:
                    self.local_cop_trail.append((local_cop[0], local_cop[1]))
                self.latest_local_cop = local_cop
                remote_cop = calculate_cop(
                    max(0, row[4]), max(0, row[5]), max(0, row[6]), max(0, row[7])
                )
                if remote_cop[3]:
                    self.remote_cop_trail.append((remote_cop[0], remote_cop[1]))
                self.latest_remote_cop = remote_cop

        for ch in range(8):
            self.val_labels[ch].setText(str(self.latest_vals[ch]))
        local_sum = sum(self.latest_vals[0:4])
        remote_sum = sum(self.latest_vals[4:8])
        total_sum = local_sum + remote_sum
        self.local_sum_label.setText(str(local_sum))
        self.local_kg_label.setText(f"({local_sum / 100:.2f} kg)")
        self.remote_sum_label.setText(str(remote_sum))
        self.remote_kg_label.setText(f"({remote_sum / 100:.2f} kg)")
        self.total_sum_label.setText(str(total_sum))
        self.total_kg_label.setText(f"({total_sum / 100:.2f} kg)")

        if self.cop_enabled:
            if self.latest_local_cop[3]:
                self.local_cop_label.setText(
                    f"Local: X:{self.latest_local_cop[0]:+.1f}, Y:{self.latest_local_cop[1]:+.1f} cm"
                )
            else:
                self.local_cop_label.setText("Local: -- (no load) --")
            if self.latest_remote_cop[3]:
                self.remote_cop_label.setText(
                    f"Remote: X:{self.latest_remote_cop[0]:+.1f}, Y:{self.latest_remote_cop[1]:+.1f} cm"
                )
            else:
                self.remote_cop_label.setText("Remote: -- (no load) --")

    def _update_plot(self):
        if self.paused:
            return
        ch_idx = self.selected_channel_index
        y = list(self.buffers[ch_idx])
        if y:
            if _HAS_NUMPY:
                x = np.arange(len(y), dtype=np.float64)
                y_arr = np.array(y, dtype=np.float64)
                self.curve.setData(x, y_arr)
            else:
                x = list(range(len(y)))
                self.curve.setData(x, y)
            # Force view to show the data (fixes chart not updating on macOS)
            vb = self.plot_widget.getViewBox()
            vb.updateAutoRange()
        if self.cop_enabled and self.cop_plot_widget is not None:
            self._update_cop_plot()

    def _update_cop_plot(self):
        if self.cop_plot_widget is None or self.cop_trail_local is None:
            return
        try:
            if self.show_cop_trail and len(self.local_cop_trail) > 1:
                trail = list(self.local_cop_trail)
                self.cop_trail_local.setData([p[0] for p in trail], [p[1] for p in trail])
            else:
                self.cop_trail_local.setData([], [])
            if self.show_cop_trail and len(self.remote_cop_trail) > 1:
                trail = list(self.remote_cop_trail)
                self.cop_trail_remote.setData([p[0] for p in trail], [p[1] for p in trail])
            else:
                self.cop_trail_remote.setData([], [])

            if self.show_local_cop and self.latest_local_cop[3]:
                x, y = self.latest_local_cop[0], self.latest_local_cop[1]
                self.cop_scatter_local.setData([x], [y])
                self.cop_scatter_local.setVisible(True)
            else:
                self.cop_scatter_local.setData([], [])
                self.cop_scatter_local.setVisible(False)
            if self.show_remote_cop and self.latest_remote_cop[3]:
                x, y = self.latest_remote_cop[0], self.latest_remote_cop[1]
                self.cop_scatter_remote.setData([x], [y])
                self.cop_scatter_remote.setVisible(True)
            else:
                self.cop_scatter_remote.setData([], [])
                self.cop_scatter_remote.setVisible(False)
        except Exception as e:
            self._log(f"CoP plot error: {e}")

    def _print_line(self, s: str):
        self.log_text.append(s)
        sb = self.log_text.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _log(self, s: str):
        ts = time.strftime("%H:%M:%S")
        self._print_line(f"[{ts}] {s}")

    def _handle_calibration_log(self, message: str):
        if self.calibration_window:
            self.calibration_window.add_message(message)

    def closeEvent(self, event):
        self.ble.stop()
        time.sleep(0.15)
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

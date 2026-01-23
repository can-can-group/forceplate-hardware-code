# ble_loadcell_gui.py
# GUI for ESP32 Load Cell BLE Server with:
# - Scan & device selection
# - Connect/Disconnect
# - Commands + custom command
# - Raw values (8 channels) live monitor
# - Live graph for a selected channel (L1..L4, R5..R8)
# - Center of Pressure (CoP) visualization for force plates
#
# Spec: Data notifications contain 1..10 samples/packet, each sample = 8 x int16 LE
#       Order: local[4], remote[4] => L1..L4, R5..R8
#
# Force Plate Geometry (for CoP calculation):
#   LC1 ----22cm---- LC2
#    |               |
#   35cm   CENTER   35cm
#    |               |
#   LC4 ----22cm---- LC3

import asyncio
import struct
import threading
import queue
import sys
import time
from collections import deque
from typing import Optional, List, Dict

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except Exception:
    print("Tkinter not available. Please install/enable Tk support.")
    sys.exit(1)

from bleak import BleakClient, BleakScanner, BleakError

# Matplotlib embed
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# ==== UUIDs (from your doc) ====
SERVICE_UUID       = "12345678-1234-1234-1234-123456789abc"
DATA_CHAR_UUID     = "87654321-4321-4321-4321-cba987654321"  # Notify + Read
COMMAND_CHAR_UUID  = "11111111-2222-3333-4444-555555555555"  # Write
# Optional characteristic (older firmware used this). Current `esp32_ble_slave.ino`
# does NOT create it, so we must not require it for connecting.
CALIBRATION_LOG_CHAR_UUID: Optional[str] = None  # e.g. "22222222-3333-4444-5555-666666666666"

DEFAULT_DEVICE_NAME = "LoadCell_BLE_Server"

# ==== Command sets (matching current esp32_ble_slave.ino) ====
COMMAND_GROUPS = {
    "Local": ["START", "STOP", "RESTART", "RESET"],
    "Remote": ["REMOTE_START", "REMOTE_STOP", "REMOTE_RESTART", "REMOTE_RESET"],
    "Dual": ["ALL_START", "ALL_STOP", "ALL_RESTART", "ALL_RESET"],
    "Ping": ["LOCAL_PING", "REMOTE_PING"],
    "System": ["STATS", "RESET_STATS"],
}

CHANNEL_NAMES = ["L1", "L2", "L3", "L4", "R5", "R6", "R7", "R8"]

# Force plate geometry (cm) - used for CoP calculation
# Layout:
#   LC1(0) ----WIDTH---- LC2(1)
#    |                    |
#   LENGTH    CENTER     LENGTH
#    |                    |
#   LC4(3) ----WIDTH---- LC3(2)
PLATE_WIDTH_CM = 22.0   # Distance LC1-LC2 and LC3-LC4
PLATE_LENGTH_CM = 35.0  # Distance LC1-LC4 and LC2-LC3

# ===== CoP Calculator =====
def calculate_cop(f1: float, f2: float, f3: float, f4: float, 
                  width: float = PLATE_WIDTH_CM, length: float = PLATE_LENGTH_CM):
    """
    Calculate Center of Pressure (CoP) from 4 corner load cell readings.
    
    Layout (looking down at plate):
        LC1(f1) ----width---- LC2(f2)
         |                     |
        length    CENTER      length
         |                     |
        LC4(f4) ----width---- LC3(f3)
    
    Origin (0,0) is at center of plate.
    X-axis: positive = right (toward LC2/LC3)
    Y-axis: positive = up (toward LC1/LC2)
    
    Returns: (cop_x, cop_y, total_force, is_valid)
    """
    total = f1 + f2 + f3 + f4
    
    # Minimum threshold to avoid division by zero and noise
    if total < 10:  # Less than 100g total
        return (0.0, 0.0, total, False)
    
    # CoP calculation using moment balance
    # X: right side (LC2+LC3) vs left side (LC1+LC4)
    cop_x = (width / 2.0) * ((f2 + f3) - (f1 + f4)) / total
    
    # Y: top side (LC1+LC2) vs bottom side (LC4+LC3)
    cop_y = (length / 2.0) * ((f1 + f2) - (f4 + f3)) / total
    
    return (cop_x, cop_y, total, True)


# ===== Packet parser =====
def parse_packet(data: bytes):
    """Return {"sample_count": int, "samples": [[8 int16], ...]}"""
    if not data or len(data) < 1:
        return {"sample_count": 0, "samples": []}
    sample_count = data[0]
    if sample_count > 10:
        # Corrupt/partial packet; ignore.
        return {"sample_count": 0, "samples": []}
    expected_len = 1 + sample_count * 16
    if len(data) < expected_len or sample_count == 0:
        return {"sample_count": 0, "samples": []}

    samples: List[List[int]] = []
    for i in range(sample_count):
        off = 1 + i * 16
        local4  = struct.unpack_from("<4h", data, off)
        remote4 = struct.unpack_from("<4h", data, off + 8)
        samples.append(list(local4) + list(remote4))
    return {"sample_count": sample_count, "samples": samples}

# ===== BLE worker thread (async) =====
class BLEWorker:
    def __init__(self, ui_q: queue.Queue):
        self.client: Optional[BleakClient] = None
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        self.ui_q = ui_q
        self.selected_address: Optional[str] = None
        self.packet_counter = 0
        self._subscribed_notify_uuids = set()

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
            if self.client and self.client.is_connected:
                await self.client.disconnect()
                self.send_ui("status", "Disconnected.")
        except Exception as e:
            self.send_ui("log", f"Error during disconnect: {e}")
        finally:
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

    # ----- public: called from UI thread -----
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

    # ----- async tasks -----
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

        results.sort(key=lambda x: (x["rssi"] is None, -(x["rssi"] or -999), x["name"]))
        self.send_ui("devices", results)
        self.send_ui("status", f"Scan complete. Found {len(results)} device(s).")

    async def _connect_task(self):
        if not self.selected_address:
            self.send_ui("status", "No device selected.")
            return
        if self.client and self.client.is_connected:
            self.send_ui("status", "Already connected.")
            return

        self.client = BleakClient(self.selected_address)
        try:
            await self.client.connect()
            self.send_ui("status", f"Connected: {self.selected_address}")
            self._subscribed_notify_uuids.clear()

            # Discover what this device actually exposes (important because the
            # GUI used to assume extra characteristics that are not present now).
            services = await self.client.get_services()
            available = set()
            try:
                for svc in services:
                    for ch in getattr(svc, "characteristics", []) or []:
                        if getattr(ch, "uuid", None):
                            available.add(ch.uuid.lower())
            except Exception:
                # If service parsing fails, we'll just attempt operations below.
                available = set()

            # Data notifications (required)
            if not available or DATA_CHAR_UUID.lower() in available:
                await self.client.start_notify(DATA_CHAR_UUID, self._notification_handler)
                self._subscribed_notify_uuids.add(DATA_CHAR_UUID)
            else:
                raise BleakError(f"Device missing DATA characteristic {DATA_CHAR_UUID}")

            # Optional calibration log notifications (only if configured and present)
            if CALIBRATION_LOG_CHAR_UUID:
                if (not available) or (CALIBRATION_LOG_CHAR_UUID.lower() in available):
                    await self.client.start_notify(CALIBRATION_LOG_CHAR_UUID, self._calibration_log_handler)
                    self._subscribed_notify_uuids.add(CALIBRATION_LOG_CHAR_UUID)
                else:
                    self.send_ui("log", f"(info) Calibration log characteristic not found; skipping.")

            self.packet_counter = 0
            self.send_ui("status", "Notifications enabled.")
        except BleakError as e:
            self.send_ui("status", f"Connect error: {e}")
            self.client = None
        except Exception as e:
            self.send_ui("status", f"Connect error: {e}")
            self.client = None

    async def _disconnect_task(self):
        try:
            if self.client and self.client.is_connected:
                for uuid in list(self._subscribed_notify_uuids):
                    try:
                        await self.client.stop_notify(uuid)
                    except Exception:
                        pass
                await self.client.disconnect()
                self.send_ui("status", "Disconnected.")
        except Exception as e:
            self.send_ui("log", f"Disconnect error: {e}")
        finally:
            self._subscribed_notify_uuids.clear()
            self.client = None

    async def _send_command_task(self, cmd: str):
        cmd = (cmd or "").strip()
        if not cmd:
            self.send_ui("log", "Empty command.")
            return
        if not self.client or not self.client.is_connected:
            self.send_ui("log", "Not connected.")
            return
        try:
            # Slave supports both Write and Write Without Response; prefer without response.
            await self.client.write_gatt_char(COMMAND_CHAR_UUID, cmd.encode("utf-8"), response=False)
            self.send_ui("log", f">> {cmd}")
        except Exception as e:
            self.send_ui("log", f"Command error: {e}")

    # ----- notification handler -----
    def _notification_handler(self, sender, data: bytearray):
        parsed = parse_packet(bytes(data))
        if not parsed["sample_count"]:
            return

        self.packet_counter += 1
        samples = parsed["samples"]  # list of [8 ints]
        # push raw samples to UI (for plotting + raw values)
        self.send_ui("samples", samples)

        # also send a terse textual summary (like before)
        mins = [min(s[ch] for s in samples) for ch in range(8)]
        maxs = [max(s[ch] for s in samples) for ch in range(8)]
        first = samples[0]; last = samples[-1]
        line1 = (f"[pkt {self.packet_counter:06d}] samples={parsed['sample_count']}  "
                 f"first L={first[0:4]} R={first[4:8]}  "
                 f"last L={last[0:4]} R={last[4:8]}")
        line2 = (f"           mins L={mins[0:4]} R={mins[4:8]}  "
                 f"maxs L={maxs[0:4]} R={maxs[4:8]}")
        self.send_ui("data", line1)
        self.send_ui("data", line2)

    def _calibration_log_handler(self, sender, data: bytearray):
        try:
            # Decode calibration log message
            message = data.decode('utf-8').strip()
            if message:
                self.send_ui("calibration_log", message)
        except Exception as e:
            self.send_ui("log", f"Calibration log decode error: {e}")

# ===== Calibration Window =====
class CalibrationWindow(tk.Toplevel):
    def __init__(self, parent, ble_worker):
        super().__init__(parent)
        self.parent = parent
        self.ble = ble_worker
        self.title("Force Plate Calibration Monitor")
        self.geometry("1000x700")
        
        # Make window modal
        self.transient(parent)
        self.grab_set()
        
        # Calibration log queue
        self.cal_log_queue = queue.Queue()
        
        # Current calibration status
        self.cal_status_var = tk.StringVar(value="No calibration active")
        self.cal_type_var = tk.StringVar(value="None")
        
        # Build UI
        self._build_ui()
        
        # Start processing calibration logs
        self.after(100, self._process_calibration_logs)
        
        # Center the window
        self._center_window()
    
    def _center_window(self):
        """Center the calibration window on the parent"""
        self.update_idletasks()
        x = (self.winfo_screenwidth() // 2) - (1000 // 2)
        y = (self.winfo_screenheight() // 2) - (700 // 2)
        self.geometry(f"1000x700+{x}+{y}")
    
    def _build_ui(self):
        """Build the calibration window UI"""
        # Main container
        main_frame = ttk.Frame(self)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Top section - Status and Controls
        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill="x", pady=(0, 10))
        
        # Status display
        status_frame = ttk.LabelFrame(top_frame, text="Calibration Status")
        status_frame.pack(side="left", fill="x", expand=True, padx=(0, 10))
        
        self.status_label = ttk.Label(status_frame, textvariable=self.cal_status_var, 
                                    font=("Arial", 12, "bold"))
        self.status_label.pack(pady=5)
        
        self.type_label = ttk.Label(status_frame, textvariable=self.cal_type_var, 
                                  font=("Arial", 10))
        self.type_label.pack()
        
        # Control buttons
        control_frame = ttk.LabelFrame(top_frame, text="Calibration Controls")
        control_frame.pack(side="right", fill="y")
        
        # Start calibration buttons
        ttk.Button(control_frame, text="Start Local Cal", 
                  command=lambda: self._send_command("LOCAL_AUTO_CAL"),
                  style="Accent.TButton").pack(pady=2, padx=5, fill="x")
        
        ttk.Button(control_frame, text="Start Remote Cal", 
                  command=lambda: self._send_command("REMOTE_AUTO_CAL"),
                  style="Accent.TButton").pack(pady=2, padx=5, fill="x")
        
        # Control flow buttons
        ttk.Button(control_frame, text="Continue", 
                  command=lambda: self._send_command("CONTINUE"),
                  style="Success.TButton").pack(pady=2, padx=5, fill="x")
        
        ttk.Button(control_frame, text="Skip Step", 
                  command=lambda: self._send_command("SKIP"),
                  style="Warning.TButton").pack(pady=2, padx=5, fill="x")
        
        ttk.Button(control_frame, text="Abort", 
                  command=lambda: self._send_command("ABORT"),
                  style="Danger.TButton").pack(pady=2, padx=5, fill="x")
        
        # Status check button
        ttk.Button(control_frame, text="Check Status", 
                  command=lambda: self._send_command("CAL_STATUS")).pack(pady=2, padx=5, fill="x")
        
        # Main log area
        log_frame = ttk.LabelFrame(main_frame, text="Calibration Log")
        log_frame.pack(fill="both", expand=True)
        
        # Create text widget with scrollbar
        text_frame = ttk.Frame(log_frame)
        text_frame.pack(fill="both", expand=True, padx=5, pady=5)
        
        self.log_text = tk.Text(text_frame, 
                               font=("Consolas", 11),
                               wrap=tk.WORD,
                               bg="#1e1e1e",
                               fg="#ffffff",
                               insertbackground="#ffffff",
                               selectbackground="#404040",
                               state="disabled")
        
        # Scrollbar
        scrollbar = ttk.Scrollbar(text_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        # Pack text and scrollbar
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Bottom controls
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(fill="x", pady=(10, 0))
        
        # Clear log button
        ttk.Button(bottom_frame, text="Clear Log", 
                  command=self._clear_log).pack(side="left")
        
        # Auto-scroll toggle
        self.auto_scroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(bottom_frame, text="Auto-scroll", 
                       variable=self.auto_scroll_var).pack(side="left", padx=(20, 0))
        
        # Close button
        ttk.Button(bottom_frame, text="Close", 
                  command=self.destroy).pack(side="right")
    
    def _send_command(self, command):
        """Send command to BLE worker"""
        if self.ble and self.ble.client and self.ble.client.is_connected:
            self.ble.send_command(command)
            self._add_log(f"📤 Sent: {command}", "command")
        else:
            self._add_log("❌ Not connected to device", "error")
    
    def _add_log(self, message, log_type="info"):
        """Add message to the calibration log"""
        timestamp = time.strftime("%H:%M:%S")
        
        # Color coding based on log type
        colors = {
            "info": "#ffffff",
            "success": "#4CAF50",
            "error": "#F44336",
            "warning": "#FF9800",
            "command": "#2196F3",
            "prompt": "#9C27B0",
            "header": "#FFC107"
        }
        
        color = colors.get(log_type, "#ffffff")
        
        # Enable text widget for editing
        self.log_text.config(state="normal")
        
        # Insert timestamp and message
        self.log_text.insert("end", f"[{timestamp}] ", "timestamp")
        self.log_text.insert("end", f"{message}\n", log_type)
        
        # Configure tags for colors
        self.log_text.tag_configure("timestamp", foreground="#888888")
        self.log_text.tag_configure(log_type, foreground=color)
        
        # Auto-scroll if enabled
        if self.auto_scroll_var.get():
            self.log_text.see("end")
        
        # Disable text widget to prevent editing
        self.log_text.config(state="disabled")
    
    def _clear_log(self):
        """Clear the calibration log"""
        self.log_text.config(state="normal")
        self.log_text.delete(1.0, "end")
        self.log_text.config(state="disabled")
    
    def _process_calibration_logs(self):
        """Process calibration log messages from BLE"""
        try:
            while True:
                message = self.cal_log_queue.get_nowait()
                self._handle_calibration_message(message)
        except queue.Empty:
            pass
        
        # Schedule next check
        self.after(100, self._process_calibration_logs)
    
    def _handle_calibration_message(self, message):
        """Handle calibration message and update UI"""
        # Extract message type and content
        message_type = ""
        content = message
        
        if message.startswith("CALIBRATION_START:"):
            message_type = "calibration_start"
            content = message.replace("CALIBRATION_START: ", "")
        elif message.startswith("PROMPT:"):
            message_type = "prompt"
            content = message.replace("PROMPT: ", "")
        elif message.startswith("SUCCESS:"):
            message_type = "success"
            content = message.replace("SUCCESS: ", "")
        elif message.startswith("ERROR:"):
            message_type = "error"
            content = message.replace("ERROR: ", "")
        elif message.startswith("HEADER:"):
            message_type = "header"
            content = message.replace("HEADER: ", "")
        elif message.startswith("INFO:"):
            message_type = "info"
            content = message.replace("INFO: ", "")
        else:
            message_type = "info"
            content = message
        
        # Update status based on message type
        if message_type == "calibration_start":
            if "LOCAL" in content:
                self.cal_status_var.set("🟢 LOCAL Calibration Active")
                self.cal_type_var.set("Local Force Plate")
            elif "REMOTE" in content:
                self.cal_status_var.set("🔵 REMOTE Calibration Active")
                self.cal_type_var.set("Remote Force Plate")
            self._add_log(content, "success")
        elif message_type == "prompt":
            self.cal_status_var.set("❓ Waiting for user input")
            self._add_log(content, "prompt")
        elif message_type == "success":
            self.cal_status_var.set("✅ Step completed successfully")
            self._add_log(content, "success")
        elif message_type == "error":
            self.cal_status_var.set("❌ Error occurred")
            self._add_log(content, "error")
        elif message_type == "header":
            # Extract step information for status
            if "STEP A" in content:
                self.cal_status_var.set("📋 Step A: ADC Calibration")
            elif "STEP B" in content:
                self.cal_status_var.set("📋 Step B: Load Cell Calibration")
            elif "STEP C" in content:
                self.cal_status_var.set("📋 Step C: Matrix Calibration")
            elif "COMPLETE" in content:
                self.cal_status_var.set("🎉 Calibration Complete!")
            self._add_log(content, "header")
        else:
            # Info or generic message
            self._add_log(content, "info")
    
    def add_calibration_message(self, message):
        """Add calibration message to the queue (called from main app)"""
        self.cal_log_queue.put(message)

# ===== Main Tk app =====
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ESP32 Load Cell BLE Monitor")
        self.geometry("1200x820")

        # Cross-thread queue
        self.q = queue.Queue()

        # BLE worker
        self.ble = BLEWorker(self.q)
        self.ble.start()

        # Device list cache
        self.device_map: Dict[str, str] = {}

        # Ring buffers for plots (per channel)
        self.sample_rate_hz = 1000  # overall per channel rate
        self.window_seconds = tk.IntVar(value=10)
        self.max_points = self.sample_rate_hz * self.window_seconds.get()
        self.buffers = [deque(maxlen=self.max_points) for _ in range(8)]
        self.sample_index = 0  # monotonically increasing index for x-axis
        self.paused = tk.BooleanVar(value=False)

        # Latest raw values
        self.latest_vals = [0]*8
        
        # CoP tracking (disabled by default for performance)
        self.cop_enabled = tk.BooleanVar(value=False)
        self.cop_trail_length = 100  # Number of CoP points to show as trail
        self.local_cop_trail = deque(maxlen=self.cop_trail_length)
        self.remote_cop_trail = deque(maxlen=self.cop_trail_length)
        self.latest_local_cop = (0.0, 0.0, 0.0, False)  # (x, y, force, valid)
        self.latest_remote_cop = (0.0, 0.0, 0.0, False)
        self.cop_panel = None  # Will be created when enabled

        # Selected channel
        self.selected_channel = tk.StringVar(value=CHANNEL_NAMES[0])  # default L1
        
        # Calibration window reference
        self.calibration_window = None
        
        # Configure button styles
        style = ttk.Style()
        style.configure("Accent.TButton", foreground="white", background="#0078d4")
        style.configure("Success.TButton", foreground="white", background="#28a745")
        style.configure("Warning.TButton", foreground="white", background="#ffc107")
        style.configure("Danger.TButton", foreground="white", background="#dc3545")

        # ------------- UI Layout -------------
        self._build_top_bar()
        self._build_scan_select()
        self._build_commands()
        self._build_monitor_and_plot()
        self._build_calibration_log()
        self._build_output()

        # Timers
        self.after(50, self._pump_queue)
        self.after(100, self._update_plot)

        self._log("Ready. Scan, select a device, Connect, then use commands.")

    # ---- UI sections ----
    def _build_top_bar(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=10)

        self.status_var = tk.StringVar(value="Idle.")
        ttk.Label(top, textvariable=self.status_var).pack(side="left")

        btns = ttk.Frame(top)
        btns.pack(side="right")
        ttk.Button(btns, text="Disconnect", command=self.ble.disconnect).pack(side="right", padx=4)
        ttk.Button(btns, text="Connect", command=self._connect_selected).pack(side="right", padx=4)

    def _build_scan_select(self):
        scan_frame = ttk.LabelFrame(self, text="Scan & Select Device")
        scan_frame.pack(fill="x", padx=10, pady=6)

        left = ttk.Frame(scan_frame)
        left.pack(side="left", fill="x", expand=True, padx=6, pady=6)

        self.filter_var = tk.StringVar(value=DEFAULT_DEVICE_NAME)
        ttk.Label(left, text="Name filter (optional):").pack(side="left")
        self.filter_entry = ttk.Entry(left, textvariable=self.filter_var, width=28)
        self.filter_entry.pack(side="left", padx=6)

        self.use_service_filter = tk.BooleanVar(value=False)
        ttk.Checkbutton(left, text="Filter by Service UUID", variable=self.use_service_filter).pack(side="left", padx=6)

        ttk.Button(scan_frame, text="Scan", command=self._scan_devices).pack(side="right", padx=6)

        sel_row = ttk.Frame(self)
        sel_row.pack(fill="x", padx=10, pady=6)
        ttk.Label(sel_row, text="Select device:").pack(side="left")
        self.device_combo = ttk.Combobox(sel_row, state="readonly", width=90)
        self.device_combo.pack(side="left", fill="x", expand=True, padx=6)

    def _build_commands(self):
        cmd_frame = ttk.LabelFrame(self, text="Commands")
        cmd_frame.pack(fill="x", padx=10, pady=6)

        for grp, cmds in COMMAND_GROUPS.items():
            row = ttk.Frame(cmd_frame)
            row.pack(fill="x", padx=4, pady=2)
            ttk.Label(row, text=grp, width=14).pack(side="left")
            for c in cmds:
                ttk.Button(row, text=c, command=lambda cc=c: self.ble.send_command(cc)).pack(side="left", padx=2, pady=2)
        
        # Calibration UI is kept for older firmware, but current esp32_ble_slave.ino
        # does not stream calibration logs over BLE (no characteristic), so we hide it.

        custom = ttk.Frame(self)
        custom.pack(fill="x", padx=10, pady=6)
        ttk.Label(custom, text="Custom Command:").pack(side="left")
        self.custom_entry = ttk.Entry(custom)
        self.custom_entry.pack(side="left", fill="x", expand=True, padx=6)
        ttk.Button(custom, text="Send", command=self._send_custom).pack(side="left")

    def _build_monitor_and_plot(self):
        holder = ttk.Frame(self)
        holder.pack(fill="both", expand=True, padx=10, pady=6)

        # Left: Raw values table
        left = ttk.LabelFrame(holder, text="Calibrated Values (10g units)")
        left.pack(side="left", fill="y", padx=6, pady=6)

        self.val_labels = []
        for i, name in enumerate(CHANNEL_NAMES):
            row = ttk.Frame(left)
            row.pack(fill="x", padx=6, pady=2)
            ttk.Label(row, text=f"{name}:", width=6).pack(side="left")
            lbl = ttk.Label(row, text="0", width=12)
            lbl.pack(side="left")
            self.val_labels.append(lbl)
        
        # Separator
        ttk.Separator(left, orient="horizontal").pack(fill="x", padx=6, pady=8)
        
        # Local Sum (L1+L2+L3+L4)
        local_sum_row = ttk.Frame(left)
        local_sum_row.pack(fill="x", padx=6, pady=2)
        ttk.Label(local_sum_row, text="Local:", width=6, font=("TkDefaultFont", 9, "bold")).pack(side="left")
        self.local_sum_label = ttk.Label(local_sum_row, text="0", width=12, font=("TkDefaultFont", 9, "bold"))
        self.local_sum_label.pack(side="left")
        self.local_kg_label = ttk.Label(local_sum_row, text="(0.00 kg)", width=12)
        self.local_kg_label.pack(side="left")
        
        # Remote Sum (R5+R6+R7+R8)
        remote_sum_row = ttk.Frame(left)
        remote_sum_row.pack(fill="x", padx=6, pady=2)
        ttk.Label(remote_sum_row, text="Remote:", width=6, font=("TkDefaultFont", 9, "bold")).pack(side="left")
        self.remote_sum_label = ttk.Label(remote_sum_row, text="0", width=12, font=("TkDefaultFont", 9, "bold"))
        self.remote_sum_label.pack(side="left")
        self.remote_kg_label = ttk.Label(remote_sum_row, text="(0.00 kg)", width=12)
        self.remote_kg_label.pack(side="left")
        
        # Separator
        ttk.Separator(left, orient="horizontal").pack(fill="x", padx=6, pady=8)
        
        # Total Sum (all 8 channels)
        total_sum_row = ttk.Frame(left)
        total_sum_row.pack(fill="x", padx=6, pady=2)
        ttk.Label(total_sum_row, text="TOTAL:", width=6, font=("TkDefaultFont", 10, "bold")).pack(side="left")
        self.total_sum_label = ttk.Label(total_sum_row, text="0", width=12, font=("TkDefaultFont", 10, "bold"), foreground="blue")
        self.total_sum_label.pack(side="left")
        self.total_kg_label = ttk.Label(total_sum_row, text="(0.00 kg)", width=12, font=("TkDefaultFont", 10, "bold"), foreground="blue")
        self.total_kg_label.pack(side="left")
        
        # Separator
        ttk.Separator(left, orient="horizontal").pack(fill="x", padx=6, pady=8)
        
        # CoP toggle and coordinates display
        cop_toggle_frame = ttk.Frame(left)
        cop_toggle_frame.pack(fill="x", padx=6, pady=4)
        ttk.Checkbutton(cop_toggle_frame, text="Enable CoP Chart", 
                       variable=self.cop_enabled, 
                       command=self._toggle_cop_panel).pack(side="left")
        
        # CoP coordinates (always shown when CoP enabled)
        self.cop_info_frame = ttk.LabelFrame(left, text="Center of Pressure")
        
        # Local CoP
        local_cop_row = ttk.Frame(self.cop_info_frame)
        local_cop_row.pack(fill="x", padx=4, pady=2)
        ttk.Label(local_cop_row, text="Local:", width=6).pack(side="left")
        self.local_cop_label = ttk.Label(local_cop_row, text="X: 0.0, Y: 0.0 cm", width=20)
        self.local_cop_label.pack(side="left")
        
        # Remote CoP
        remote_cop_row = ttk.Frame(self.cop_info_frame)
        remote_cop_row.pack(fill="x", padx=4, pady=2)
        ttk.Label(remote_cop_row, text="Remote:", width=6).pack(side="left")
        self.remote_cop_label = ttk.Label(remote_cop_row, text="X: 0.0, Y: 0.0 cm", width=20)
        self.remote_cop_label.pack(side="left")
        
        # Store reference to holder for CoP panel
        self.plot_holder = holder
        
        # CoP panel variables (created on demand)
        self.show_local_cop = tk.BooleanVar(value=True)
        self.show_remote_cop = tk.BooleanVar(value=True)
        self.show_cop_trail = tk.BooleanVar(value=True)
        self.cop_fig = None
        self.cop_ax = None
        self.cop_canvas = None

        # Right: Time series Plot
        self.time_series_frame = ttk.LabelFrame(holder, text="Live Plot")
        self.time_series_frame.pack(side="left", fill="both", expand=True, padx=6, pady=6)
        right = self.time_series_frame

        control = ttk.Frame(right)
        control.pack(fill="x", padx=6, pady=6)

        ttk.Label(control, text="Channel:").pack(side="left")
        self.channel_combo = ttk.Combobox(control, state="readonly", values=CHANNEL_NAMES, width=6,
                                          textvariable=self.selected_channel)
        self.channel_combo.pack(side="left", padx=6)

        ttk.Label(control, text="Window (s):").pack(side="left")
        window_box = ttk.Combobox(control, state="readonly", values=[5,10,30], width=5,
                                  textvariable=self.window_seconds)
        window_box.pack(side="left", padx=6)
        window_box.bind("<<ComboboxSelected>>", self._on_window_change)

        ttk.Checkbutton(control, text="Pause", variable=self.paused).pack(side="left", padx=12)

        # Matplotlib figure
        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Selected channel")
        self.ax.set_xlabel("Sample")
        self.ax.set_ylabel("Raw int16")
        self.line, = self.ax.plot([], [])  # default color/style

        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
    
    def _setup_cop_axes(self):
        """Setup the CoP plot axes with force plate outline"""
        if self.cop_ax is None:
            return
        self.cop_ax.clear()
        
        # Set axis limits based on plate dimensions (with some margin)
        margin = 2.0
        half_w = PLATE_WIDTH_CM / 2
        half_l = PLATE_LENGTH_CM / 2
        self.cop_ax.set_xlim(-half_w - margin, half_w + margin)
        self.cop_ax.set_ylim(-half_l - margin, half_l + margin)
        
        # Draw force plate outline
        plate_x = [-half_w, half_w, half_w, -half_w, -half_w]
        plate_y = [half_l, half_l, -half_l, -half_l, half_l]
        self.cop_ax.plot(plate_x, plate_y, 'k-', linewidth=2, label='Plate')
        
        # Draw center crosshair
        self.cop_ax.axhline(y=0, color='gray', linestyle='--', linewidth=0.5, alpha=0.5)
        self.cop_ax.axvline(x=0, color='gray', linestyle='--', linewidth=0.5, alpha=0.5)
        
        # Mark load cell positions
        lc_positions = [
            (-half_w, half_l, 'LC1'),   # Top-left
            (half_w, half_l, 'LC2'),    # Top-right
            (half_w, -half_l, 'LC3'),   # Bottom-right
            (-half_w, -half_l, 'LC4'),  # Bottom-left
        ]
        for x, y, name in lc_positions:
            self.cop_ax.plot(x, y, 'ks', markersize=8)
            self.cop_ax.annotate(name, (x, y), textcoords="offset points", 
                               xytext=(5, 5), fontsize=8)
        
        self.cop_ax.set_xlabel('X (cm) - Left/Right')
        self.cop_ax.set_ylabel('Y (cm) - Back/Front')
        self.cop_ax.set_title('Center of Pressure')
        self.cop_ax.set_aspect('equal')
        self.cop_ax.grid(True, alpha=0.3)
    
    def _clear_cop_trail(self):
        """Clear the CoP trail history"""
        self.local_cop_trail.clear()
        self.remote_cop_trail.clear()
        self._log("CoP trail cleared.")
    
    def _toggle_cop_panel(self):
        """Toggle the CoP panel on/off"""
        if self.cop_enabled.get():
            self._create_cop_panel()
            self.cop_info_frame.pack(fill="x", padx=6, pady=4)
            self._log("CoP chart enabled.")
        else:
            self._destroy_cop_panel()
            self.cop_info_frame.pack_forget()
            self._log("CoP chart disabled.")
    
    def _create_cop_panel(self):
        """Create the CoP visualization panel"""
        if self.cop_panel is not None:
            return  # Already exists
        
        # Create CoP panel
        self.cop_panel = ttk.LabelFrame(self.plot_holder, text="Center of Pressure (CoP)")
        # Insert before the time series plot (pack after values, before plot)
        self.cop_panel.pack(side="left", fill="both", expand=True, padx=6, pady=6, before=self.time_series_frame)
        
        # CoP controls
        cop_control = ttk.Frame(self.cop_panel)
        cop_control.pack(fill="x", padx=6, pady=6)
        
        ttk.Checkbutton(cop_control, text="Local", variable=self.show_local_cop).pack(side="left", padx=4)
        ttk.Checkbutton(cop_control, text="Remote", variable=self.show_remote_cop).pack(side="left", padx=4)
        ttk.Checkbutton(cop_control, text="Show Trail", variable=self.show_cop_trail).pack(side="left", padx=4)
        ttk.Button(cop_control, text="Clear Trail", command=self._clear_cop_trail).pack(side="left", padx=4)
        
        # CoP matplotlib figure
        self.cop_fig = Figure(figsize=(4, 4), dpi=100)
        self.cop_ax = self.cop_fig.add_subplot(111)
        self._setup_cop_axes()
        
        self.cop_canvas = FigureCanvasTkAgg(self.cop_fig, master=self.cop_panel)
        self.cop_canvas.get_tk_widget().pack(fill="both", expand=True)
    
    def _destroy_cop_panel(self):
        """Destroy the CoP visualization panel"""
        if self.cop_panel is not None:
            self.cop_panel.destroy()
            self.cop_panel = None
            self.cop_fig = None
            self.cop_ax = None
            self.cop_canvas = None
            # Clear trails
            self.local_cop_trail.clear()
            self.remote_cop_trail.clear()

    def _build_calibration_log(self):
        # Current esp32_ble_slave.ino has no calibration-log characteristic, so we
        # don't show the calibration log panel (prevents confusion).
        self.cal_log_text = None
        self.cal_status_var = tk.StringVar(value="Calibration log not available on this firmware")

    def _build_output(self):
        out_frame = ttk.LabelFrame(self, text="Log / Output")
        out_frame.pack(fill="both", expand=True, padx=10, pady=6)

        self.text = tk.Text(out_frame, wrap="none", height=10)
        self.text.pack(side="left", fill="both", expand=True)
        yscroll = ttk.Scrollbar(out_frame, command=self.text.yview)
        yscroll.pack(side="right", fill="y")
        self.text.configure(yscrollcommand=yscroll.set)

    # ---- actions ----
    def _scan_devices(self):
        self._log("Scanning for 5 seconds...")
        self.ble.scan(self.use_service_filter.get(), self.filter_entry.get())

    def _connect_selected(self):
        disp = (self.device_combo.get() or "").strip()
        if not disp or disp == "<no devices>":
            messagebox.showinfo("Select device", "Please scan and select a device first.")
            return
        addr = self.device_map.get(disp)
        if not addr:
            messagebox.showerror("Error", "Selected entry has no address.")
            return
        self._log(f"Connecting to {disp} ...")
        self.ble.connect(addr)

    def _send_custom(self):
        cmd = self.custom_entry.get().strip()
        if cmd:
            self.ble.send_command(cmd)

    def _on_window_change(self, _evt=None):
        # Resize ring buffers
        new_secs = int(self.window_seconds.get())
        self.max_points = self.sample_rate_hz * new_secs
        # rebuild buffers preserving recent tail
        for ch in range(8):
            old = list(self.buffers[ch])
            self.buffers[ch] = deque(old[-self.max_points:], maxlen=self.max_points)
        self._log(f"Plot window set to {new_secs}s ({self.max_points} samples).")

    # ---- queue + UI updates ----
    def _pump_queue(self):
        try:
            while True:
                msg = self.q.get_nowait()
                kind = msg.get("kind")
                payload = msg.get("payload")
                if kind == "status":
                    self.status_var.set(payload)
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
        self.after(50, self._pump_queue)

    def _populate_devices(self, items: List[Dict]):
        self.device_map.clear()
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
            self.device_map[display_list[0]] = ""
        self.device_combo["values"] = display_list
        if display_list:
            self.device_combo.set(display_list[0])

    def _ingest_samples(self, samples: List[List[int]]):
        # samples: list of 8-int rows
        cop_enabled = self.cop_enabled.get()
        
        for row in samples:
            self.sample_index += 1
            for ch in range(8):
                self.buffers[ch].append(row[ch])
                self.latest_vals[ch] = row[ch]
            
            # Only calculate CoP when enabled (performance optimization)
            if cop_enabled:
                # Calculate CoP for each sample (for trail)
                # Local plate: L1(0), L2(1), L3(2), L4(3)
                local_cop = calculate_cop(
                    max(0, row[0]), max(0, row[1]), max(0, row[2]), max(0, row[3])
                )
                if local_cop[3]:  # is_valid
                    self.local_cop_trail.append((local_cop[0], local_cop[1]))
                self.latest_local_cop = local_cop
                
                # Remote plate: R5(4), R6(5), R7(6), R8(7)
                remote_cop = calculate_cop(
                    max(0, row[4]), max(0, row[5]), max(0, row[6]), max(0, row[7])
                )
                if remote_cop[3]:  # is_valid
                    self.remote_cop_trail.append((remote_cop[0], remote_cop[1]))
                self.latest_remote_cop = remote_cop
        
        # update raw values panel
        for ch in range(8):
            self.val_labels[ch].configure(text=str(self.latest_vals[ch]))
        
        # Update sum labels
        # Local sum (L1+L2+L3+L4 = channels 0-3)
        local_sum = sum(self.latest_vals[0:4])
        self.local_sum_label.configure(text=str(local_sum))
        self.local_kg_label.configure(text=f"({local_sum / 100:.2f} kg)")
        
        # Remote sum (R5+R6+R7+R8 = channels 4-7)
        remote_sum = sum(self.latest_vals[4:8])
        self.remote_sum_label.configure(text=str(remote_sum))
        self.remote_kg_label.configure(text=f"({remote_sum / 100:.2f} kg)")
        
        # Total sum (all 8 channels)
        total_sum = local_sum + remote_sum
        self.total_sum_label.configure(text=str(total_sum))
        self.total_kg_label.configure(text=f"({total_sum / 100:.2f} kg)")
        
        # Update CoP coordinate labels only when enabled
        if cop_enabled:
            if self.latest_local_cop[3]:
                self.local_cop_label.configure(
                    text=f"X:{self.latest_local_cop[0]:+.1f}, Y:{self.latest_local_cop[1]:+.1f} cm"
                )
            else:
                self.local_cop_label.configure(text="-- (no load) --")
            
            if self.latest_remote_cop[3]:
                self.remote_cop_label.configure(
                    text=f"X:{self.latest_remote_cop[0]:+.1f}, Y:{self.latest_remote_cop[1]:+.1f} cm"
                )
            else:
                self.remote_cop_label.configure(text="-- (no load) --")

    def _update_plot(self):
        try:
            if not self.paused.get():
                # Update time series plot
                ch_name = self.selected_channel.get()
                ch_idx = CHANNEL_NAMES.index(ch_name) if ch_name in CHANNEL_NAMES else 0
                y = list(self.buffers[ch_idx])
                if y:
                    x = list(range(len(y)))  # sample indices in window
                    self.line.set_data(x, y)
                    self.ax.relim()
                    self.ax.autoscale_view()
                self.canvas.draw_idle()
                
                # Update CoP plot only when enabled
                if self.cop_enabled.get() and self.cop_panel is not None:
                    self._update_cop_plot()
        except Exception as e:
            self._log(f"Plot update error: {e}")
        # refresh rate ~10 Hz is enough (lighter on CPU); tune as needed
        self.after(100, self._update_plot)
    
    def _update_cop_plot(self):
        """Update the Center of Pressure visualization"""
        if self.cop_ax is None or self.cop_canvas is None:
            return
        
        try:
            # Redraw the base (plate outline, labels, etc.)
            self._setup_cop_axes()
            
            # Draw trails if enabled
            if self.show_cop_trail.get():
                # Local trail (green, fading)
                if self.show_local_cop.get() and len(self.local_cop_trail) > 1:
                    trail = list(self.local_cop_trail)
                    x_trail = [p[0] for p in trail]
                    y_trail = [p[1] for p in trail]
                    # Fade effect: older points are more transparent
                    n = len(trail)
                    for i in range(n - 1):
                        alpha = 0.1 + 0.4 * (i / n)
                        self.cop_ax.plot(x_trail[i:i+2], y_trail[i:i+2], 
                                        'g-', linewidth=1, alpha=alpha)
                
                # Remote trail (red, fading)
                if self.show_remote_cop.get() and len(self.remote_cop_trail) > 1:
                    trail = list(self.remote_cop_trail)
                    x_trail = [p[0] for p in trail]
                    y_trail = [p[1] for p in trail]
                    n = len(trail)
                    for i in range(n - 1):
                        alpha = 0.1 + 0.4 * (i / n)
                        self.cop_ax.plot(x_trail[i:i+2], y_trail[i:i+2], 
                                        'r-', linewidth=1, alpha=alpha)
            
            # Draw current CoP positions
            # Local CoP (green dot)
            if self.show_local_cop.get() and self.latest_local_cop[3]:
                x, y = self.latest_local_cop[0], self.latest_local_cop[1]
                force_kg = self.latest_local_cop[2] / 100.0
                # Size based on force (min 10, max 30)
                size = min(30, max(10, force_kg * 0.5))
                self.cop_ax.plot(x, y, 'go', markersize=size, alpha=0.8, label=f'Local ({force_kg:.1f}kg)')
                self.cop_ax.plot(x, y, 'g+', markersize=size * 0.6, markeredgewidth=2)
            
            # Remote CoP (red dot)
            if self.show_remote_cop.get() and self.latest_remote_cop[3]:
                x, y = self.latest_remote_cop[0], self.latest_remote_cop[1]
                force_kg = self.latest_remote_cop[2] / 100.0
                size = min(30, max(10, force_kg * 0.5))
                self.cop_ax.plot(x, y, 'ro', markersize=size, alpha=0.8, label=f'Remote ({force_kg:.1f}kg)')
                self.cop_ax.plot(x, y, 'r+', markersize=size * 0.6, markeredgewidth=2)
            
            # Add legend if there's data
            if (self.show_local_cop.get() and self.latest_local_cop[3]) or \
               (self.show_remote_cop.get() and self.latest_remote_cop[3]):
                self.cop_ax.legend(loc='upper right', fontsize=8)
            
            self.cop_canvas.draw_idle()
        except Exception as e:
            self._log(f"CoP plot error: {e}")

    # ---- helpers ----
    def _print_line(self, s: str):
        self.text.insert("end", s + "\n")
        self.text.see("end")

    def _log(self, s: str):
        ts = time.strftime("%H:%M:%S")
        self._print_line(f"[{ts}] {s}")

    def _handle_calibration_log(self, message: str):
        """Handle calibration log messages from BLE"""
        ts = time.strftime("%H:%M:%S")
        
        # Add to calibration log text area (preserve formatting)
        if self.cal_log_text is not None:
            self.cal_log_text.insert("end", f"[{ts}] {message}\n")
            self.cal_log_text.see("end")
        
        # Update calibration status based on message type
        if "CALIBRATION_START" in message:
            if "LOCAL" in message:
                self.cal_status_var.set("🟢 LOCAL Calibration Active")
            elif "REMOTE" in message:
                self.cal_status_var.set("🔵 REMOTE Calibration Active")
        elif "PROMPT:" in message:
            self.cal_status_var.set("❓ Waiting for user input")
        elif "SUCCESS:" in message:
            self.cal_status_var.set("✅ Step completed successfully")
        elif "ERROR:" in message:
            self.cal_status_var.set("❌ Error occurred")
        elif "HEADER:" in message:
            # Extract step information from header
            if "STEP A" in message:
                self.cal_status_var.set("📋 Step A: ADC Calibration")
            elif "STEP B" in message:
                self.cal_status_var.set("📋 Step B: Load Cell Calibration")
            elif "STEP C" in message:
                self.cal_status_var.set("📋 Step C: Matrix Calibration")
            elif "COMPLETE" in message:
                self.cal_status_var.set("🎉 Calibration Complete!")
        
        # Forward message to calibration window if open
        if self.calibration_window:
            self.calibration_window.add_calibration_message(message)

    def _open_calibration_window(self):
        """Open the calibration monitoring window"""
        if self.calibration_window is None or not self.calibration_window.winfo_exists():
            self.calibration_window = CalibrationWindow(self, self.ble)
        else:
            # Bring existing window to front
            self.calibration_window.lift()
            self.calibration_window.focus_force()

    def destroy(self):
        try:
            self.ble.stop()
        except Exception:
            pass
        # give BLE thread a breath
        time.sleep(0.15)
        super().destroy()

if __name__ == "__main__":
    App().mainloop()

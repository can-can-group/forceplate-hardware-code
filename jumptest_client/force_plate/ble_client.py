"""
BLE client for force plate: scan, connect, START/STOP, and stream samples into a buffer.
"""

import asyncio
import threading
from collections import deque
from typing import Optional, List, Callable, Any

from bleak import BleakClient, BleakScanner, BleakError

from .packet_parser import parse_packet

try:
    import config as _config
except ImportError:
    from .. import config as _config


class ForcePlateBLEClient:
    """
    Async BLE client that runs in a background thread. Exposes:
    - scan() -> list of {name, address, rssi}
    - connect(address), disconnect()
    - send_command(cmd) e.g. "ALL_START", "ALL_STOP"
    - Recording buffer: samples appended in notification handler; consumer reads via get_buffer_copy() / clear_buffer().
    """

    def __init__(
        self,
        on_status: Optional[Callable[[str], None]] = None,
        on_log: Optional[Callable[[str], None]] = None,
        max_buffer_samples: Optional[int] = None,
    ):
        self.on_status = on_status or (lambda _: None)
        self.on_log = on_log or (lambda _: None)
        self._max_buffer = max_buffer_samples  # None = unbounded
        self._buffer: deque = deque(maxlen=max_buffer_samples) if max_buffer_samples else deque()
        self._lock = threading.Lock()
        self._client: Optional[BleakClient] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._selected_address: Optional[str] = None
        self._notify_uuids: set = set()

    def _send_status(self, msg: str) -> None:
        self.on_status(msg)

    def _send_log(self, msg: str) -> None:
        self.on_log(msg)

    def start(self) -> None:
        """Start the async event loop in a background thread."""
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the loop and disconnect."""
        if self._loop:
            asyncio.run_coroutine_threadsafe(self._shutdown(), self._loop)

    async def _shutdown(self) -> None:
        try:
            if self._client and self._client.is_connected:
                for uuid in list(self._notify_uuids):
                    try:
                        await self._client.stop_notify(uuid)
                    except Exception:
                        pass
                await self._client.disconnect()
                self._send_status("Disconnected.")
        except Exception as e:
            self._send_status(f"Disconnect error: {e}")
        finally:
            self._notify_uuids.clear()
            self._client = None
            await asyncio.sleep(0.05)
            if self._loop and self._loop.is_running():
                self._loop.stop()

    def _run_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._send_status("BLE thread started.")
        try:
            self._loop.run_forever()
        except Exception as e:
            self._send_log(f"Loop error: {e}")
        finally:
            self._send_status("BLE thread stopped.")

    def scan(self, name_hint: Optional[str] = None) -> None:
        """Discover devices; results reported via on_status and stored for connect."""
        if not self._loop:
            self._send_status("BLE thread not started.")
            return
        hint = (name_hint or _config.DEFAULT_DEVICE_NAME).strip().lower()
        asyncio.run_coroutine_threadsafe(self._scan_task(hint), self._loop)

    async def _scan_task(self, name_hint: str) -> None:
        self._send_status("Scanning...")
        try:
            devices = await BleakScanner.discover(timeout=5.0)
        except Exception as e:
            self._send_status(f"Scan error: {e}")
            return
        results = []
        for d in devices:
            nm = (d.name or "").strip()
            if name_hint and name_hint not in nm.lower():
                continue
            results.append({
                "name": nm or "(unknown)",
                "address": d.address,
                "rssi": getattr(d, "rssi", None),
            })
        results.sort(key=lambda x: (x["rssi"] is None, -(x["rssi"] or -999), x["name"]))
        self._scan_results = results
        self._send_status(f"Found {len(results)} device(s).")

    _scan_results: List[dict] = []

    def get_scan_results(self) -> List[dict]:
        """Return last scan results (list of {name, address, rssi})."""
        return getattr(self, "_scan_results", [])

    def connect(self, address: Optional[str]) -> None:
        """Connect to device by address. Subscribe to Data characteristic."""
        self._selected_address = address
        if self._loop:
            asyncio.run_coroutine_threadsafe(self._connect_task(), self._loop)
        else:
            self._send_status("BLE thread not started.")

    async def _connect_task(self) -> None:
        if not self._selected_address:
            self._send_status("No device selected.")
            return
        if self._client and self._client.is_connected:
            self._send_status("Already connected.")
            return
        self._client = BleakClient(self._selected_address)
        try:
            await self._client.connect()
            self._send_status(f"Connected: {self._selected_address}")
            self._notify_uuids.clear()
            services = await self._client.get_services()
            available = set()
            for svc in services:
                for ch in getattr(svc, "characteristics", []) or []:
                    if getattr(ch, "uuid", None):
                        available.add(ch.uuid.lower())
            data_uuid = _config.DATA_CHAR_UUID.lower()
            if not available or data_uuid in available:
                await self._client.start_notify(_config.DATA_CHAR_UUID, self._on_notification)
                self._notify_uuids.add(_config.DATA_CHAR_UUID)
            else:
                raise BleakError(f"Device missing DATA characteristic {_config.DATA_CHAR_UUID}")
            self._send_status("Notifications enabled.")
        except BleakError as e:
            self._send_status(f"Connect error: {e}")
            self._client = None
        except Exception as e:
            self._send_status(f"Connect error: {e}")
            self._client = None

    def _on_notification(self, sender: Any, data: bytearray) -> None:
        """Called from BLE thread: parse packet and append samples to buffer."""
        parsed = parse_packet(bytes(data))
        if not parsed["sample_count"]:
            return
        with self._lock:
            for row in parsed["samples"]:
                self._buffer.append(list(row))
            if self._max_buffer and len(self._buffer) > self._max_buffer:
                # deque with maxlen auto-drops left; otherwise trim manually
                pass  # already using maxlen in __init__ if _max_buffer set

    def disconnect(self) -> None:
        if self._loop:
            asyncio.run_coroutine_threadsafe(self._disconnect_task(), self._loop)

    async def _disconnect_task(self) -> None:
        try:
            if self._client and self._client.is_connected:
                for uuid in list(self._notify_uuids):
                    try:
                        await self._client.stop_notify(uuid)
                    except Exception:
                        pass
                await self._client.disconnect()
                self._send_status("Disconnected.")
        except Exception as e:
            self._send_log(f"Disconnect error: {e}")
        finally:
            self._notify_uuids.clear()
            self._client = None

    def send_command(self, cmd: str) -> None:
        """Send a command string (e.g. ALL_START, ALL_STOP) to the device."""
        cmd = (cmd or "").strip()
        if not cmd:
            self._send_log("Empty command.")
            return
        if not self._loop:
            self._send_log("BLE thread not started.")
            return
        asyncio.run_coroutine_threadsafe(self._send_command_task(cmd), self._loop)

    async def _send_command_task(self, cmd: str) -> None:
        if not self._client or not self._client.is_connected:
            self._send_log("Not connected.")
            return
        try:
            await self._client.write_gatt_char(
                _config.COMMAND_CHAR_UUID, cmd.encode("utf-8"), response=False
            )
            self._send_log(f">> {cmd}")
        except Exception as e:
            self._send_log(f"Command error: {e}")

    def get_buffer_copy(self) -> List[List[int]]:
        """Return a copy of the current recording buffer (list of 8-int samples)."""
        with self._lock:
            return [list(row) for row in self._buffer]

    def clear_buffer(self) -> None:
        """Clear the recording buffer."""
        with self._lock:
            self._buffer.clear()

    def buffer_length(self) -> int:
        with self._lock:
            return len(self._buffer)

    @property
    def is_connected(self) -> bool:
        return self._client is not None and self._client.is_connected

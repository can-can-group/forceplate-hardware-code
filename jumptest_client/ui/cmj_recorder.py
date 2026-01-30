"""
CMJ recorder UI: connect to force plate, record with live plot, run CMJ analysis.
"""

import sys
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from typing import Optional, List

# Ensure jumptest_client root is on path when run as script
from pathlib import Path
_root = Path(__file__).resolve().parent.parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

import config
from force_plate import ForcePlateBLEClient, buffer_to_cmj_input
from analysers import CMJ_Analyser

# Matplotlib embedding
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

SAMPLE_RATE_HZ = getattr(config, "SAMPLE_RATE_HZ", 1000)
PLOT_WINDOW_SEC = 10  # show last N seconds in live plot
PLOT_UPDATE_MS = 100


class CMJRecorderApp:
    def __init__(self):
        self.win = tk.Tk()
        self.win.title("CMJ Test – Force Plate Client")
        self.win.geometry("900x700")
        self.win.minsize(700, 500)

        self._recording = False
        self._ble: Optional[ForcePlateBLEClient] = None
        self._device_var = tk.StringVar()
        self._status_var = tk.StringVar(value="Not connected")
        self._last_result = None

        self._build_ui()
        self._start_ble()
        self._schedule_plot_update()

    def _build_ui(self) -> None:
        main = ttk.Frame(self.win, padding=8)
        main.pack(fill=tk.BOTH, expand=True)

        # --- Connection ---
        conn = ttk.LabelFrame(main, text="Connection", padding=6)
        conn.pack(fill=tk.X, pady=(0, 6))
        row0 = ttk.Frame(conn)
        row0.pack(fill=tk.X)
        ttk.Button(row0, text="Scan", command=self._on_scan).pack(side=tk.LEFT, padx=(0, 4))
        self._device_combo = ttk.Combobox(row0, textvariable=self._device_var, width=40, state="readonly")
        self._device_combo.pack(side=tk.LEFT, padx=4)
        ttk.Button(row0, text="Connect", command=self._on_connect).pack(side=tk.LEFT, padx=4)
        ttk.Button(row0, text="Disconnect", command=self._on_disconnect).pack(side=tk.LEFT, padx=4)
        ttk.Label(conn, textvariable=self._status_var).pack(anchor=tk.W)

        # --- Record / Analyse ---
        rec = ttk.LabelFrame(main, text="Record & Analyse", padding=6)
        rec.pack(fill=tk.X, pady=(0, 6))
        row1 = ttk.Frame(rec)
        row1.pack(fill=tk.X)
        self._btn_start_rec = ttk.Button(row1, text="Start recording", command=self._on_start_recording)
        self._btn_start_rec.pack(side=tk.LEFT, padx=(0, 4))
        self._btn_stop_rec = ttk.Button(row1, text="Stop recording", command=self._on_stop_recording, state=tk.DISABLED)
        self._btn_stop_rec.pack(side=tk.LEFT, padx=4)
        self._btn_run_cmj = ttk.Button(row1, text="Run CMJ", command=self._on_run_cmj)
        self._btn_run_cmj.pack(side=tk.LEFT, padx=4)
        ttk.Button(row1, text="Clear / New trial", command=self._on_clear).pack(side=tk.LEFT, padx=4)
        self._samples_label = ttk.Label(rec, text="Samples: 0")
        self._samples_label.pack(anchor=tk.W)

        # --- Live plot ---
        plot_frame = ttk.LabelFrame(main, text="Live force (total)", padding=6)
        plot_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 6))
        self._fig = Figure(figsize=(6, 3), dpi=100)
        self._ax = self._fig.add_subplot(111)
        self._ax.set_xlabel("Time (s)")
        self._ax.set_ylabel("Total force (10g units)")
        self._line, = self._ax.plot([], [], "b-", lw=1)
        self._fig.tight_layout()
        self._canvas = FigureCanvasTkAgg(self._fig, master=plot_frame)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # --- Results ---
        res_frame = ttk.LabelFrame(main, text="CMJ results", padding=6)
        res_frame.pack(fill=tk.BOTH, expand=True)
        self._results_text = scrolledtext.ScrolledText(res_frame, height=10, wrap=tk.WORD, state=tk.DISABLED)
        self._results_text.pack(fill=tk.BOTH, expand=True)

        # --- Log ---
        log_frame = ttk.LabelFrame(main, text="Log", padding=6)
        log_frame.pack(fill=tk.X)
        self._log_text = scrolledtext.ScrolledText(log_frame, height=3, wrap=tk.WORD, state=tk.DISABLED)
        self._log_text.pack(fill=tk.X)

    def _start_ble(self) -> None:
        self._ble = ForcePlateBLEClient(
            on_status=lambda s: self.win.after(0, lambda: self._status_var.set(s)),
            on_log=lambda s: self.win.after(0, lambda: self._log(s)),
        )
        self._ble.start()

    def _log(self, msg: str) -> None:
        self._log_text.configure(state=tk.NORMAL)
        self._log_text.insert(tk.END, msg + "\n")
        self._log_text.see(tk.END)
        self._log_text.configure(state=tk.DISABLED)

    def _on_scan(self) -> None:
        self._status_var.set("Scanning...")
        self._ble.scan(name_hint=config.DEFAULT_DEVICE_NAME)
        self.win.after(6000, self._apply_scan_results)

    def _apply_scan_results(self) -> None:
        results = self._ble.get_scan_results()
        self._scan_addresses = [r["address"] for r in results]
        addrs = [f"{r['name']} ({r['address']})" for r in results]
        self._device_combo["values"] = addrs
        if addrs:
            self._device_combo.set(addrs[0])
            self._device_combo.current(0)

    _scan_addresses: List[str] = []

    def _on_connect(self) -> None:
        if not self._scan_addresses:
            messagebox.showwarning("Connect", "Run Scan first and wait for devices.")
            return
        sel = self._device_combo.current()
        if sel < 0:
            sel = 0
        if sel >= len(self._scan_addresses):
            sel = 0
        addr = self._scan_addresses[sel]
        self._ble.connect(addr)

    def _on_disconnect(self) -> None:
        self._ble.disconnect()

    def _on_start_recording(self) -> None:
        if not self._ble.is_connected:
            messagebox.showwarning("Record", "Connect to the force plate first.")
            return
        self._ble.clear_buffer()
        self._ble.send_command("ALL_START")
        self._recording = True
        self._btn_start_rec.configure(state=tk.DISABLED)
        self._btn_stop_rec.configure(state=tk.NORMAL)
        self._status_var.set("Recording...")

    def _on_stop_recording(self) -> None:
        self._ble.send_command("ALL_STOP")
        self._recording = False
        self._btn_start_rec.configure(state=tk.NORMAL)
        self._btn_stop_rec.configure(state=tk.DISABLED)
        n = self._ble.buffer_length()
        self._samples_label.configure(text=f"Samples: {n}")
        self._status_var.set("Stopped. Run CMJ or start a new recording.")

    def _on_run_cmj(self) -> None:
        samples = self._ble.get_buffer_copy()
        if not samples:
            messagebox.showwarning("CMJ", "No data. Record a jump first (Start recording → jump → Stop recording).")
            return
        try:
            input_data = buffer_to_cmj_input(samples, sample_rate_hz=SAMPLE_RATE_HZ)
            result = CMJ_Analyser.compute_cmj_test(input_data)
            self._last_result = result
            self._show_result(result)
        except Exception as e:
            messagebox.showerror("CMJ analysis", str(e))
            self._log(f"CMJ error: {e}")

    def _show_result(self, result: dict) -> None:
        self._results_text.configure(state=tk.NORMAL)
        self._results_text.delete(1.0, tk.END)
        lines = []
        for p in result.get("points", []):
            lines.append(f"  {p['point_type']}: t={p['time']}, F={p['total_force']}")
        lines.append("")
        for a in result.get("areas", []):
            lines.append(f"  {a['phase']}: {a['start_time']:.3f} – {a['end_time']:.3f} s")
        lines.append("")
        ana = result.get("analysis", {})
        for k, v in ana.items():
            if v is not None:
                lines.append(f"  {k}: {v}")
        self._results_text.insert(tk.END, "\n".join(lines))
        self._results_text.configure(state=tk.DISABLED)

    def _on_clear(self) -> None:
        self._ble.clear_buffer()
        self._samples_label.configure(text="Samples: 0")
        self._line.set_data([], [])
        self._ax.relim()
        self._ax.autoscale_view()
        self._canvas.draw_idle()
        self._results_text.configure(state=tk.NORMAL)
        self._results_text.delete(1.0, tk.END)
        self._results_text.configure(state=tk.DISABLED)
        self._last_result = None
        self._status_var.set("Buffer cleared. Start a new recording when ready.")

    def _schedule_plot_update(self) -> None:
        self._update_plot()
        self.win.after(PLOT_UPDATE_MS, self._schedule_plot_update)

    def _update_plot(self) -> None:
        try:
            samples = self._ble.get_buffer_copy() if self._ble else []
            if not samples:
                self._line.set_data([], [])
            else:
                n = len(samples)
                max_pts = int(PLOT_WINDOW_SEC * SAMPLE_RATE_HZ)
                if n > max_pts:
                    samples = samples[-max_pts:]
                    n = len(samples)
                t = [i / SAMPLE_RATE_HZ for i in range(n)]
                total = [sum(s[0:4]) + sum(s[4:8]) for s in samples]
                self._line.set_data(t, total)
                self._ax.relim()
                self._ax.autoscale_view()
            self._canvas.draw_idle()
        except Exception:
            pass

    def run(self) -> None:
        self.win.protocol("WM_DELETE_WINDOW", self._on_close)
        self.win.mainloop()

    def _on_close(self) -> None:
        if self._ble:
            self._ble.stop()
        self.win.destroy()


def main() -> None:
    app = CMJRecorderApp()
    app.run()


if __name__ == "__main__":
    main()

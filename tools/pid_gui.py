#!/usr/bin/env python3
"""
Simple PID serial GUI for your ESP32 SBR project.

Requires: pyserial
Install: pip install pyserial
Run: python pid_gui.py
"""

import threading, queue, time, re
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports

READ_TIMEOUT = 0.1
# telemetry throttle (seconds) when Show raw is OFF
TELEMETRY_THROTTLE = 0.05  # 20 Hz

class SerialReader(threading.Thread):
    def __init__(self, ser, out_q, stop_event):
        super().__init__(daemon=True)
        self.ser = ser
        self.out_q = out_q
        self.stop_event = stop_event

    def run(self):
        buf = b""
        while not self.stop_event.is_set():
            try:
                data = self.ser.read(256)
                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        try:
                            text = line.decode('utf-8', errors='ignore').strip()
                        except:
                            text = repr(line)
                        self.out_q.put(text)
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.out_q.put(f"<SERIAL ERROR> {e}")
                break

class PIDGui:
    def __init__(self, root):
        self.root = root
        root.title("SBR PID Control")
        frm = ttk.Frame(root, padding=8)
        frm.grid(sticky="nsew")
        # Port selection
        ttk.Label(frm, text="Serial:").grid(row=0, column=0, sticky="w")
        self.port_cb = ttk.Combobox(frm, width=20, values=self._list_ports())
        self.port_cb.grid(row=0, column=1, sticky="w")
        ttk.Button(frm, text="Refresh", command=self._refresh_ports).grid(row=0, column=2, sticky="w")
        self.b_connect = ttk.Button(frm, text="Connect", command=self.toggle_connect)
        self.b_connect.grid(row=0, column=3, sticky="w")
        # PID entries
        ttk.Label(frm, text="KP").grid(row=1, column=0)
        self.e_kp = ttk.Entry(frm, width=10)
        self.e_kp.grid(row=1, column=1, sticky="w")
        ttk.Label(frm, text="KI").grid(row=1, column=2)
        self.e_ki = ttk.Entry(frm, width=10)
        self.e_ki.grid(row=1, column=3, sticky="w")
        ttk.Label(frm, text="KD").grid(row=1, column=4)
        self.e_kd = ttk.Entry(frm, width=10)
        self.e_kd.grid(row=1, column=5, sticky="w")
        ttk.Button(frm, text="GET PID", command=self.cmd_get_pid).grid(row=2, column=1, sticky="w")
        ttk.Button(frm, text="SET PID", command=self.cmd_set_pid).grid(row=2, column=2, sticky="w")
        # Show raw toggle
        self.raw_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(frm, text="Show raw", variable=self.raw_var).grid(row=2, column=3, sticky="w")
        # Manual send
        self.manual = ttk.Entry(frm, width=50)
        self.manual.grid(row=3, column=0, columnspan=4, sticky="we")
        ttk.Button(frm, text="Send", command=self.manual_send).grid(row=3, column=4, sticky="w")
        # Output area (filtered)
        self.out_txt = scrolledtext.ScrolledText(frm, width=80, height=18, state='disabled')
        self.out_txt.grid(row=4, column=0, columnspan=6, pady=6)
        # connection state
        self.ser = None
        self.reader = None
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.root.after(100, self._poll_queue)
        # filtering regexes (tweak if your ESP prints differently)
        self.re_telemetry = re.compile(r'^(PITCH:|ROLL:|YAW:)', re.IGNORECASE)
        self.re_pid_line = re.compile(r'^(KP:|KI:|KD:)', re.IGNORECASE)
        self.max_lines = 500
        # telemetry throttle state
        self._last_telemetry_ts = 0.0

    def _list_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _refresh_ports(self):
        self.port_cb['values'] = self._list_ports()

    def toggle_connect(self):
        if self.ser:
            self._disconnect()
        else:
            port = self.port_cb.get().strip()
            if not port:
                messagebox.showwarning("Port", "Select a serial port first.")
                return
            try:
                self.ser = serial.Serial(port, 115200, timeout=READ_TIMEOUT)
            except Exception as e:
                messagebox.showerror("Open serial", str(e))
                self.ser = None
                return
            self.stop_event.clear()
            self.reader = SerialReader(self.ser, self.q, self.stop_event)
            self.reader.start()
            self.b_connect.config(text="Disconnect")
            self._log(f"Connected to {port}")

    def _disconnect(self):
        self.stop_event.set()
        time.sleep(0.05)
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except:
            pass
        self.ser = None
        self.b_connect.config(text="Connect")
        self._log("Disconnected")

    def _poll_queue(self):
        flushed = False
        while True:
            try:
                line = self.q.get_nowait()
            except queue.Empty:
                break
            flushed = True
            # if raw is enabled, show everything
            if self.raw_var.get():
                self._log(line)
                continue
            # filter: show telemetry or PID get responses only, but throttle high-rate telemetry
            if self.re_telemetry.match(line):
                now = time.time()
                if (now - self._last_telemetry_ts) >= TELEMETRY_THROTTLE:
                    self._last_telemetry_ts = now
                    self._log(line)
                # else drop this telemetry line (throttled)
                continue
            if self.re_pid_line.match(line):
                self._log(line)
                continue
            # else ignore noisy lines
        if flushed:
            self._trim()
        self.root.after(100, self._poll_queue)

    def _log(self, text):
        # Insert text but only auto-scroll if the view is already at the bottom.
        self.out_txt.configure(state='normal')
        self.out_txt.insert('end', text + "\n")
        # check if scrollbar currently at bottom (last fraction ~= 1.0)
        try:
            first, last = self.out_txt.yview()
        except Exception:
            first, last = 0.0, 1.0
        if last >= 0.999:
            self.out_txt.see('end')
        # if user has scrolled up (last < 1.0) we do not force scroll
        self.out_txt.configure(state='disabled')

    def _trim(self):
        lines = int(self.out_txt.index('end-1c').split('.')[0])
        if lines > self.max_lines:
            # delete top lines
            self.out_txt.configure(state='normal')
            self.out_txt.delete('1.0', f'{lines - self.max_lines}.0')
            self.out_txt.configure(state='disabled')

    def cmd_get_pid(self):
        if not self._ensure_ser(): return
        # protocol assumed: send "GET PID\n"
        self.ser.write(b"GET PID\n")

    def cmd_set_pid(self):
        if not self._ensure_ser(): return
        try:
            kp = float(self.e_kp.get())
            ki = float(self.e_ki.get())
            kd = float(self.e_kd.get())
        except:
            messagebox.showerror("Parse", "KP/KI/KD must be numbers")
            return
        # protocol assumed: send "SET PID <kp> <ki> <kd>\n"
        cmd = f"SET PID {kp:.6f} {ki:.6f} {kd:.6f}\n"
        self.ser.write(cmd.encode('utf-8'))
        self._log(f"> {cmd.strip()}")

    def manual_send(self):
        if not self._ensure_ser(): return
        txt = self.manual.get().strip()
        if not txt: return
        self.ser.write((txt + "\n").encode('utf-8'))
        self._log("> " + txt)

    def _ensure_ser(self):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Serial", "Not connected.")
            return False
        return True

    def on_close(self):
        if self.ser:
            self._disconnect()
        self.root.quit()

if __name__ == "__main__":
    root = tk.Tk()
    gui = PIDGui(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_close)
    root.mainloop()

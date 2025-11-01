#!/usr/bin/env python3
"""
sbr_tuner.py — Minimal PID tuner + telemetry viewer for the self-balancing robot.
Dependencies: pyserial
  pip install pyserial
"""

import threading, queue, time, sys
import math
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import socket
import re

# ---------------------- Serial Background Reader ----------------------
class SerialReader(threading.Thread):
    def __init__(self, port, baud, line_q, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.line_q = line_q
        self.stop_event = stop_event
        self.ser = None

    def run(self):
        try:
            print(f"DEBUG: attempting open {self.port}@{self.baud}")
            tried = []
            def try_serial_for_url(u):
                try:
                    s = serial.serial_for_url(u, baudrate=self.baud, timeout=0.1)
                    return s
                except Exception as e:
                    tried.append((u, repr(e)))
                    return None

            serobj = None
            if isinstance(self.port, str) and '://' in self.port:
                serobj = try_serial_for_url(self.port)
                if serobj is None and self.port.startswith('rfc2217://'):
                    alt = 'socket://' + self.port.split('://',1)[1]
                    serobj = try_serial_for_url(alt)
                if serobj is None and 'localhost' in self.port:
                    alt2 = self.port.replace('localhost', '127.0.0.1')
                    serobj = try_serial_for_url(alt2)
                if serobj is None:
                    m = re.search(r'://([^:/]+):?([0-9]+)$', self.port)
                    if m:
                        host, portnum = m.group(1), m.group(2)
                        alt3 = f'socket://{host}:{portnum}'
                        serobj = try_serial_for_url(alt3)
                if serobj is None:
                    m = re.search(r'://([^:/]+):?([0-9]+)$', self.port)
                    if m:
                        host, portnum = m.group(1), int(m.group(2))
                        s = socket.create_connection((host, portnum), timeout=2.0)
                        s.settimeout(0.1)
                        class SockWrap:
                            def __init__(self, sock): self.sock = sock
                            def read(self, n=1):
                                try:
                                    return self.sock.recv(n)
                                except socket.timeout:
                                    return b''
                            def write(self, b):
                                try:
                                    return self.sock.sendall(b)
                                except:
                                    return None
                            def close(self):
                                try: self.sock.close()
                                except: pass
                            @property
                            def is_open(self): return True
                        serobj = SockWrap(s)
            else:
                try:
                    serobj = serial.Serial(self.port, self.baud, timeout=0.1)
                except Exception as e:
                    tried = [(self.port, repr(e))]

            if serobj is None:
                emsg = '; '.join(f"{u} -> {err}" for u, err in tried) if tried else 'no attempts'
                raise OSError(f'Could not open any serial variant. Attempts: {emsg}')

            self.ser = serobj
        except Exception as e:
            self.line_q.put(("__error__", f"Failed open {self.port}@{self.baud}: {e}"))
            print(f"DEBUG: Failed open {self.port}@{self.baud}: {e}")
            return

        self.line_q.put(("__info__", f"Opened {self.port} @ {self.baud}"))
        print(f"DEBUG: Opened {self.port} @ {self.baud}")
        buf = b''
        while not self.stop_event.is_set():
            try:
                data = self.ser.read(256)
                if data:
                    buf += data
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        try:
                            text = line.decode('utf-8', errors='ignore').strip()
                        except:
                            text = repr(line)
                        self.line_q.put(("line", text))
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.line_q.put(("__error__", f"Serial read error: {e}"))
                break
        try:
            self.ser.close()
        except:
            pass
        self.line_q.put(("__info__", "Serial thread exiting"))

    def write_line(self, s):
        if self.ser and self.ser.is_open:
            self.ser.write((s + "\n").encode('utf-8'))

# ---------------------- Simple Telemetry Parser ----------------------
def parse_telemetry(line):
    out = {}
    try:
        parts = line.replace(',', ' ').split()
        for p in parts:
            if ':' in p:
                k, v = p.split(':', 1)
            elif '=' in p:
                k, v = p.split('=', 1)
            else:
                continue
            k = k.strip().upper()
            v = v.strip()
            try:
                fv = float(v)
            except:
                continue
            if ("PITCH" in k) or (k == "P"):
                out['pitch'] = fv
            elif ("ROLL" in k) or (k == "R"):
                out['roll'] = fv
            elif ("YAW" in k) or (k == "Y"):
                out['yaw'] = fv
            elif k in ("KP","KI","KD") or k=="PID":
                out[k.lower()] = fv
    except Exception:
        pass
    return out

# ---------------------- Tk GUI ----------------------
class TunerApp:
    def __init__(self, root):
        self.root = root
        root.title("SBR PID Tuner - Minimal Dashboard")
        self.line_q = queue.Queue()
        self.stop_event = threading.Event()
        self.reader = None
        self.last_pid = None

        # Top frame: connection
        f_conn = ttk.Frame(root, padding=6)
        f_conn.grid(row=0, column=0, sticky="ew")
        f_conn.columnconfigure(4, weight=1)

        self.port_var = tk.StringVar(value='rfc2217://127.0.0.1:4000')
        self.baud_var = tk.StringVar(value="115200")
        self.kp_var = tk.StringVar(value="0.0")
        self.ki_var = tk.StringVar(value="0.0")
        self.kd_var = tk.StringVar(value="0.0")

        ttk.Label(f_conn, text="Port:").grid(row=0, column=0)
        self.port_cb = ttk.Combobox(f_conn, values=self.list_ports(), textvariable=self.port_var, width=18)
        self.port_cb.grid(row=0, column=1)
        ttk.Label(f_conn, text="Baud:").grid(row=0, column=2)
        ttk.Entry(f_conn, textvariable=self.baud_var, width=8).grid(row=0, column=3)
        self.btn_conn = ttk.Button(f_conn, text="Connect", command=self.toggle_connect)
        self.btn_conn.grid(row=0, column=4, sticky="e")

        # Middle frame: PID controls
        f_pid = ttk.Frame(root, padding=6)
        f_pid.grid(row=1, column=0, sticky="ew")
        ttk.Label(f_pid, text="Kp").grid(row=0, column=0)
        ttk.Entry(f_pid, textvariable=self.kp_var, width=8).grid(row=0, column=1)
        ttk.Label(f_pid, text="Ki").grid(row=0, column=2)
        ttk.Entry(f_pid, textvariable=self.ki_var, width=8).grid(row=0, column=3)
        ttk.Label(f_pid, text="Kd").grid(row=0, column=4)
        ttk.Entry(f_pid, textvariable=self.kd_var, width=8).grid(row=0, column=5)
        ttk.Button(f_pid, text="Set PID", command=self.send_set_pid).grid(row=0, column=6)
        ttk.Button(f_pid, text="Get PID", command=self.send_get_pid).grid(row=0, column=7)

        # Telemetry frame: numbers + 2D tilt canvas
        f_tel = ttk.Frame(root, padding=6)
        f_tel.grid(row=2, column=0, sticky="nsew")
        root.rowconfigure(2, weight=1)
        f_tel.columnconfigure(1, weight=1)

        # Numeric readout
        f_vals = ttk.Frame(f_tel)
        f_vals.grid(row=0, column=0, sticky="n")
        ttk.Label(f_vals, text="Pitch (deg):").grid(row=0, column=0, sticky="w")
        self.pitch_lbl = ttk.Label(f_vals, text="---")
        self.pitch_lbl.grid(row=0, column=1, sticky="w")
        ttk.Label(f_vals, text="Roll (deg):").grid(row=1, column=0, sticky="w")
        self.roll_lbl = ttk.Label(f_vals, text="---")
        self.roll_lbl.grid(row=1, column=1, sticky="w")
        ttk.Label(f_vals, text="Yaw (deg):").grid(row=2, column=0, sticky="w")
        self.yaw_lbl = ttk.Label(f_vals, text="---")
        self.yaw_lbl.grid(row=2, column=1, sticky="w")

        # Canvas indicator (simple rectangle representing robot body; roll rotates it; pitch shifts it)
        self.canvas = tk.Canvas(f_tel, width=240, height=160, bg="white")
        self.canvas.grid(row=0, column=1, sticky="nsew", padx=8, pady=8)
        self.center = (120, 80)
        self.rect_w = 120
        self.rect_h = 40
        # original rectangle points centered at origin
        self.rect_pts = [(-self.rect_w/2, -self.rect_h/2),
                         (self.rect_w/2, -self.rect_h/2),
                         (self.rect_w/2, self.rect_h/2),
                         (-self.rect_w/2, self.rect_h/2)]
        self.rect_id = None

        # PID log (separate from RX log)
        self.pid_log = scrolledtext.ScrolledText(root, height=4, state='disabled')
        self.pid_log.grid(row=3, column=0, sticky="ew", padx=6, pady=(6,0))

        # Log window
        self.log = scrolledtext.ScrolledText(root, height=8, state='disabled')
        self.log.grid(row=4, column=0, sticky="ew", padx=6, pady=6)

        # Start UI tick
        self.last_pitch = 0.0
        self.last_roll = 0.0
        self.root.after(50, self.ui_tick)

    def list_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return ports if ports else ["COM3","/dev/ttyUSB0"]

    def toggle_connect(self):
        if self.reader and self.reader.is_alive():
            # disconnect
            self.stop_event.set()
            self.btn_conn.config(text="Connect")
            self.reader = None
        else:
            port = self.port_var.get()
            try:
                baud = int(self.baud_var.get())
            except:
                messagebox.showerror("Baud error", "Baud must be integer")
                return
            self.stop_event.clear()
            self.line_q = queue.Queue()
            self.reader = SerialReader(port, baud, self.line_q, self.stop_event)
            self.reader.start()
            self.btn_conn.config(text="Disconnect")
            # start draining messages immediately
            self.root.after(50, self.drain_serial)

    def drain_serial(self):
        if not self.reader:
            return
        while not self.line_q.empty():
            typ, payload = self.line_q.get()
            if typ == "line":
                self.handle_line(payload)
            elif typ == "__error__":
                self.log_msg("ERROR: " + payload)
            else:
                self.log_msg(payload)
        if self.reader and self.reader.is_alive():
            self.root.after(30, self.drain_serial)
        else:
            self.log_msg("Disconnected")
            self.btn_conn.config(text="Connect")

    def handle_line(self, line):
        # parse telemetry and PID values
        parsed = parse_telemetry(line)
        pid_present = False
        pid_vals = {}
        if parsed:
            # telemetry updates
            if 'pitch' in parsed or 'roll' in parsed or 'yaw' in parsed:
                pitch = parsed.get('pitch', self.last_pitch)
                roll = parsed.get('roll', self.last_roll)
                yaw = parsed.get('yaw', None)
                self.update_telemetry(pitch, roll, yaw)
            # capture PID keys if present (kp, ki, kd)
            for k in ('kp','ki','kd'):
                if k in parsed:
                    pid_present = True
                    pid_vals[k] = parsed[k]
        # route PID messages to pid_log (separate)
        if pid_present:
            kp = pid_vals.get('kp', None)
            ki = pid_vals.get('ki', None)
            kd = pid_vals.get('kd', None)
            msg = f"PID: Kp={kp} Ki={ki} Kd={kd}"
            try:
                self.pid_log.config(state='normal')
                if self.last_pid != (kp,ki,kd):
                    self.pid_log.insert('end', f"PID CHANGED -> {msg}\n")
                    self.last_pid = (kp,ki,kd)
                else:
                    self.pid_log.insert('end', f"PID ECHO -> {msg}\n")
                self.pid_log.yview('end')
                self.pid_log.config(state='disabled')
            except Exception:
                pass
            # also add a short entry in main log for traceability
            self.log_msg(msg, prefix='PID: ')
        # always log raw RX for debugging, unchanged
        self.log_msg(line, prefix="RX: ")

    def update_telemetry(self, pitch, roll, yaw):
        self.last_pitch = pitch
        self.last_roll = roll
        self.pitch_lbl.config(text=f"{pitch:.2f}")
        self.roll_lbl.config(text=f"{roll:.2f}")
        self.yaw_lbl.config(text="-" if yaw is None else f"{yaw:.2f}")
        self.draw_indicator(pitch, roll)

    def draw_indicator(self, pitch, roll):
        cx, cy = self.center
        theta = math.radians(roll)
        cos_t = math.cos(theta); sin_t = math.sin(theta)
        pts = []
        for (x,y) in self.rect_pts:
            rx = x * cos_t - y * sin_t
            ry = x * sin_t + y * cos_t
            ry += (pitch * 0.6)
            pts.append((cx + rx, cy + ry))
        flat = [coord for p in pts for coord in p]
        if self.rect_id is None:
            self.rect_id = self.canvas.create_polygon(flat, fill="", outline="black", width=2)
        else:
            self.canvas.coords(self.rect_id, *flat)

    def log_msg(self, s, prefix=""):
        self.log.config(state='normal')
        self.log.insert('end', f"{prefix}{s}\n")
        self.log.yview('end')
        self.log.config(state='disabled')

    def send_set_pid(self):
        try:
            kp = float(self.kp_var.get()); ki = float(self.ki_var.get()); kd = float(self.kd_var.get())
        except:
            messagebox.showerror("Values", "KP/KI/KD must be numbers")
            return
        line = f"SET PID {kp} {ki} {kd}"
        if self.reader and getattr(self.reader, "ser", None) and getattr(self.reader.ser, "is_open", False):
            self.reader.write_line(line)
            self.log_msg("TX: " + line)
        else:
            messagebox.showerror("Serial", "Not connected")

    def send_get_pid(self):
        line = "GET PID"
        if self.reader and getattr(self.reader, "ser", None) and getattr(self.reader.ser, "is_open", False):
            self.reader.write_line(line)
            self.log_msg("TX: " + line)
        else:
            messagebox.showerror("Serial", "Not connected")

    def ui_tick(self):
        if not (self.reader and self.reader.is_alive()):
            if int(time.time()) % 5 == 0:
                try:
                    self.port_cb['values'] = self.list_ports()
                except Exception:
                    pass
        self.root.after(200, self.ui_tick)

def main():
    root = tk.Tk()
    app = TunerApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (setattr(app, 'stop_event', app.stop_event) or app.stop_event.set() or root.destroy()))
    root.mainloop()

if __name__ == "__main__":
    main()

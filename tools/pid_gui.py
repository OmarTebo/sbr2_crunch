#!/usr/bin/env python3
"""
SBR PID Control — compact UI with 2D horizon + strip graph
- Compact KP/KI/KD layout
- 2D horizon (pitch/roll) + tiny rolling strip graph for history
- Tabbed output: Filtered / Raw
- Show raw toggles whether raw lines are recorded into Raw tab
- GET PID updates KP/KI/KD entries from serial responses
- Auto-scroll: follows only when view already at bottom; if you scroll up it won't yank you.
Requires: pyserial
Install: pip install pyserial
Run: python pid_gui.py
"""
import threading, queue, time, re, math, collections
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import serial
import serial.tools.list_ports

READ_TIMEOUT = 0.1
TELEMETRY_THROTTLE = 0.05  # seconds between telemetry updates
GRAPH_HISTORY = 400        # points in strip graph

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
        frm = ttk.Frame(root, padding=6)
        frm.grid(sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(3, weight=1)

        # --- top: port and connect ---
        top = ttk.Frame(frm)
        top.grid(row=0, column=0, sticky="we")
        ttk.Label(top, text="Serial:").pack(side="left")
        self.port_cb = ttk.Combobox(top, width=20, values=self._list_ports())
        self.port_cb.pack(side="left", padx=(4,6))
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side="left")
        self.b_connect = ttk.Button(top, text="Connect", command=self.toggle_connect)
        self.b_connect.pack(side="right")

        # --- compact PID row ---
        pidrow = ttk.Frame(frm)
        pidrow.grid(row=1, column=0, sticky="we", pady=(6,2))
        # tighten spacing
        lbl_kp = ttk.Label(pidrow, text="KP"); lbl_kp.grid(row=0, column=0, padx=(0,2))
        self.e_kp = ttk.Entry(pidrow, width=9); self.e_kp.grid(row=0, column=1)
        lbl_ki = ttk.Label(pidrow, text="KI"); lbl_ki.grid(row=0, column=2, padx=(8,2))
        self.e_ki = ttk.Entry(pidrow, width=9); self.e_ki.grid(row=0, column=3)
        lbl_kd = ttk.Label(pidrow, text="KD"); lbl_kd.grid(row=0, column=4, padx=(8,2))
        self.e_kd = ttk.Entry(pidrow, width=9); self.e_kd.grid(row=0, column=5)
        ttk.Button(pidrow, text="GET PID", width=8, command=self.cmd_get_pid).grid(row=0, column=6, padx=(12,4))
        ttk.Button(pidrow, text="SET PID", width=8, command=self.cmd_set_pid).grid(row=0, column=7)
        self.raw_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(pidrow, text="Show raw", variable=self.raw_var).grid(row=0, column=8, padx=(12,0))

        # --- middle: horizon canvas + strip graph ---
        canvas_frame = ttk.Frame(frm)
        canvas_frame.grid(row=2, column=0, sticky="we", pady=(6,4))
        canvas_frame.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(canvas_frame, width=560, height=180, bg="#f6f6f6", bd=1, relief="solid")
        self.canvas.grid(sticky="we")
        # strip graph canvas
        self.graph = tk.Canvas(canvas_frame, width=560, height=70, bg="#111", bd=1, relief="solid")
        self.graph.grid(sticky="we", pady=(6,0))

        # history buffers for strip graph
        self.pitch_hist = collections.deque([0.0]*GRAPH_HISTORY, maxlen=GRAPH_HISTORY)
        self.roll_hist  = collections.deque([0.0]*GRAPH_HISTORY, maxlen=GRAPH_HISTORY)
        self._last_telemetry_ts = 0.0
        self.pitch = 0.0; self.roll = 0.0; self.yaw = 0.0

        # static draw
        self._draw_horizon_static()
        self._draw_graph_static()

        # --- bottom: tabbed outputs ---
        bottom_frame = ttk.Frame(frm)
        bottom_frame.grid(row=3, column=0, sticky="nsew")
        bottom_frame.columnconfigure(0, weight=1)
        bottom_frame.rowconfigure(0, weight=1)
        self.notebook = ttk.Notebook(bottom_frame)
        self.notebook.grid(sticky="nsew")
        self.filtered_tab = ttk.Frame(self.notebook)
        self.raw_tab = ttk.Frame(self.notebook)
        self.out_filtered = scrolledtext.ScrolledText(self.filtered_tab, width=120, height=18, state='disabled')
        self.out_filtered.pack(fill="both", expand=True)
        self.out_raw = scrolledtext.ScrolledText(self.raw_tab, width=120, height=18, state='disabled')
        self.out_raw.pack(fill="both", expand=True)
        self.notebook.add(self.filtered_tab, text="Filtered")
        self.notebook.add(self.raw_tab, text="Raw")

        # manual send
        manual_row = ttk.Frame(frm)
        manual_row.grid(row=4, column=0, sticky="we", pady=(6,0))
        self.manual = ttk.Entry(manual_row, width=80)
        self.manual.pack(side="left", fill="x", expand=True)
        ttk.Button(manual_row, text="Send", command=self.manual_send).pack(side="left", padx=(6,0))

        # serial state
        self.ser = None; self.reader = None; self.q = queue.Queue(); self.stop_event = threading.Event()
        self.root.after(40, self._poll_queue)

        # regex parsers
        self.re_kv = re.compile(r'\b(KP|KI|KD)[:=]\s*([-\d.]+)', re.IGNORECASE)
        self.re_pid_line = re.compile(r'KP[:=]\s*([-\d.]+)\s+KI[:=]\s*([-\d.]+)\s+KD[:=]\s*([-\d.]+)', re.IGNORECASE)
        self.re_telemetry = re.compile(r'PITCH[:=]\s*([-\d.]+)\s+ROLL[:=]\s*([-\d.]+)\s+YAW[:=]\s*([-\d.]+)', re.IGNORECASE)
        self.re_telemetry_alt = re.compile(r'^(PITCH:|ROLL:|YAW:)', re.IGNORECASE)
        # track whether user scrolled to bottom for each widget
        self._follow_filtered = True
        self._follow_raw = True
        self._attach_scroll_handlers()

    def _attach_scroll_handlers(self):
        # add bindings to toggle follow flags when user scrolls manually
        def bind_widget(widget, follow_attr):
            vsb = widget.vbar = widget.vbar if hasattr(widget, 'vbar') else None
            # can't rely on internal vbar; instead bind mousewheel & key events to detect user scroll
            def on_scroll_user(event):
                # check current view; if near bottom set follow True else False
                try:
                    f, l = widget.yview()
                    if l >= 0.999:
                        setattr(self, follow_attr, True)
                    else:
                        setattr(self, follow_attr, False)
                except:
                    setattr(self, follow_attr, True)
            widget.bind("<Button-1>", on_scroll_user)      # click in box
            widget.bind("<MouseWheel>", on_scroll_user)    # wheel scroll
            widget.bind("<Key>", on_scroll_user)           # arrow keys
            widget.bind("<Button-4>", on_scroll_user)      # *nix wheel
            widget.bind("<Button-5>", on_scroll_user)
        bind_widget(self.out_filtered, "_follow_filtered")
        bind_widget(self.out_raw, "_follow_raw")

    # --- Canvas drawing ---
    def _draw_horizon_static(self):
        c = self.canvas
        c.delete("all")
        w = int(c.winfo_reqwidth()); h = int(c.winfo_reqheight())
        self.h_center = (w//2, h//2)
        # rectangle baseline (body)
        c.create_rectangle(10, 10, w-10, h-10, outline="#aaa", width=2, tags="frame")
        c.create_line(0, h//2, w, h//2, fill="#fff", width=2, tags="horizon_line")
        c.create_text(12, h-24, anchor="w", text="Pitch: 0.00°  Roll: 0.00°", tags="txt_pr", fill="#111")

    def _draw_graph_static(self):
        g = self.graph
        g.delete("all")
        w = int(g.winfo_reqwidth()); h = int(g.winfo_reqheight())
        g.create_text(6, 6, anchor="nw", text="roll (red)   pitch (green)", fill="#ddd", tags="lbl", font=("TkDefaultFont", 8))

    def _update_horizon(self):
        try:
            pitch = float(self.pitch); roll = float(self.roll)
        except:
            return
        c = self.canvas
        w = int(c.winfo_width()); h = int(c.winfo_height())
        cx, cy = w//2, h//2
        # rotate a rectangle by roll, and shift vertical by pitch
        wbox, hbox = 220, 80
        angle = math.radians(-roll)
        corners = [(-wbox/2, -hbox/2), (wbox/2, -hbox/2), (wbox/2, hbox/2), (-wbox/2, hbox/2)]
        pts = []
        for x, y in corners:
            xr = x*math.cos(angle) - y*math.sin(angle)
            yr = x*math.sin(angle) + y*math.cos(angle)
            pts.extend([cx + xr, cy + yr + (pitch*0.6)])  # pitch shifts vertically
        if c.find_withtag("body"):
            c.coords("body", *pts)
        else:
            c.create_polygon(*pts, fill="#61aaff", outline="#003", tags="body")
        c.itemconfigure("txt_pr", text=f"Pitch: {pitch:.2f}°  Roll: {roll:.2f}°")

    def _update_graph(self):
        # draw small strip chart of history
        g = self.graph
        g.delete("line_roll"); g.delete("line_pitch")
        w = int(g.winfo_width()); h = int(g.winfo_height())
        ph = list(self.pitch_hist); rh = list(self.roll_hist)
        # normalize to +/-45 deg for display
        def map_y(val):
            r = max(-60.0, min(60.0, val))
            return int((h-10)/2 - (r / 60.0) * ((h-10)/2)) + 8
        step = max(1, w // GRAPH_HISTORY)
        xs = list(range(2, 2 + step*len(ph), step))[:len(ph)]
        if len(xs) < 2: return
        coords_r = []
        coords_p = []
        for xi, rv, pv in zip(xs[-len(rh):], rh[-len(xs):], ph[-len(xs):]):
            coords_r.extend([xi, map_y(rv)])
            coords_p.extend([xi, map_y(pv)])
        if coords_r:
            g.create_line(*coords_r, fill="#ff4444", width=1, tags="line_roll", smooth=True)
        if coords_p:
            g.create_line(*coords_p, fill="#33ff77", width=1, tags="line_pitch", smooth=True)

    # --- serial / UI handling ---
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
            self._clear_outputs()
            self._log_filtered(f"Connected to {port}")

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
        self._log_filtered("Disconnected")

    def _poll_queue(self):
        flushed = False
        while True:
            try:
                line = self.q.get_nowait()
            except queue.Empty:
                break
            flushed = True

            # parse PID triple lines first
            m_pid = self.re_pid_line.search(line)
            if m_pid:
                try:
                    kp = float(m_pid.group(1)); ki = float(m_pid.group(2)); kd = float(m_pid.group(3))
                    self._set_entry(self.e_kp, f"{kp:.6f}")
                    self._set_entry(self.e_ki, f"{ki:.6f}")
                    self._set_entry(self.e_kd, f"{kd:.6f}")
                except:
                    pass
                self._log_filtered("PID: " + line)
                if self.raw_var.get(): self._log_raw(line)
                continue

            # parse any KP/KI/KD pairs anywhere (robust)
            kvs = self.re_kv.findall(line)
            if kvs:
                for k, v in kvs:
                    try:
                        val = float(v)
                    except:
                        continue
                    kk = k.upper()
                    if kk == "KP": self._set_entry(self.e_kp, f"{val:.6f}")
                    elif kk == "KI": self._set_entry(self.e_ki, f"{val:.6f}")
                    elif kk == "KD": self._set_entry(self.e_kd, f"{val:.6f}")
                self._log_filtered(line)
                if self.raw_var.get(): self._log_raw(line)
                continue

            # telemetry single-line
            mt = self.re_telemetry.search(line)
            if mt:
                now = time.time()
                if (now - self._last_telemetry_ts) >= TELEMETRY_THROTTLE:
                    self._last_telemetry_ts = now
                    try:
                        self.pitch = float(mt.group(1)); self.roll = float(mt.group(2)); self.yaw = float(mt.group(3))
                    except:
                        pass
                    # append to history
                    self.pitch_hist.append(self.pitch); self.roll_hist.append(self.roll)
                    self._update_horizon(); self._update_graph()
                    self._log_filtered(line)
                if self.raw_var.get(): self._log_raw(line)
                continue

            # fallback: loose telemetry prefix
            if self.re_telemetry_alt.match(line):
                now = time.time()
                if (now - self._last_telemetry_ts) >= TELEMETRY_THROTTLE:
                    self._last_telemetry_ts = now
                    self._log_filtered(line)
                if self.raw_var.get(): self._log_raw(line)
                continue

            # other: log to raw only if requested
            if self.raw_var.get():
                self._log_raw(line)
            # else ignore

        if flushed:
            self._trim_all()
        self.root.after(40, self._poll_queue)

    # text helpers with follow-on-bottom behavior
    def _append_text_widget(self, widget, text, follow_attr_name):
        widget.configure(state='normal')
        widget.insert('end', text + "\n")
        # check current view; only auto-scroll if follow flag True (user hasn't scrolled up)
        try:
            f, l = widget.yview()
        except:
            f, l = 0.0, 1.0
        if getattr(self, follow_attr_name):
            widget.see("end")
        widget.configure(state='disabled')

    def _log_filtered(self, text):
        self._append_text_widget(self.out_filtered, text, "_follow_filtered")

    def _log_raw(self, text):
        self._append_text_widget(self.out_raw, text, "_follow_raw")

    def _clear_outputs(self):
        for w in (self.out_filtered, self.out_raw):
            w.configure(state='normal')
            w.delete('1.0', 'end')
            w.configure(state='disabled')

    def _trim_all(self):
        for w in (self.out_filtered, self.out_raw):
            lines = int(w.index('end-1c').split('.')[0])
            max_lines = 3000
            if lines > max_lines:
                w.configure(state='normal')
                w.delete('1.0', f'{lines - max_lines}.0')
                w.configure(state='disabled')

    def _set_entry(self, entry, val):
        try:
            entry.delete(0, 'end'); entry.insert(0, str(val))
        except:
            pass

    # GET/SET
    def cmd_get_pid(self):
        if not self._ensure_ser(): return
        self._set_entry(self.e_kp, ""); self._set_entry(self.e_ki, ""); self._set_entry(self.e_kd, "")
        self.ser.write(b"GET PID\n")

    def cmd_set_pid(self):
        if not self._ensure_ser(): return
        try:
            kp = float(self.e_kp.get()); ki = float(self.e_ki.get()); kd = float(self.e_kd.get())
        except:
            messagebox.showerror("Parse", "KP/KI/KD must be numbers"); return
        cmd = f"SET PID {kp:.6f} {ki:.6f} {kd:.6f}\n"
        self.ser.write(cmd.encode('utf-8'))
        self._log_filtered("> " + cmd.strip())
        if self.raw_var.get(): self._log_raw("> " + cmd.strip())

    def manual_send(self):
        if not self._ensure_ser(): return
        txt = self.manual.get().strip()
        if not txt: return
        self.ser.write((txt + "\n").encode('utf-8'))
        self._log_filtered("> " + txt)
        if self.raw_var.get(): self._log_raw("> " + txt)

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

#!/usr/bin/env python3
"""
battery_viewer.py  —  Read-only battery topic viewer

Subscribes to all battery/* topics (relative, so a namespace can be applied) and displays them. Publishes nothing.

Usage:
    ros2 run nissan_bridge_gui battery_viewer
    ros2 run nissan_bridge_gui battery_viewer --ros-args -r __ns:=/nissan9

vide coded, don't take as a reference for how to write a GUI    
"""

import threading
import time
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.node import Node
from nissan_bridge_msgs.msg import Float64Stamped
from sensor_msgs.msg import Temperature

# ── Palette ───────────────────────────────────────────────────────────────────
BG     = "#0d0f14"
PANEL  = "#13161e"
BORDER = "#1e2330"
ACCENT = "#00d4ff"
ORANGE = "#ff6b35"
GOOD   = "#00e676"
WARN   = "#ffea00"
BAD    = "#ff1744"
TEXT   = "#e8eaf0"
MUTED  = "#4a5068"

# (topic, key, unit, color, formatter, message type)
TOPICS = [
    ("battery/soc",              "soc",         "%",  GOOD,   lambda v: f"{v*100:.2f}", Float64Stamped),
    ("battery/voltage",          "voltage",     "V",  ACCENT, lambda v: f"{v:.3f}",     Float64Stamped),
    ("battery/current",          "current",     "A",  ACCENT, lambda v: f"{v:.2f}",     Float64Stamped),
    ("battery/temperature",      "temperature", "°C", WARN,   lambda v: f"{v:.2f}",     Temperature),
    ("battery/soh",              "soh",         "%",  ORANGE, lambda v: f"{v*100:.2f}", Float64Stamped),
    ("battery/energy_consumed",  "energy",      "Wh", TEXT,   lambda v: f"{v:.2f}",     Float64Stamped),
    ("battery/max_load_power",   "max_load",    "W",  MUTED,  lambda v: f"{v:.0f}",     Float64Stamped),
    ("battery/max_charge_power", "max_charge",  "W",  MUTED,  lambda v: f"{v:.0f}",     Float64Stamped),
]

# (label, key, unit, color, lo, hi, scale) — scale converts raw value to shown value
GAUGES = [
    ("SOC",     "soc",         "%",  GOOD,   0,   100, 100.0),
    ("VOLTAGE", "voltage",     "V",  ACCENT, 300, 430, 1.0),
    ("TEMP",    "temperature", "°C", WARN,   15,  55,  1.0),
    ("SOH",     "soh",         "%",  ORANGE, 0,   100, 100.0),
]


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class BatterySubscriber(Node):
    def __init__(self):
        super().__init__("battery_viewer_gui")
        self.data = {}
        self.last_update = {}
        for topic, key, *_, msg_type in TOPICS:
            field = "temperature" if msg_type is Temperature else "data"
            self.create_subscription(
                msg_type, topic,
                lambda msg, k=key, f=field: self._recv(k, getattr(msg, f)), 10)

    def _recv(self, key, value):
        self.data[key] = value
        self.last_update[key] = time.time()


# ── GUI ───────────────────────────────────────────────────────────────────────

class BatteryViewer:
    HIST_LEN = 300  # ticks at 100 ms ≈ 30 s

    def __init__(self, root: tk.Tk, node: BatterySubscriber):
        self.root = root
        self.node = node
        self.gauges = {}
        self.rows = {}
        self.history = {g[1]: [] for g in GAUGES}

        root.title("Battery Viewer")
        root.configure(bg=BG)
        root.geometry("760x650")

        self.fT = tkfont.Font(family="Courier", size=10, weight="bold")
        self.fV = tkfont.Font(family="Courier", size=24, weight="bold")
        self.fL = tkfont.Font(family="Courier", size=9)
        self.fS = tkfont.Font(family="Courier", size=8)
        self.fM = tkfont.Font(family="Courier", size=10, weight="bold")

        self._build()
        self._tick()

    def _section(self, text):
        tk.Frame(self.root, bg=BORDER, height=1).pack(fill="x", padx=14, pady=(10, 4))
        tk.Label(self.root, text=text, font=self.fL, fg=MUTED, bg=BG).pack(anchor="w", padx=14)

    def _build(self):
        # Header
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=14, pady=(12, 0))
        tk.Label(top, text="BATTERY VIEWER", font=self.fT, fg=ACCENT, bg=BG).pack(side="left")
        self.lbl_status = tk.Label(top, text="● WAITING", font=self.fL, fg=MUTED, bg=BG)
        self.lbl_status.pack(side="right")

        # Gauges
        self._section("OVERVIEW")
        gf = tk.Frame(self.root, bg=BG)
        gf.pack(fill="x", padx=14, pady=(4, 0))
        for col, (label, key, unit, color, lo, hi, scale) in enumerate(GAUGES):
            f = tk.Frame(gf, bg=PANEL, padx=8, pady=6,
                         highlightthickness=1, highlightbackground=BORDER)
            f.grid(row=0, column=col, padx=3, sticky="nsew")
            gf.columnconfigure(col, weight=1)
            tk.Label(f, text=label, font=self.fL, fg=MUTED, bg=PANEL).pack()
            var = tk.StringVar(value="—")
            val_lbl = tk.Label(f, textvariable=var, font=self.fV, fg=color, bg=PANEL)
            val_lbl.pack()
            tk.Label(f, text=unit, font=self.fL, fg=MUTED, bg=PANEL).pack()
            c = tk.Canvas(f, height=6, bg=PANEL, highlightthickness=0)
            c.pack(fill="x", pady=(4, 0))
            self.gauges[key] = {"var": var, "lbl": val_lbl, "canvas": c, "color": color,
                                "lo": lo, "hi": hi, "scale": scale}

        # Topic table
        self._section("SUBSCRIBED TOPICS")
        tf = tk.Frame(self.root, bg=BG)
        tf.pack(fill="x", padx=14, pady=(4, 0))
        for topic, key, unit, color, fmt, _ in TOPICS:
            row = tk.Frame(tf, bg=PANEL, highlightthickness=1, highlightbackground=BORDER)
            row.pack(fill="x", pady=1)
            tk.Label(row, text=topic, font=self.fS, fg=MUTED, bg=PANEL,
                     anchor="w", width=28).pack(side="left", padx=(7, 3), pady=4)
            dot = tk.Label(row, text="●", font=self.fS, fg=MUTED, bg=PANEL)
            dot.pack(side="left")
            val = tk.StringVar(value="—")
            tk.Label(row, textvariable=val, font=self.fM, fg=color, bg=PANEL,
                     width=12, anchor="e").pack(side="left", padx=3)
            tk.Label(row, text=unit, font=self.fL, fg=MUTED, bg=PANEL,
                     width=3, anchor="w").pack(side="left")
            age = tk.StringVar(value="no data")
            tk.Label(row, textvariable=age, font=self.fS, fg=MUTED, bg=PANEL,
                     width=10, anchor="e").pack(side="right", padx=6)
            self.rows[key] = {"val": val, "age": age, "dot": dot, "fmt": fmt}

        # History
        self._section("HISTORY  (~30 s)")
        self.cv_hist = tk.Canvas(self.root, height=110, bg=PANEL,
                                 highlightthickness=1, highlightbackground=BORDER)
        self.cv_hist.pack(fill="both", expand=True, padx=14, pady=(4, 12))

    # ── Drawing ───────────────────────────────────────────────────────────────
    def _draw_bar(self, g, shown):
        c = g["canvas"]
        c.delete("all")
        w = c.winfo_width()
        if w < 4:
            return
        frac = max(0.0, min(1.0, (shown - g["lo"]) / (g["hi"] - g["lo"])))
        c.create_rectangle(0, 0, w, 6, fill=BORDER, outline="")
        if frac > 0:
            c.create_rectangle(0, 0, int(frac * w), 6, fill=g["color"], outline="")

    def _draw_history(self):
        c = self.cv_hist
        c.delete("all")
        cw, ch = c.winfo_width(), c.winfo_height()
        if cw < 10:
            return
        col_w = cw // len(GAUGES)
        for idx, (label, key, unit, color, lo, hi, scale) in enumerate(GAUGES):
            x0 = idx * col_w
            if idx > 0:
                c.create_line(x0, 0, x0, ch, fill=BORDER)
            c.create_text(x0 + 4, 3, text=label, anchor="nw", font=self.fS, fill=MUTED)
            vals = self.history[key]
            if not vals:
                continue
            span = hi - lo
            n = len(vals)
            pts = []
            for i, v in enumerate(vals):
                x = x0 + 1 + i * (col_w - 2) / max(n - 1, 1)
                y = ch - 4 - (v - lo) / span * (ch - 8)
                pts.extend([x, max(2, min(ch - 2, y))])
            if len(pts) >= 4:
                c.create_line(*pts, fill=color, width=1.5)
            c.create_text(x0 + col_w - 3, max(10, min(ch - 6, pts[-1])),
                          text=f"{vals[-1]:.0f}", anchor="e", font=self.fS, fill=color)

    # ── Main tick ─────────────────────────────────────────────────────────────
    def _tick(self):
        d = self.node.data
        now = time.time()

        # Connection status: live if any topic updated in the last 2 s
        newest = max(self.node.last_update.values(), default=0)
        if newest == 0:
            self.lbl_status.config(text="● WAITING FOR DATA", fg=MUTED)
        elif now - newest < 2.0:
            self.lbl_status.config(text="● LIVE", fg=GOOD)
        else:
            self.lbl_status.config(text=f"● STALE ({now - newest:.0f}s)", fg=BAD)

        # Threshold colours
        if "soc" in d:
            s = d["soc"]
            self.gauges["soc"]["color"] = GOOD if s > 0.3 else (WARN if s > 0.15 else BAD)
        if "temperature" in d:
            t = d["temperature"]
            self.gauges["temperature"]["color"] = GOOD if t < 35 else (WARN if t < 45 else BAD)

        # Gauges + history
        for key, g in self.gauges.items():
            if key not in d:
                continue
            shown = d[key] * g["scale"]
            g["var"].set(f"{shown:.1f}")
            g["lbl"].config(fg=g["color"])
            self._draw_bar(g, shown)
            h = self.history[key]
            h.append(shown)
            if len(h) > self.HIST_LEN:
                h.pop(0)
        self._draw_history()

        # Topic rows
        for key, row in self.rows.items():
            if key not in d:
                continue
            row["val"].set(row["fmt"](d[key]))
            age = now - self.node.last_update[key]
            if age < 0.5:
                row["dot"].config(fg=GOOD)
                row["age"].set("live")
            elif age < 2.0:
                row["dot"].config(fg=WARN)
                row["age"].set(f"{age:.1f}s ago")
            else:
                row["dot"].config(fg=BAD)
                row["age"].set(f"{age:.0f}s ago")

        self.root.after(100, self._tick)


# ── Entry ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = BatterySubscriber()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    BatteryViewer(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

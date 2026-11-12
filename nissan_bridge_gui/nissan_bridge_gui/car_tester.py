#!/usr/bin/env python3
"""
car_tester.py  —  Manual command publisher for the cRIO control cluster (ID 5)

Publishes (relative topics, so a namespace can be applied):
    cmd_vel                    geometry_msgs/Twist      linear.x = speed, angular.z = steering
    control/accel_cmd          Float64Stamped
    control/brake_cmd          Float64Stamped
    control/control_state_cmd  std_msgs/Int32           1..5
    control/buzzer_cmd         std_msgs/Bool            true only while the button is held

Values are published on every change; with CONTINUOUS enabled all topics are re-published at 10 Hz.

Usage:
    ros2 run nissan_bridge_gui car_tester --ros-args -r __ns:=/nissan9
    ros2 launch nissan_bridge_gui car_tester.launch.py            # namespace:=nissan9 by default
"""

import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from geometry_msgs.msg import Twist
from nissan_bridge_msgs.msg import Float64Stamped
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

# ── Palette (same as battery_viewer) ──────────────────────────────────────────
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
TROUGH = "#2a3042"

CONTROL_STATES = [
    (1, "MANUAL"),
    (2, "AUTONOMOUS_SPEED_REF"),
    (3, "AUTONOMOUS_PEDAL_REF"),
    (4, "ERROR"),
    (5, "TESTING"),
]

CONTINUOUS_PERIOD_MS = 100   # 10 Hz
TICK_MS = 200                # GUI status refresh


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class CarTesterNode(Node):
    # key -> (topic, message type)
    TOPICS = {
        "cmd_vel":       ("cmd_vel", Twist),
        "accel":         ("control/accel_cmd", Float64Stamped),
        "brake":         ("control/brake_cmd", Float64Stamped),
        "control_state": ("control/control_state_cmd", Int32),
        "buzzer":        ("control/buzzer_cmd", Bool),
    }

    def __init__(self):
        super().__init__("car_tester_gui")
        self.speed_limit = self.declare_parameter("speed_limit", 10.0).value      # m/s, 0..limit
        self.steer_limit = self.declare_parameter("steer_limit", 1.0).value       # rad/s, ±
        self.pedal_max = self.declare_parameter("pedal_max", 100.0).value         # accel/brake
        self.frame_id = self.declare_parameter("frame_id", "base_link").value

        self.pub = {}
        self.tx = {}
        self.rx = {}
        self.last = {}   # last value shown in the table
        for key, (topic, msg_type) in self.TOPICS.items():
            self.pub[key] = self.create_publisher(msg_type, topic, 10)
            self.tx[key] = 0
            self.rx[key] = 0
            # Own subscription confirms the message actually goes through the middleware
            self.create_subscription(
                msg_type, topic, lambda _msg, k=key: self._on_rx(k), 10)

    def _on_rx(self, key):
        self.rx[key] += 1

    def other_subscribers(self, key):
        # Minus our own echo subscription
        return max(0, self.pub[key].get_subscription_count() - 1)

    def publish_cmd_vel(self, speed, steer):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(steer)
        self._send("cmd_vel", msg, f"v={speed:+.2f} ω={steer:+.2f}")

    def publish_float(self, key, value):
        msg = Float64Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.data = float(value)
        self._send(key, msg, f"{value:.2f}")

    def publish_control_state(self, state):
        self._send("control_state", Int32(data=int(state)), str(state))

    def publish_buzzer(self, on):
        self._send("buzzer", Bool(data=bool(on)), "TRUE" if on else "FALSE")

    def _send(self, key, msg, shown):
        self.pub[key].publish(msg)
        self.tx[key] += 1
        self.last[key] = shown


# ── GUI ───────────────────────────────────────────────────────────────────────

class CarTester:
    def __init__(self, root: tk.Tk, node: CarTesterNode):
        self.root = root
        self.node = node
        self.rows = {}
        self.state_buttons = {}

        self.speed = tk.DoubleVar(value=0.0)
        self.steer = tk.DoubleVar(value=0.0)
        self.accel = tk.DoubleVar(value=0.0)
        self.brake = tk.DoubleVar(value=0.0)
        self.control_state = 1
        self.buzzer = False
        self.continuous = tk.BooleanVar(value=False)

        root.title("Car Tester")
        root.configure(bg=BG)
        root.geometry("780x700")

        self.fT = tkfont.Font(family="Courier", size=10, weight="bold")
        self.fV = tkfont.Font(family="Courier", size=12, weight="bold")
        self.fL = tkfont.Font(family="Courier", size=9)
        self.fS = tkfont.Font(family="Courier", size=8)
        self.fM = tkfont.Font(family="Courier", size=10, weight="bold")

        self._build()
        self._select_state(self.control_state, publish=False)
        self._continuous_loop()
        self._tick()

    # ── Layout helpers ────────────────────────────────────────────────────────
    def _section(self, text):
        tk.Frame(self.root, bg=BORDER, height=1).pack(fill="x", padx=14, pady=(10, 4))
        tk.Label(self.root, text=text, font=self.fL, fg=MUTED, bg=BG).pack(anchor="w", padx=14)

    def _slider(self, label, unit, var, lo, hi, color, on_change):
        row = tk.Frame(self.root, bg=PANEL, highlightthickness=1, highlightbackground=BORDER)
        row.pack(fill="x", padx=14, pady=2)
        tk.Label(row, text=label, font=self.fL, fg=MUTED, bg=PANEL,
                 width=18, anchor="w").pack(side="left", padx=(8, 0), pady=8)
        shown = tk.StringVar()
        tk.Label(row, textvariable=shown, font=self.fV, fg=color, bg=PANEL,
                 width=7, anchor="e").pack(side="left")
        tk.Label(row, text=unit, font=self.fS, fg=MUTED, bg=PANEL,
                 width=6, anchor="w").pack(side="left", padx=(4, 8))

        def changed(_=None):
            shown.set(f"{var.get():+.2f}" if lo < 0 else f"{var.get():.2f}")
            on_change()

        tk.Button(row, text="0", font=self.fS, fg=TEXT, bg=BORDER, activebackground=MUTED,
                  relief="flat", width=2, command=lambda: (var.set(0.0), changed())
                  ).pack(side="right", padx=8)
        tk.Label(row, text=f"{lo:g}", font=self.fS, fg=MUTED, bg=PANEL,
                 width=4, anchor="e").pack(side="left")
        tk.Scale(row, variable=var, from_=lo, to=hi, resolution=(hi - lo) / 400.0,
                 orient="horizontal", showvalue=False, command=changed,
                 bg=color, fg=TEXT, troughcolor=TROUGH, activebackground=TEXT,
                 highlightthickness=1, highlightbackground=MUTED, bd=1, relief="sunken",
                 sliderrelief="raised", sliderlength=30, width=22
                 ).pack(side="left", fill="x", expand=True, padx=4, pady=6)
        tk.Label(row, text=f"{hi:g}", font=self.fS, fg=MUTED, bg=PANEL,
                 width=4, anchor="w").pack(side="left")
        shown.set(f"{var.get():+.2f}" if lo < 0 else f"{var.get():.2f}")

    def _build(self):
        n = self.node

        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=14, pady=(12, 0))
        tk.Label(top, text="CAR TESTER", font=self.fT, fg=ORANGE, bg=BG).pack(side="left")
        tk.Label(top, text=f"  ns: {n.get_namespace()}", font=self.fL, fg=MUTED,
                 bg=BG).pack(side="left")
        self.lbl_status = tk.Label(top, text="● NO SUBSCRIBER", font=self.fL, fg=MUTED, bg=BG)
        self.lbl_status.pack(side="right")

        self._section("CMD_VEL  (geometry_msgs/Twist)")
        self._slider("SPEED  linear.x", "m/s", self.speed, 0.0, n.speed_limit,
                     ACCENT, self._send_cmd_vel)
        self._slider("STEER  angular.z", "rad/s", self.steer, -n.steer_limit, n.steer_limit,
                     ACCENT, self._send_cmd_vel)

        self._section("PEDALS  (Float64Stamped)")
        self._slider("ACCEL  accel_cmd", "", self.accel, 0.0, n.pedal_max, GOOD,
                     lambda: n.publish_float("accel", self.accel.get()))
        self._slider("BRAKE  brake_cmd", "", self.brake, 0.0, n.pedal_max, ORANGE,
                     lambda: n.publish_float("brake", self.brake.get()))

        self._section("CONTROL STATE  (std_msgs/Int32)")
        sf = tk.Frame(self.root, bg=BG)
        sf.pack(fill="x", padx=14, pady=(4, 0))
        for col, (state, name) in enumerate(CONTROL_STATES):
            b = tk.Button(sf, text=f"[{state}]\n{name}", font=self.fS, relief="flat", bd=0,
                          height=3, command=lambda s=state: self._select_state(s))
            b.grid(row=0, column=col, padx=2, sticky="nsew")
            sf.columnconfigure(col, weight=1, uniform="state")
            self.state_buttons[state] = b

        self._section("BUZZER  (std_msgs/Bool)  ·  ACTIONS")
        af = tk.Frame(self.root, bg=BG)
        af.pack(fill="x", padx=14, pady=(4, 0))
        self.btn_buzzer = tk.Label(af, text="BUZZER  (hold)", font=self.fM, fg=BG, bg=MUTED,
                                   width=18, pady=8, cursor="hand2")
        self.btn_buzzer.pack(side="left")
        self.btn_buzzer.bind("<ButtonPress-1>", lambda _e: self._set_buzzer(True))
        self.btn_buzzer.bind("<ButtonRelease-1>", lambda _e: self._set_buzzer(False))
        tk.Button(af, text="STOP  (all → 0)", font=self.fL, fg=BAD, bg=PANEL,
                  activebackground=BORDER, activeforeground=BAD, relief="flat",
                  highlightthickness=1, highlightbackground=BAD, padx=10, pady=6,
                  command=self._stop).pack(side="right")
        tk.Checkbutton(af, text="CONTINUOUS @ 10 Hz", variable=self.continuous, font=self.fS,
                       fg=TEXT, bg=BG, selectcolor=PANEL, activebackground=BG,
                       activeforeground=TEXT, highlightthickness=0).pack(side="right", padx=10)

        self._section("PUBLISHED TOPICS   tx = sent · rx = received back · subs = other subscribers")
        tf = tk.Frame(self.root, bg=BG)
        tf.pack(fill="x", padx=14, pady=(4, 12))
        ns = n.get_namespace().rstrip("/")
        for key, (topic, _) in CarTesterNode.TOPICS.items():
            row = tk.Frame(tf, bg=PANEL, highlightthickness=1, highlightbackground=BORDER)
            row.pack(fill="x", pady=1)
            tk.Label(row, text=f"{ns}/{topic}", font=self.fS, fg=MUTED, bg=PANEL,
                     anchor="w", width=34).pack(side="left", padx=(7, 3), pady=4)
            dot = tk.Label(row, text="●", font=self.fS, fg=MUTED, bg=PANEL)
            dot.pack(side="left")
            val = tk.StringVar(value="—")
            tk.Label(row, textvariable=val, font=self.fM, fg=TEXT, bg=PANEL,
                     width=18, anchor="e").pack(side="left", padx=3)
            stats = tk.StringVar()
            tk.Label(row, textvariable=stats, font=self.fS, fg=MUTED, bg=PANEL,
                     anchor="e").pack(side="right", padx=6)
            self.rows[key] = {"val": val, "stats": stats, "dot": dot}

    # ── Actions ───────────────────────────────────────────────────────────────
    def _send_cmd_vel(self):
        self.node.publish_cmd_vel(self.speed.get(), self.steer.get())

    def _select_state(self, state, publish=True):
        self.control_state = state
        for s, b in self.state_buttons.items():
            if s == state:
                b.config(bg=ACCENT, fg=BG, activebackground=ACCENT, activeforeground=BG)
            else:
                b.config(bg=PANEL, fg=MUTED, activebackground=BORDER, activeforeground=TEXT)
        if publish:
            self.node.publish_control_state(state)

    def _set_buzzer(self, on):
        if on == self.buzzer:
            return
        self.buzzer = on
        self.btn_buzzer.config(bg=WARN if on else MUTED)
        self.node.publish_buzzer(on)

    def _stop(self):
        for var in (self.speed, self.steer, self.accel, self.brake):
            var.set(0.0)
        self._set_buzzer(False)
        self._send_cmd_vel()
        self.node.publish_float("accel", 0.0)
        self.node.publish_float("brake", 0.0)

    def _publish_all(self):
        self._send_cmd_vel()
        self.node.publish_float("accel", self.accel.get())
        self.node.publish_float("brake", self.brake.get())
        self.node.publish_control_state(self.control_state)
        self.node.publish_buzzer(self.buzzer)

    # ── Loops ─────────────────────────────────────────────────────────────────
    def _continuous_loop(self):
        if self.continuous.get():
            self._publish_all()
        self.root.after(CONTINUOUS_PERIOD_MS, self._continuous_loop)

    def _tick(self):
        n = self.node
        matched = n.other_subscribers("cmd_vel")
        if matched:
            self.lbl_status.config(text=f"● {matched} SUBSCRIPTION(S) MATCHED", fg=GOOD)
        else:
            self.lbl_status.config(text="● NO SUBSCRIBER ON cmd_vel", fg=MUTED)

        for key, row in self.rows.items():
            if key in n.last:
                row["val"].set(n.last[key])
            subs = n.other_subscribers(key)
            # grey: nothing sent yet, yellow: sent but nobody else listens, green: subscribed
            dot = MUTED if n.tx[key] == 0 else (GOOD if subs else WARN)
            row["dot"].config(fg=dot)
            row["stats"].set(f"tx {n.tx[key]:>6}   rx {n.rx[key]:>6}   subs {subs}")

        self.root.after(TICK_MS, self._tick)


# ── Entry ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CarTesterNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    CarTester(root, node)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

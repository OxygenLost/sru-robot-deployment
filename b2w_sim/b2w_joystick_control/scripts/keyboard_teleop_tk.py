#!/usr/bin/env python3
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class KeyboardTeleopTk(Node):
    def __init__(self) -> None:
        super().__init__(
            "keyboard_teleop_tk",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self._declare_if_missing("scale_linear", 0.5)
        self._declare_if_missing("scale_linear_turbo", 1.0)
        self._declare_if_missing("scale_angular", 1.0)
        self._declare_if_missing("scale_angular_turbo", 1.5)
        self._declare_if_missing("publish_rate", 20.0)
        self._declare_if_missing("linear_step", 1.0)
        self._declare_if_missing("angular_step", 1.0)
        self._declare_if_missing("enable_on_start", False)
        self._declare_if_missing(
            "cmd_vel_topic",
            "/path_manager/path_manager_ros/nav_vel",
        )

        self.scale_linear = self.get_parameter("scale_linear").value
        self.scale_linear_turbo = self.get_parameter("scale_linear_turbo").value
        self.scale_angular = self.get_parameter("scale_angular").value
        self.scale_angular_turbo = self.get_parameter("scale_angular_turbo").value
        self.linear_step = self.get_parameter("linear_step").value
        self.angular_step = self.get_parameter("angular_step").value
        self.enabled = bool(self.get_parameter("enable_on_start").value)
        self.turbo = False
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0.0:
            publish_rate = 20.0

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_cmd)

        self._lock = threading.Lock()
        self._pressed_keys = set()
        self._root = None
        self._status_var = None

        self.get_logger().info("===========================================")
        self.get_logger().info("Keyboard Teleop (Tk) Started")
        self.get_logger().info(f"Publishing to: {self.cmd_vel_topic}")
        self.get_logger().info("Controls:")
        self.get_logger().info("  Space: Toggle enable ON/OFF")
        self.get_logger().info("  T: Toggle TURBO mode")
        self.get_logger().info("  W/S or Up/Down: Forward/Backward")
        self.get_logger().info("  A/D or Left/Right: Strafe")
        self.get_logger().info("  Q/E: Rotate Left/Right")
        self.get_logger().info("===========================================")

    def _declare_if_missing(self, name: str, default_value) -> None:
        if not self.has_parameter(name):
            self.declare_parameter(name, default_value)

    def start_ui(self) -> None:
        self._root = tk.Tk()
        self._root.title("B2W Keyboard Teleop")
        self._root.geometry("420x220")
        self._root.resizable(False, False)

        instructions = (
            "Space: Toggle enable    |    T: Toggle turbo\n"
            "W/S or Up/Down: Forward/Backward\n"
            "A/D or Left/Right: Strafe\n"
            "Q/E: Rotate Left/Right"
        )

        tk.Label(self._root, text="B2W Keyboard Teleop", font=("Helvetica", 14, "bold")).pack(pady=8)
        tk.Label(self._root, text=instructions, justify="center").pack(pady=8)

        self._status_var = tk.StringVar()
        tk.Label(self._root, textvariable=self._status_var, font=("Helvetica", 12)).pack(pady=8)
        self._update_status()

        self._root.bind("<KeyPress>", self._on_key_press)
        self._root.bind("<KeyRelease>", self._on_key_release)
        self._root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._root.after(200, self._update_status)
        self._root.mainloop()

    def _on_key_press(self, event) -> None:
        key = event.keysym.lower()
        with self._lock:
            is_new = key not in self._pressed_keys
            if is_new:
                if key == "space":
                    self.enabled = not self.enabled
                    state = "ENABLED" if self.enabled else "DISABLED"
                    self.get_logger().info(f"Control {state}")
                elif key == "t":
                    self.turbo = not self.turbo
                    state = "ON" if self.turbo else "OFF"
                    self.get_logger().info(f"Turbo {state}")
            self._pressed_keys.add(key)

    def _on_key_release(self, event) -> None:
        key = event.keysym.lower()
        with self._lock:
            self._pressed_keys.discard(key)

    def _update_status(self) -> None:
        if self._status_var is None or self._root is None:
            return
        with self._lock:
            enabled = self.enabled
            turbo = self.turbo
        status = "ENABLED" if enabled else "DISABLED"
        turbo_text = "TURBO" if turbo else "NORMAL"
        self._status_var.set(f"Status: {status} | Mode: {turbo_text}")
        self._root.after(200, self._update_status)

    def _publish_cmd(self) -> None:
        with self._lock:
            enabled = self.enabled
            turbo = self.turbo
            keys = set(self._pressed_keys)

        twist = Twist()
        if enabled:
            linear_scale = self.scale_linear_turbo if turbo else self.scale_linear
            angular_scale = self.scale_angular_turbo if turbo else self.scale_angular

            forward = ("w" in keys) or ("up" in keys)
            backward = ("s" in keys) or ("down" in keys)
            left = ("a" in keys) or ("left" in keys)
            right = ("d" in keys) or ("right" in keys)
            rot_left = "q" in keys
            rot_right = "e" in keys

            linear_x = 0.0
            if forward and not backward:
                linear_x = self.linear_step
            elif backward and not forward:
                linear_x = -self.linear_step

            linear_y = 0.0
            if right and not left:
                linear_y = -self.linear_step
            elif left and not right:
                linear_y = self.linear_step

            angular_z = 0.0
            if rot_left and not rot_right:
                angular_z = self.angular_step
            elif rot_right and not rot_left:
                angular_z = -self.angular_step

            twist.linear.x = linear_x * linear_scale
            twist.linear.y = linear_y * linear_scale
            twist.angular.z = angular_z * angular_scale

        self.cmd_vel_pub.publish(twist)

    def _on_close(self) -> None:
        self.get_logger().info("Shutting down keyboard teleop.")
        rclpy.shutdown()
        if self._root is not None:
            self._root.quit()
            self._root.destroy()


def main() -> None:
    rclpy.init()
    node = KeyboardTeleopTk()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    node.start_ui()
    spin_thread.join()


if __name__ == "__main__":
    main()

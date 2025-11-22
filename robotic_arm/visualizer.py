"""Tkinter-based 3D visualizer and controller for the robotic arm."""

from __future__ import annotations

import argparse
import sys
import threading
import tkinter as tk
from tkinter import messagebox
from typing import Iterable

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from robotic_arm.serial_controller import RoboticArmController, setup_logging


class ArmVisualizerApp:
    """Interactive 3D visualizer that can drive the robotic arm."""

    def __init__(self, controller: RoboticArmController, refresh_ms: int = 500) -> None:
        self.controller = controller
        self.refresh_ms = refresh_ms
        self._running = False
        self._target_pose: tuple[float, float, float] | None = None
        self._last_pose: dict[str, float] | None = None
        self._home_pose = _home_pose()

        self.root = tk.Tk()
        self.root.title("Robotic Arm Visualizer")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._status_label = tk.Label(self.root, text="Connecting…", anchor="w")
        self._status_label.pack(fill="x", padx=8, pady=(8, 4))

        # Text variables must be created before building the controls that reference them
        self._status_text = tk.StringVar(value="Idle")
        self._response_text = tk.StringVar(value="")

        self._build_controls()
        self._build_plot()

    def _build_controls(self) -> None:
        controls = tk.Frame(self.root)
        controls.pack(side="left", fill="y", padx=8, pady=8)

        tk.Label(controls, text="Target coordinates").grid(row=0, column=0, columnspan=2, sticky="w")

        self._entries: dict[str, tk.Entry] = {}
        labels = ["X", "Y", "Z", "A", "B", "C"]
        for idx, name in enumerate(labels, start=1):
            tk.Label(controls, text=name).grid(row=idx, column=0, sticky="e", pady=2)
            entry = tk.Entry(controls, width=12)
            entry.insert(0, "0")
            entry.grid(row=idx, column=1, sticky="w", pady=2)
            self._entries[name] = entry

        button_row = len(labels) + 1
        tk.Button(controls, text="Move (G01)", command=self._move_linear).grid(
            row=button_row, column=0, columnspan=2, sticky="ew", pady=(8, 2)
        )
        tk.Button(controls, text="Rapid (G00)", command=self._move_rapid).grid(
            row=button_row + 1, column=0, columnspan=2, sticky="ew", pady=2
        )
        tk.Button(controls, text="Camera pose (G30)", command=self._go_camera).grid(
            row=button_row + 2, column=0, columnspan=2, sticky="ew", pady=2
        )
        tk.Button(controls, text="Refresh coords (T06)", command=self._refresh_coords).grid(
            row=button_row + 3, column=0, columnspan=2, sticky="ew", pady=2
        )

        tk.Label(controls, textvariable=self._status_text, fg="gray25").grid(
            row=button_row + 4, column=0, columnspan=2, sticky="w", pady=(10, 0)
        )
        tk.Label(controls, textvariable=self._response_text, fg="gray25", wraplength=180).grid(
            row=button_row + 5, column=0, columnspan=2, sticky="w"
        )

    def _build_plot(self) -> None:
        plot_frame = tk.Frame(self.root)
        plot_frame.pack(side="right", fill="both", expand=True, padx=8, pady=8)

        figure = Figure(figsize=(6, 6))
        self._ax = figure.add_subplot(111, projection="3d")
        self._configure_axes()
        self._current_point = None
        self._target_point = None

        canvas = FigureCanvasTkAgg(figure, master=plot_frame)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas = canvas
        self._draw_points(self._home_pose, self._home_pose, {})

    def run(self) -> None:
        self._running = True
        self._status_label.config(text="Initializing robot…")

        thread = threading.Thread(target=self._initialize_robot, daemon=True)
        thread.start()
        self.root.mainloop()

    def _initialize_robot(self) -> None:
        try:
            status = self.controller.initialize()
            self.controller.start_coordinate_monitor()
            status_text = f"Serial: {status.serial_number or '?'} | Firmware: {status.firmware_version or '?'}"
            self._set_status(status_text)
            self._schedule_update()
        except Exception as exc:  # pylint: disable=broad-except
            self._set_status(f"Initialization failed: {exc}")
            messagebox.showerror("Initialization failed", str(exc))

    def _set_status(self, text: str) -> None:
        self._status_label.config(text=text)
        self._status_text.set(text)

    def _schedule_update(self) -> None:
        if not self._running:
            return
        self._update_from_status()
        self.root.after(self.refresh_ms, self._schedule_update)

    def _update_from_status(self) -> None:
        coordinates = self.controller.status.coordinates
        if not coordinates:
            return
        parsed = _parse_coordinate_line(coordinates)
        if not parsed:
            return
        self._last_pose = parsed
        x, y, z = parsed.get("X", 0.0), parsed.get("Y", 0.0), parsed.get("Z", 0.0)
        target = self._target_pose or self._read_target()
        self._draw_points((x, y, z), target, parsed)
        pretty = _format_pose(parsed, target)
        self._status_text.set(pretty)

    def _configure_axes(self) -> None:
        self._ax.set_xlabel("X")
        self._ax.set_ylabel("Y")
        self._ax.set_zlabel("Z")
        reach = _max_reach()
        self._ax.set_xlim(-reach, reach)
        self._ax.set_ylim(-reach, reach)
        self._ax.set_zlim(0, reach * 1.1)
        self._ax.view_init(elev=25, azim=45)
        self._ax.grid(True)

    def _draw_points(
        self,
        current: tuple[float, float, float],
        target: tuple[float, float, float],
        pose: dict[str, float],
    ) -> None:
        self._ax.cla()
        self._configure_axes()

        joints = _solve_simple_arm(current, pose)
        xs, ys, zs = zip(*joints)
        self._ax.plot(xs, ys, zs, "-o", c="tab:blue", label="Arm")
        self._ax.scatter(*current, c="tab:blue", s=80)

        self._ax.scatter(*target, c="tab:orange", s=80, label="Target")
        self._ax.plot([current[0], target[0]], [current[1], target[1]], [current[2], target[2]], "k--", alpha=0.3)

        # Reference: home pose along the vertical plane through the base
        self._ax.scatter(*self._home_pose, c="gray", s=50, alpha=0.7, label="Home")
        self._ax.plot([0, self._home_pose[0]], [0, self._home_pose[1]], [0, self._home_pose[2]], "gray", alpha=0.3)

        orient_vec = _orientation_vector(pose)
        end = current
        self._ax.quiver(
            end[0],
            end[1],
            end[2],
            orient_vec[0],
            orient_vec[1],
            orient_vec[2],
            length=60,
            normalize=True,
            color="tab:green",
            label="Tool axis",
        )

        self._ax.legend(loc="upper right")
        self._canvas.draw()

    def _read_target(self) -> tuple[float, float, float]:
        try:
            return (
                float(self._entries["X"].get()),
                float(self._entries["Y"].get()),
                float(self._entries["Z"].get()),
            )
        except ValueError:
            return 0.0, 0.0, 0.0

    def _refresh_coords(self) -> None:
        try:
            response = self.controller.send_command("T06")
            self.controller.status.coordinates = response
            self._response_text.set(response)
        except Exception as exc:  # pylint: disable=broad-except
            self._response_text.set(f"Error: {exc}")

    def _move_linear(self) -> None:
        self._send_motion_command("G01")

    def _move_rapid(self) -> None:
        self._send_motion_command("G00")

    def _go_camera(self) -> None:
        try:
            response = self.controller.send_command("G30")
            self._response_text.set(response)
        except Exception as exc:  # pylint: disable=broad-except
            self._response_text.set(f"Error: {exc}")

    def _send_motion_command(self, opcode: str) -> None:
        try:
            coords = {axis: float(entry.get()) for axis, entry in self._entries.items()}
        except ValueError:
            messagebox.showwarning("Invalid input", "Coordinates must be numeric.")
            return

        target = (coords["X"], coords["Y"], coords["Z"])
        if not _within_workspace(target):
            messagebox.showwarning(
                "Out of range",
                "目标超出工作空间：以基座为中心，臂展在竖直平面的中心附近，请减小半径或高度后再试。",
            )
            return

        command = (
            f"{opcode} "
            f"X{coords['X']} Y{coords['Y']} Z{coords['Z']} "
            f"A{coords['A']} B{coords['B']} C{coords['C']}"
        )
        try:
            response = self.controller.send_command(command)
            self._response_text.set(response)
            self._target_pose = (
                coords["X"],
                coords["Y"],
                coords["Z"],
            )
            self._animate_preview()
        except Exception as exc:  # pylint: disable=broad-except
            self._response_text.set(f"Error: {exc}")

    def _animate_preview(self) -> None:
        if not self._target_pose:
            return

        start_pose = self._last_pose or {}
        start = (
            float(start_pose.get("X", 0.0)),
            float(start_pose.get("Y", 0.0)),
            float(start_pose.get("Z", 0.0)),
        )
        end = self._target_pose
        steps = 15

        def _step(idx: int) -> None:
            if not self._running:
                return
            t = idx / steps
            interp = (
                start[0] + (end[0] - start[0]) * t,
                start[1] + (end[1] - start[1]) * t,
                start[2] + (end[2] - start[2]) * t,
            )
            pose = dict(start_pose)
            pose.update({"X": interp[0], "Y": interp[1], "Z": interp[2]})
            self._draw_points(interp, end, pose)
            if idx < steps:
                self.root.after(30, lambda: _step(idx + 1))

        _step(0)

    def _on_close(self) -> None:
        self._running = False
        self.controller.graceful_shutdown()
        self.root.destroy()


def _parse_coordinate_line(text: str) -> dict[str, float]:
    values: dict[str, float] = {}
    for token in text.split():
        if token.startswith("RX"):
            key, value = "A", token[2:]
        elif token.startswith("RY"):
            key, value = "B", token[2:]
        elif token.startswith("RZ"):
            key, value = "C", token[2:]
        else:
            key, value = token[0], token[1:]
        try:
            values[key] = float(value)
        except ValueError:
            continue
    return values


def _format_pose(values: dict[str, float], target: tuple[float, float, float] | None) -> str:
    pieces = []
    for key in ("X", "Y", "Z", "A", "B", "C"):
        if key in values:
            pieces.append(f"{key}={values[key]:.2f}")
    if target:
        pieces.append(f"Target=({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})")
    return "  ".join(pieces)


def _orientation_vector(values: dict[str, float]) -> tuple[float, float, float]:
    from math import cos, radians, sin

    pitch = radians(values.get("B", 0.0))
    yaw = radians(values.get("C", 0.0))

    x = cos(pitch) * cos(yaw)
    y = cos(pitch) * sin(yaw)
    z = -sin(pitch)
    return x, y, z


def _solve_simple_arm(
    end_effector: tuple[float, float, float], values: dict[str, float]
) -> list[tuple[float, float, float]]:
    from math import atan2, cos, radians, sin, sqrt

    base = (0.0, 0.0, 0.0)

    l1, l2, l3 = _link_lengths()

    x, y, z = end_effector
    yaw = radians(values.get("C", 0.0))

    planar_dist = sqrt(x * x + y * y)
    total_dist = sqrt(planar_dist * planar_dist + z * z)
    if total_dist < 1e-6:
        return [base, base, base, base]

    reachable = min(total_dist - l3, l1 + l2 - 1e-3)
    if reachable < 0:
        reachable = 0.0
    scale = reachable / total_dist
    wrist_r = planar_dist * scale
    wrist_z = z * scale

    dist_to_wrist = sqrt(wrist_r * wrist_r + wrist_z * wrist_z)
    dist_clamped = min(max(dist_to_wrist, abs(l1 - l2) + 1e-3), l1 + l2 - 1e-3)

    cos_angle_elbow = (l1 * l1 + l2 * l2 - dist_clamped * dist_clamped) / (2 * l1 * l2)
    cos_angle_elbow = max(min(cos_angle_elbow, 1.0), -1.0)
    elbow_angle = atan2(sqrt(1 - cos_angle_elbow * cos_angle_elbow), cos_angle_elbow)

    cos_angle_shoulder = (l1 * l1 + dist_clamped * dist_clamped - l2 * l2) / (2 * l1 * dist_clamped)
    cos_angle_shoulder = max(min(cos_angle_shoulder, 1.0), -1.0)
    shoulder_angle = atan2(wrist_z, wrist_r) + atan2(
        sqrt(1 - cos_angle_shoulder * cos_angle_shoulder), cos_angle_shoulder
    )

    shoulder_r = l1 * cos(shoulder_angle)
    shoulder_z = l1 * sin(shoulder_angle)

    elbow_r = shoulder_r + l2 * cos(shoulder_angle - elbow_angle)
    elbow_z = shoulder_z + l2 * sin(shoulder_angle - elbow_angle)

    sin_yaw, cos_yaw = sin(yaw), cos(yaw)

    shoulder = (shoulder_r * cos_yaw, shoulder_r * sin_yaw, shoulder_z)
    elbow = (elbow_r * cos_yaw, elbow_r * sin_yaw, elbow_z)
    wrist = (x - l3 * cos_yaw, y - l3 * sin_yaw, z)

    return [base, shoulder, elbow, wrist]


def _link_lengths() -> tuple[float, float, float]:
    """Nominal link lengths (mm) measured from the home pose."""

    return 140.0, 140.0, 100.0


def _home_pose() -> tuple[float, float, float]:
    """Return the startup position derived from the vertical 90° posture."""

    l1, l2, l3 = _link_lengths()
    return 0.0, 0.0, l1 + l2 + l3


def _max_reach() -> float:
    """Max reach from base to tool center in any direction (mm)."""

    return sum(_link_lengths())


def _within_workspace(point: tuple[float, float, float]) -> bool:
    """Rough workspace guard to avoid sending impossible targets."""

    from math import sqrt

    reach = _max_reach()
    x, y, z = point
    radial = sqrt(x * x + y * y)
    dist = sqrt(radial * radial + z * z)
    return 0 <= z <= reach and dist <= reach


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="3D visualizer for the robotic arm")
    parser.add_argument("port", help="Serial port connected to the FT232RL (e.g. COM15)")
    parser.add_argument("--interval", type=float, default=0.5, help="Refresh interval for coordinates (seconds)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable verbose logging")
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_arguments(argv or sys.argv[1:])
    setup_logging(args.verbose)

    controller = RoboticArmController(port=args.port, coordinate_interval=args.interval)
    app = ArmVisualizerApp(controller, refresh_ms=int(args.interval * 1000))
    app.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

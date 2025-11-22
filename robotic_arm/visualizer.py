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

        self.root = tk.Tk()
        self.root.title("Robotic Arm Visualizer")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._status_label = tk.Label(self.root, text="Connecting…", anchor="w")
        self._status_label.pack(fill="x", padx=8, pady=(8, 4))

        self._build_controls()
        self._build_plot()

        self._status_text = tk.StringVar(value="Idle")
        self._response_text = tk.StringVar(value="")

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
        self._ax.set_xlabel("X")
        self._ax.set_ylabel("Y")
        self._ax.set_zlabel("Z")
        self._ax.set_xlim(-300, 300)
        self._ax.set_ylim(-300, 300)
        self._ax.set_zlim(0, 400)
        self._current_point = None
        self._target_point = None
        self._draw_points((0, 0, 0), (0, 0, 0))

        canvas = FigureCanvasTkAgg(figure, master=plot_frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas = canvas

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
        x, y, z = parsed.get("X", 0.0), parsed.get("Y", 0.0), parsed.get("Z", 0.0)
        self._draw_points((x, y, z), self._read_target())
        pretty = _format_pose(parsed)
        self._status_text.set(pretty)

    def _draw_points(self, current: tuple[float, float, float], target: tuple[float, float, float]) -> None:
        self._ax.cla()
        self._ax.set_xlabel("X")
        self._ax.set_ylabel("Y")
        self._ax.set_zlabel("Z")
        self._ax.set_xlim(-300, 300)
        self._ax.set_ylim(-300, 300)
        self._ax.set_zlim(0, 400)
        self._ax.scatter(*current, c="tab:blue", s=60, label="Current")
        self._ax.scatter(*target, c="tab:orange", s=60, label="Target")
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

        command = (
            f"{opcode} "
            f"X{coords['X']} Y{coords['Y']} Z{coords['Z']} "
            f"A{coords['A']} B{coords['B']} C{coords['C']}"
        )
        try:
            response = self.controller.send_command(command)
            self._response_text.set(response)
            self._draw_points(self._read_target(), self._read_target())
        except Exception as exc:  # pylint: disable=broad-except
            self._response_text.set(f"Error: {exc}")

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


def _format_pose(values: dict[str, float]) -> str:
    pieces = []
    for key in ("X", "Y", "Z", "A", "B", "C"):
        if key in values:
            pieces.append(f"{key}={values[key]:.2f}")
    return "  ".join(pieces)


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

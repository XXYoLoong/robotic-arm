"""Command-line interface for the robotic arm controller."""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable

from robotic_arm.serial_controller import RoboticArmController, setup_logging


def parse_arguments(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Six-axis robotic arm controller")
    parser.add_argument("port", help="Serial port connected to the FT232RL (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Coordinate refresh interval in seconds (default: 1.0)",
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose debug logging"
    )
    parser.add_argument(
        "--command",
        help="Send a one-off command after initialization (e.g. 'G30' or 'M02 F200')",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Open an interactive prompt after startup for manual commands",
    )
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_arguments(argv or sys.argv[1:])
    setup_logging(args.verbose)

    controller = RoboticArmController(port=args.port, coordinate_interval=args.interval)
    try:
        status = controller.initialize()
        print(f"Serial: {status.serial_number}  Firmware: {status.firmware_version}")

        controller.start_coordinate_monitor()

        if args.command:
            response = controller.send_command(args.command)
            print(f"Response to '{args.command}': {response}")

        if args.interactive:
            _run_interactive_loop(controller)
        else:
            # Keep the monitor running until interrupted.
            print("Monitoring coordinates. Press Ctrl+C to exit.")
            while True:
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping controller...")
    except Exception as exc:  # pylint: disable=broad-except
        print(f"Error: {exc}")
        return 1
    finally:
        controller.graceful_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


def _run_interactive_loop(controller: RoboticArmController) -> None:
    """Provide a simple REPL for sending commands to the arm."""

    print("Enter any arm command (e.g. 'G30', 'G04 T1.0', 'G05 X0 Y0 Z0 A0 B0 C0').")
    print("Type 'help' to see shortcuts or 'exit' to quit.")
    shortcuts = {
        "home": "G29",
        "origin": "G28",
        "camera": "G30",
        "coords": "T06",
        "mos-off": "M04 A0",
        "mos-on": "M04 A1",
        "factory": "M01",
    }

    while True:
        user_input = input("arm> ").strip()
        if user_input.lower() in {"exit", "quit"}:
            break
        if user_input.lower() == "help":
            print("Shortcuts:")
            for name, command in shortcuts.items():
                print(f"  {name:7s} -> {command}")
            continue
        if user_input.lower() in shortcuts:
            user_input = shortcuts[user_input.lower()]

        if not user_input:
            continue

        try:
            response = controller.send_command(user_input)
            print(response)
        except Exception as exc:  # pylint: disable=broad-except
            print(f"Error: {exc}")
            print("If the connection dropped, press reset on the arm and re-run the program.")

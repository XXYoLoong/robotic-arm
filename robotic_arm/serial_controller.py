"""Serial controller for the six-axis robotic arm.

This module implements a high-level controller around the provided
command set. It automatically handles the recommended startup sequence
and provides coordinate refreshing so operators can see the current
position without knowing the default coordinates in advance.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

import serial


_LOGGER = logging.getLogger(__name__)


@dataclass
class ArmStatus:
    """Represents the latest robot state."""

    serial_number: Optional[str] = None
    firmware_version: Optional[str] = None
    coordinates: Optional[str] = None


class RoboticArmController:
    """High-level controller for the six-axis robotic arm.

    The controller enforces the recommended startup procedure:
    1. Ensure power is applied and the MCU is in "host" mode.
    2. Open the serial connection (115200 8N1).
    3. Query T01, T02, and T03 successfully.
    4. Execute G29 to move to the startup position.

    After initialization, ``start_coordinate_monitor`` can be used to
    refresh coordinates continuously so operators can see real-time
    pose updates.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115_200,
        read_timeout: float = 1.0,
        coordinate_interval: float = 1.0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.read_timeout = read_timeout
        self.coordinate_interval = coordinate_interval
        self._serial: Optional[serial.Serial] = None
        self._monitor_thread: Optional[threading.Thread] = None
        self._monitor_stop = threading.Event()
        self.status = ArmStatus()

    def connect(self) -> None:
        """Open the serial port.

        The connection uses 115200 8N1 as required by the robot firmware.
        """

        _LOGGER.info("Opening serial port %s", self.port)
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.read_timeout,
        )
        # Clear any boot-time noise so the first read belongs to our first command.
        self._serial.reset_input_buffer()
        _LOGGER.debug("Serial connection ready: %s", self._serial)

    def close(self) -> None:
        """Close the serial port and stop background monitoring."""

        self._monitor_stop.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2)
        if self._serial and self._serial.is_open:
            self._serial.close()
            _LOGGER.info("Serial port %s closed", self.port)

    def send_command(self, command: str, *, response_timeout: Optional[float] = None) -> str:
        """Send a single-line command and return the response line.

        Some motions (e.g., ``G29`` startup) may take longer than the
        default serial timeout to acknowledge. ``response_timeout`` allows
        callers to override the wait window for a single command while
        continuing to poll the port until a non-empty line arrives.
        """

        if not self._serial or not self._serial.is_open:
            raise RuntimeError("Serial port is not open; call connect() first.")

        if not command.endswith("\n"):
            command += "\n"
        _LOGGER.debug("--> %s", command.strip())
        self._serial.write(command.encode("ascii"))
        self._serial.flush()

        deadline = time.time() + (response_timeout or self.read_timeout)
        response = ""
        while time.time() < deadline:
            response = self._serial.readline().decode("ascii", errors="ignore").strip()
            if response:
                _LOGGER.debug("<-- %s", response)
                return response
            _LOGGER.debug("<-- (no data yet, waiting for response)")

        raise RuntimeError("No response received from robotic arm.")

    def initialize(self) -> ArmStatus:
        """Run the startup sequence and return the populated status."""

        self._ensure_connected()
        self._query_identity()
        self._move_to_startup_position()
        return self.status

    def _ensure_connected(self) -> None:
        if not self._serial or not self._serial.is_open:
            self.connect()

    def _query_identity(self) -> None:
        # T01: keep-alive
        response = self.send_command("T01")
        if response != "T01":
            raise RuntimeError(f"Unexpected T01 response: {response}")

        # T02: serial number
        response = self.send_command("T02")
        if not response.startswith("T02 "):
            raise RuntimeError(f"Unexpected T02 response: {response}")
        self.status.serial_number = response.removeprefix("T02 ")

        # T03: firmware version
        response = self.send_command("T03")
        if not response.startswith("T03 "):
            raise RuntimeError(f"Unexpected T03 response: {response}")
        self.status.firmware_version = response.removeprefix("T03 ")

    def _move_to_startup_position(self) -> None:
        response = self.send_command("G29", response_timeout=10.0)
        if response != "OK":
            raise RuntimeError(f"Startup (G29) failed: {response}")
        _LOGGER.info("Robot moved to startup position via G29")

    def start_coordinate_monitor(self) -> None:
        """Start a background thread that polls T06 for coordinates."""

        if self._monitor_thread and self._monitor_thread.is_alive():
            return
        self._monitor_stop.clear()
        self._monitor_thread = threading.Thread(
            target=self._monitor_coordinates, name="arm-coordinate-monitor", daemon=True
        )
        self._monitor_thread.start()

    def _monitor_coordinates(self) -> None:
        while not self._monitor_stop.is_set():
            try:
                coordinates = self.send_command("T06")
                self.status.coordinates = coordinates
                _LOGGER.info("Coordinates: %s", coordinates)
            except Exception as exc:  # pylint: disable=broad-except
                _LOGGER.error("Coordinate refresh failed: %s", exc)
                _LOGGER.info(
                    "If communication was interrupted, reset the controller and re-run initialization."
                )
                self._monitor_stop.set()
                break
            time.sleep(self.coordinate_interval)

    def stop_coordinate_monitor(self) -> None:
        """Stop background coordinate monitoring."""

        self._monitor_stop.set()
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=2)

    def graceful_shutdown(self) -> None:
        """Ensure the monitor thread stops and the serial port closes."""

        try:
            self.stop_coordinate_monitor()
        finally:
            self.close()


def setup_logging(verbose: bool = False) -> None:
    """Configure log output for command-line usage."""

    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(level=level, format="[%(levelname)s] %(message)s")

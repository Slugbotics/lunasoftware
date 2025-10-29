"""Drivetrain hardware interface with optional console simulation.

This file provides a Drivetrain class with a small simulation mode so code
that depends on it (like the tester or teleop) can run without a serial
device.
"""

from typing import Optional

# Try to import pyserial (package name: pyserial, import as serial). If not
# available, we'll allow simulation mode to work without it.
try:
    import serial  # type: ignore
except Exception:
    serial = None  # type: ignore


class MockSerial:
    """Minimal Serial-like object that prints bytes written to it."""

    def write(self, data: bytes) -> int:
        try:
            text = data.decode().strip()
        except Exception:
            text = repr(data)
        print(f"[MOCK SERIAL WRITE] {text}")
        return len(data)

    def close(self) -> None:
        print("[MOCK SERIAL] closed")


def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, v))


class Drivetrain:
    def __init__(self, port: Optional[str] = None, baud: int = 9600,
                 simulate: bool = False, serial_obj: Optional[object] = None):
        """Create a drivetrain. By default it attempts to open a serial port.

        Args:
            port: path or name of serial port (COMx or /dev/tty...)
            baud: baud rate
            simulate: when True, use a MockSerial and print commands instead of
                      sending them to hardware.
            serial_obj: optional object with write(bytes) method to use
        """
        if simulate:
            self.serial = MockSerial()
            self._simulate = True
        elif serial_obj is not None:
            self.serial = serial_obj
            self._simulate = False
        else:
            if serial is None:
                raise RuntimeError(
                    "pyserial is not installed. Install with 'pip install pyserial' "
                    "or construct Drivetrain(simulate=True) for console testing.")

            if port is None:
                import sys
                if sys.platform.startswith("win"):
                    port = "COM3"
                elif sys.platform.startswith("darwin"):
                    port = "/dev/tty.usbserial"
                else:
                    port = "/dev/ttyUSB0"

            self.serial = serial.Serial(port, baud, timeout=1)
            self._simulate = False

    def tank_drive(self, left: float, right: float) -> None:
        """Send left/right tank values to the drivetrain.

        left/right are expected in [-1.0, 1.0]; we format them to two
        decimals when sending over serial.
        """
        left = _clamp(left)
        right = _clamp(right)
        cmd = f"DRIVE {left:.2f} {right:.2f}\n"
        self.serial.write(cmd.encode())
        if getattr(self, "_simulate", False):
            print(f"[DRIVETRAIN SIM] {cmd.strip()}")
        else:
            print(f"[DRIVETRAIN] {cmd.strip()}")

    def drive(self, speed: float, rotation: float) -> None:
        """Helper used by teleop: convert speed/rotation -> left/right.

        A simple mixing: left = speed + rotation, right = speed - rotation.
        """
        left = speed + rotation
        right = speed - rotation
        self.tank_drive(left, right)

    def stop(self) -> None:
        self.serial.write(b"STOP\n")
        if getattr(self, "_simulate", False):
            print("[DRIVETRAIN SIM] STOP")
        else:
            print("[DRIVETRAIN] STOP")


if __name__ == "__main__":
    # quick demo
    print("Drivetrain demo (simulation)")
    dt = Drivetrain(simulate=True)
    dt.tank_drive(1.0, 1.0)
    dt.tank_drive(1.0, -1.0)
    dt.drive(0.5, 0.2)
    dt.stop()

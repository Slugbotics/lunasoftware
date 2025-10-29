"""Drivetrain tester / simulator for a 4-wheel drivetrain.

This script imports the project's `Drivetrain` and runs a console-only
simulation that maps tank-drive inputs to four wheel motors
(front_left, rear_left, front_right, rear_right). It can run a demo
sequence or provide a simple interactive prompt.

Usage:
    python drivetrain_tester.py        # run demo sequence
    python drivetrain_tester.py --interactive
"""

import argparse
import time
from typing import Tuple

from driveTrain import Drivetrain


def map_tank_to_wheels(left: float, right: float) -> Tuple[float, float, float, float]:
    """Map left/right tank-drive inputs to four wheel motor values.

    For a 4WD tank-drive robot the two left motors get the `left` value and
    the two right motors get the `right` value. This function returns
    (front_left, rear_left, front_right, rear_right).
    """
    fl = left
    rl = left
    fr = right
    rr = right
    return fl, rl, fr, rr


def print_wheel_values(fl: float, rl: float, fr: float, rr: float) -> None:
    print(f"WHEELS -> FL: {fl:.2f}, RL: {rl:.2f}, FR: {fr:.2f}, RR: {rr:.2f}")


def demo_sequence(dt: Drivetrain) -> None:
    print("Starting demo sequence (4WD mapping)")

    seq = [
        (1.0, 1.0, 1.0),    # forward for 1s
        (1.0, -1.0, 1.0),   # turn in place for 1s
        (0.5, 0.5, 1.0),    # slow forward
        (0.0, 0.0, 0.5),    # coast
    ]

    for left, right, duration in seq:
        dt.tank_drive(left, right)
        fl, rl, fr, rr = map_tank_to_wheels(left, right)
        print_wheel_values(fl, rl, fr, rr)
        time.sleep(duration)

    dt.stop()
    print("Demo finished")


def interactive_loop(dt: Drivetrain) -> None:
    print("Interactive 4WD tester. Commands: f=forward, b=back, l=turn left, r=turn right, s=stop, q=quit")
    try:
        while True:
            cmd = input("cmd> ").strip().lower()
            if cmd == "f":
                dt.tank_drive(1.0, 1.0)
                fl, rl, fr, rr = map_tank_to_wheels(1.0, 1.0)
                print_wheel_values(fl, rl, fr, rr)
            elif cmd == "b":
                dt.tank_drive(-1.0, -1.0)
                fl, rl, fr, rr = map_tank_to_wheels(-1.0, -1.0)
                print_wheel_values(fl, rl, fr, rr)
            elif cmd == "l":
                dt.tank_drive(-0.5, 0.5)
                fl, rl, fr, rr = map_tank_to_wheels(-0.5, 0.5)
                print_wheel_values(fl, rl, fr, rr)
            elif cmd == "r":
                dt.tank_drive(0.5, -0.5)
                fl, rl, fr, rr = map_tank_to_wheels(0.5, -0.5)
                print_wheel_values(fl, rl, fr, rr)
            elif cmd == "s":
                dt.stop()
            elif cmd == "q":
                dt.stop()
                break
            else:
                # allow entering two numbers (left right)
                parts = cmd.split()
                if len(parts) == 2:
                    try:
                        left = float(parts[0])
                        right = float(parts[1])
                        dt.tank_drive(left, right)
                        fl, rl, fr, rr = map_tank_to_wheels(left, right)
                        print_wheel_values(fl, rl, fr, rr)
                    except ValueError:
                        print("Unknown command")
                else:
                    print("Unknown command")
    except (KeyboardInterrupt, EOFError):
        print("\nExiting interactive tester")
        dt.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="4WD Drivetrain tester (console simulation)")
    parser.add_argument("--interactive", action="store_true", help="Run interactive prompt")
    args = parser.parse_args()

    # Use simulation mode so this can run without hardware
    dt = Drivetrain(simulate=True)

    if args.interactive:
        interactive_loop(dt)
    else:
        demo_sequence(dt)


if __name__ == "__main__":
    main()

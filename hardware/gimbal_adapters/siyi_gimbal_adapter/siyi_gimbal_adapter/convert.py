"""Unit and sign conversions between the course gimbal topics and the SIYI driver.

Kept free of ROS so it can be tested on its own (test/test_convert.py).

Conventions on both sides (checked against the simulation and siyi_ros2):
  * pitch: negative = camera tilts down, range -90 to +25 deg on the A8 mini;
  * yaw:   positive = camera turns right (seen from above), range +-135 deg.
They agree, so every sign defaults to +1. The signs are parameters anyway, so a
different mounting (or a different firmware) is fixed on the bench, not in code.
"""
import math


def cmd_to_siyi(pitch_rad, yaw_rad, pitch_sign=1.0, yaw_sign=1.0):
    """Course command (rad or rad/s) -> SIYI command (deg or deg/s).

    Works for both rates and angles: the conversion is the same.
    Returns (yaw_deg, pitch_deg), in the order the SIYI messages list them.
    """
    return (yaw_sign * math.degrees(yaw_rad),
            pitch_sign * math.degrees(pitch_rad))


def attitude_to_course(roll_deg, pitch_deg, yaw_deg,
                       roll_sign=1.0, pitch_sign=1.0, yaw_sign=1.0):
    """SIYI measured attitude (deg) -> course attitude (rad).

    Returns (roll, pitch, yaw), the order of Vector3 x, y, z on /gimbal/attitude.
    The signs are the same ones used for commands, so a command and the
    attitude it produces always agree.
    """
    return (roll_sign * math.radians(roll_deg),
            pitch_sign * math.radians(pitch_deg),
            yaw_sign * math.radians(yaw_deg))


def at_limit(angle, lo, hi, margin):
    """True if angle is within margin of either travel limit (any unit)."""
    return angle <= lo + margin or angle >= hi - margin

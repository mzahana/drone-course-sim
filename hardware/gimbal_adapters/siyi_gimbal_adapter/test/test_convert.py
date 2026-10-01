import math

from siyi_gimbal_adapter.convert import at_limit, attitude_to_course, cmd_to_siyi


def test_rate_units_and_order():
    # 0.5 rad/s pitch, -1.0 rad/s yaw -> (yaw, pitch) in deg/s
    yaw, pitch = cmd_to_siyi(0.5, -1.0)
    assert math.isclose(yaw, -57.29578, rel_tol=1e-6)
    assert math.isclose(pitch, 28.64789, rel_tol=1e-6)


def test_signs_flip_one_axis_only():
    yaw, pitch = cmd_to_siyi(0.1, 0.2, pitch_sign=-1.0, yaw_sign=1.0)
    assert pitch < 0 < yaw


def test_attitude_round_trip():
    # A command and the attitude it produces must agree, for any signs.
    for ps, ys in [(1, 1), (-1, 1), (1, -1), (-1, -1)]:
        yaw_deg, pitch_deg = cmd_to_siyi(-0.6, 1.1, ps, ys)
        _, pitch, yaw = attitude_to_course(0.0, pitch_deg, yaw_deg,
                                           pitch_sign=ps, yaw_sign=ys)
        assert math.isclose(pitch, -0.6) and math.isclose(yaw, 1.1)


def test_straight_down():
    _, pitch, _ = attitude_to_course(0.0, -90.0, 0.0)
    assert math.isclose(pitch, -math.pi / 2)


def test_at_limit():
    assert at_limit(-89.5, -90.0, 25.0, 1.0)
    assert at_limit(24.2, -90.0, 25.0, 1.0)
    assert not at_limit(0.0, -90.0, 25.0, 1.0)

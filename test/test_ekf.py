import math
import random

import pytest

from csen934_project_pkg.localization_ekf_node import BicycleEKF, normalize_angle

DT = 0.01        # 100 Hz, like Webots
L = 2.995


def drive(ekf, seconds, speed_cmd, steer, truth):
    """Simulate a car that reaches the commanded speed with limited acceleration."""
    errors_noisy, errors_filtered = [], []
    for _ in range(int(seconds / DT)):
        x, y, yaw, v = truth
        v += max(-6.0, min(2.5, (speed_cmd - v) * 1.5)) * DT
        w = v * math.tan(steer) / L
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw = normalize_angle(yaw + w * DT)
        truth[:] = [x, y, yaw, v]

        z = (x + random.gauss(0, 0.3), y + random.gauss(0, 0.3),
             yaw + random.gauss(0, math.radians(1.0)),
             v + random.gauss(0, 0.3), w + random.gauss(0, math.radians(1.0)))
        if not ekf.initialized:
            ekf.initialize(*z)
        else:
            ekf.predict(DT, speed_cmd, steer)
            ekf.update(*z)
        errors_noisy.append((z[0] - x, z[1] - y, normalize_angle(z[2] - yaw)))
        errors_filtered.append((ekf.state[0] - x, ekf.state[1] - y,
                                normalize_angle(ekf.state[2] - yaw)))
    return errors_noisy, errors_filtered


def rms(errors, i):
    return math.sqrt(sum(e[i] ** 2 for e in errors) / len(errors))


def test_ekf_suppresses_noise_straight_and_curve():
    random.seed(934)
    ekf = BicycleEKF(wheelbase=L)
    truth = [0.0, 0.0, math.pi - 0.01, 0.0]
    n1, f1 = drive(ekf, 10.0, 10.0, 0.0, truth)       # accelerate, straight
    n2, f2 = drive(ekf, 15.0, 10.0, 0.0866, truth)    # curve (radius ~34.5 m)
    noisy, filtered = n1 + n2, f1 + f2
    for i in (0, 1):
        assert rms(filtered, i) < rms(noisy, i) / 3     # x, y: >3x less error
    assert rms(filtered, 2) < rms(noisy, 2) / 3         # yaw


@pytest.mark.parametrize("yaw", [math.pi - 0.01, -math.pi + 0.01])
def test_angle_normalization_stays_bounded(yaw):
    ekf = BicycleEKF()
    ekf.initialize(0.0, 0.0, yaw, 5.0, 0.5)
    for _ in range(100):
        ekf.predict(0.01, 5.0, 0.3)
        ekf.update(0.0, 0.0, yaw, 5.0, 0.5)
        assert -math.pi <= ekf.state[2] <= math.pi

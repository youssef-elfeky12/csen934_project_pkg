import math
import random

import pytest

from csen934_project_pkg.localization_ekf_node import BicycleEKF


def test_bicycle_ekf_reduces_position_noise():
    random.seed(934)
    ekf = BicycleEKF(wheelbase=2.7)
    ekf.initialize(0.0, 0.0, 0.0, 10.0, 0.0)

    noisy_x_error = []
    filtered_x_error = []
    noisy_y_error = []
    filtered_y_error = []

    for step in range(1, 201):
        gt_x = 10.0 * step * 0.05
        gt_y = 0.0
        noisy_x = gt_x + random.gauss(0.0, 0.3)
        noisy_y = gt_y + random.gauss(0.0, 0.3)

        ekf.predict(0.05, 10.0, 0.0)
        ekf.update(
            noisy_x,
            noisy_y,
            random.gauss(0.0, math.radians(1.0)),
            10.0 + random.gauss(0.0, 0.3),
            random.gauss(0.0, math.radians(1.0)),
        )

        noisy_x_error.append(noisy_x - gt_x)
        noisy_y_error.append(noisy_y - gt_y)
        filtered_x_error.append(ekf.state[0] - gt_x)
        filtered_y_error.append(ekf.state[1] - gt_y)

    noisy_rms = math.sqrt(
        sum(x * x + y * y for x, y in
            zip(noisy_x_error, noisy_y_error)) / 200
    )
    filtered_rms = math.sqrt(
        sum(x * x + y * y for x, y in
            zip(filtered_x_error, filtered_y_error)) / 200
    )

    assert filtered_rms < noisy_rms
    assert filtered_rms < 0.3


@pytest.mark.parametrize(
    "yaw",
    [math.pi - 0.01, -math.pi + 0.01],
)
def test_angle_normalization_stays_bounded(yaw):
    ekf = BicycleEKF()
    ekf.initialize(0.0, 0.0, yaw, 0.0, 0.0)
    ekf.predict(0.1, 0.0, 0.0)

    assert -math.pi <= ekf.state[2] <= math.pi

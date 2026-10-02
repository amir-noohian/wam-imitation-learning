import numpy as np

from wam_kinesthetic_dmps.dmp_retargeting import scale_forcing_weights


def test_retarget_scales_by_joint_displacement_and_caps_amplification():
    weights = np.ones((7, 3))
    demo_start = np.zeros(7)
    demo_goal = np.array([0.1, 0.1, -0.1, 0.1, 0.1, 0.1, 0.005])
    new_start = np.zeros(7)
    new_goal = np.array([0.2, 0.5, -0.2, -0.2, 0.1, 0.0, 0.5])

    scaled, factors = scale_forcing_weights(
        weights, demo_start, demo_goal, new_start, new_goal
    )

    np.testing.assert_allclose(factors, [2.0, 3.0, 2.0, -2.0, 1.0, 0.0, 0.0])
    np.testing.assert_allclose(scaled, factors[:, np.newaxis] * weights)


def test_low_motion_demo_joint_does_not_amplify_forcing():
    weights = np.ones((7, 2))
    demo_start = np.zeros(7)
    demo_goal = np.full(7, 0.1)
    demo_goal[6] = 0.001
    new_start = np.zeros(7)
    new_goal = np.full(7, 0.2)

    scaled, factors = scale_forcing_weights(
        weights, demo_start, demo_goal, new_start, new_goal
    )

    assert factors[6] == 0.0
    np.testing.assert_array_equal(scaled[6], np.zeros(2))
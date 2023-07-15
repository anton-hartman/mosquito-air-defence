import numpy as np
import unittest

from kalman_filter import KF

class TestKF(unittest.TestCase):
    def test_can_construct(self):
        x = 0.2
        vx = 2.3
        ax_variance = 1.2
        y = 0.1
        vy = 1.2
        ay_variance = 1.2

        kf = KF(x, vx, ax_variance, y, vy, ay_variance)
        self.assertAlmostEqual(kf.x_pos, x)
        self.assertAlmostEqual(kf.x_vel, vx)
        self.assertAlmostEqual(kf.x_accel_variance, ax_variance)
        self.assertAlmostEqual(kf.y_pos, y)
        self.assertAlmostEqual(kf.y_vel, vy)
        self.assertAlmostEqual(kf.y_accel_variance, ay_variance)
                

    def test_after_calling_predict_mean_and_cov_are_of_right_shape(self):
        x = 0.2
        vx = 2.3
        ax_variance = 1.2
        y = 0.1
        vy = 1.2
        ay_variance = 1.2

        kf = KF(x, vx, ax_variance, y, vy, ay_variance)
        kf.predict(0.1)

        self.assertEqual(kf.cov.shape, (4, 4))
        self.assertEqual(kf.mean.shape, (4, ))

    def test_calling_predict_increases_state_uncertainty(self):
        x = 0.2
        vx = 2.3
        ax_variance = 1.2
        y = 0.1
        vy = 1.2
        ay_variance = 1.2

        kf = KF(x, vx, ax_variance, y, vy, ay_variance)

        for _ in range(10):
            det_before = np.linalg.det(kf.cov)
            kf.predict(0.1)
            det_after = np.linalg.det(kf.cov)

            self.assertGreater(det_after, det_before)
            print(det_before, det_after)

    def test_calling_update_decreases_state_uncertainty(self):
        x = 0.2
        vx = 2.3
        ax_variance = 1.2
        y = 0.1
        vy = 1.2
        ay_variance = 1.2

        kf = KF(x, vx, ax_variance, y, vy, ay_variance)

        det_before = np.linalg.det(kf.cov)
        kf.update(meas_x=0.1, meas_y=0.1, meas_ax_variance=0.01, meas_ay_variance=0.01)
        det_after = np.linalg.det(kf.cov)

        self.assertLess(det_after, det_before)


# class TestKFPythonAndCppYieldSameResults(unittest.TestCase):
#     def test_yield_same_results_with_predict(self):
#         x = 0.2
#         v = 2.3

#         kf_py = KF(x, v, 1.2)

#         for i in range(10):
#             kf_py.predict(0.1)

#             self.assertTrue(np.allclose(kf_py.cov, 1e-6))
#             self.assertTrue(np.allclose(kf_py.mean, 1e-6))

#     def test_yield_same_results_with_update(self):
#         x = 0.2
#         v = 2.3

#         kf_py = KF(x, v, 1.2)

#         kf_py.update(0.1, 0.01)

#         self.assertTrue(np.allclose(kf_py.cov, 1e-6))
#         self.assertTrue(np.allclose(kf_py.mean, 1e-6))

import sys
import unittest
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_utlidar_fusion import ArrivalClockOffsetEstimator


class ArrivalClockOffsetEstimatorTest(unittest.TestCase):
    def test_uses_robust_median_and_locks_after_minimum_pairs(self):
        estimator = ArrivalClockOffsetEstimator(
            arrival_skew_ns=50_000_000,
            window_size=9,
            min_samples=5,
            fallback_offset_ns=402_228_000_000,
        )
        expected_ns = 384_391_000_000

        for index, jitter_ns in enumerate((0, 2_000_000, -1_000_000, 1_000_000)):
            arrival_ns = index * 100_000_000
            estimator.add_mid(arrival_ns, 10_000_000_000 + index * 100_000_000)
            estimator.add_utlidar(
                arrival_ns + 5_000_000,
                10_000_000_000 + index * 100_000_000 - expected_ns + jitter_ns,
            )
            self.assertFalse(estimator.locked)

        index = 4
        arrival_ns = index * 100_000_000
        estimator.add_mid(arrival_ns, 10_000_000_000 + index * 100_000_000)
        estimator.add_utlidar(
            arrival_ns + 5_000_000,
            10_000_000_000 + index * 100_000_000 - expected_ns,
        )

        self.assertTrue(estimator.locked)
        self.assertLessEqual(abs(estimator.offset_ns - expected_ns), 1_000_000)

    def test_tracks_offset_drift_with_sliding_window(self):
        estimator = ArrivalClockOffsetEstimator(
            arrival_skew_ns=50_000_000,
            window_size=5,
            min_samples=3,
            fallback_offset_ns=0,
        )

        for index in range(5):
            arrival_ns = index * 100_000_000
            mid_end_ns = 50_000_000_000 + index * 100_000_000
            estimator.add_mid(arrival_ns, mid_end_ns)
            estimator.add_utlidar(arrival_ns + 1_000_000, mid_end_ns - 400_000_000_000)
        self.assertEqual(estimator.offset_ns, 400_000_000_000)

        for index in range(5, 10):
            arrival_ns = index * 100_000_000
            mid_end_ns = 50_000_000_000 + index * 100_000_000
            estimator.add_mid(arrival_ns, mid_end_ns)
            estimator.add_utlidar(arrival_ns + 1_000_000, mid_end_ns - 401_000_000_000)

        self.assertEqual(estimator.offset_ns, 401_000_000_000)

    def test_does_not_calibrate_arrivals_outside_skew_window(self):
        estimator = ArrivalClockOffsetEstimator(
            arrival_skew_ns=10_000_000,
            window_size=5,
            min_samples=3,
            fallback_offset_ns=123,
        )
        estimator.add_mid(0, 1_000_000_000)
        estimator.add_utlidar(100_000_000, 500_000_000)
        self.assertFalse(estimator.locked)
        self.assertEqual(len(estimator.offset_samples), 0)


if __name__ == '__main__':
    unittest.main()

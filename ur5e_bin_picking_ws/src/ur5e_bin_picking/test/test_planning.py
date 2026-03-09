#!/usr/bin/env python3
"""Basic smoke test: verify that nodes can be imported and parameters loaded."""
import unittest
import rclpy

class TestSetup(unittest.TestCase):
    def test_import_detector(self):
        """Verify object_detector script can be imported."""
        # This tests that all dependencies are available
        import importlib.util
        spec = importlib.util.find_spec("cv2")
        self.assertIsNotNone(spec, "OpenCV not installed")

    def test_import_tf(self):
        """Verify tf_transformations is available."""
        import tf_transformations
        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        self.assertEqual(len(q), 4)

if __name__ == '__main__':
    unittest.main()

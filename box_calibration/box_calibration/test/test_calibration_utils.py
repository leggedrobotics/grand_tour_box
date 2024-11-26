import unittest
from box_calibration.calibration_utils import fun1, extract_image_topics, filter_yaml_by_rostopics

class TestUtils(unittest.TestCase):
    def test_helper_function(self):
        self.assertEqual(fun1(), True)

    def test_extract_image_topics(self):
        self.assertEqual(extract_image_topics("/media/fu/T7 Shield/2024-11-24-cam-imu/alphasense_front_center.bag"),
                         ["/gt_box/alphasense_driver_node/cam1"])

    def test_filter_camera_calibration_yaml(self):
        self.assertEqual(filter_yaml_by_rostopics(
            "/media/fu/T7 Shield/2024-11-05-leica-calib-all/kalibr_camcam_hesai_initial_guess.yaml",
            ["/gt_box/alphasense_driver_node/cam1", "/gt_box/alphasense_driver_node/cam2"],
            "/media/fu/T7 Shield/2024-11-05-leica-calib-all/cameraimu_calibration.yaml",
        ), True)
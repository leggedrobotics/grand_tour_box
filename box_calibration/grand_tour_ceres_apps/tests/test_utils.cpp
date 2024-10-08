//
// Created by fu on 09/09/2024.
//

#include <gtest/gtest.h>
#include <gtboxcalibration/utils.h>

void expect_vector_and_array_near(const std::vector<double> &vec, const double *arr, double tolerance) {
    for (size_t i = 0; i < vec.size(); ++i) {
        EXPECT_NEAR(vec[i], arr[i], tolerance) << "Difference at index " << i;
    }
}

class YamlTest : public ::testing::Test {
protected:
    YAML::Node example_kalibr_calibration;
    YAML::Node example_extrinsics;
    const std::string cam_name = "/gtbox/alphasense/cam0";
    const std::vector<double> intrinsics_data{700, 700, 960, 540};

    void SetUp() override {
        YAML::Node cam0_data;
        cam0_data["intrinsics"] = intrinsics_data;
        cam0_data["distortion_coeffs"] = std::vector<double>{-0.04, 0, 0.1, 0};
        cam0_data["distortion_model"] = "equidistant";
        cam0_data["resolution"] = std::vector<int>{1920, 1080};
        cam0_data["rostopic"] = cam_name;

        example_kalibr_calibration["cam0"] = cam0_data;

        std::vector<std::vector<double>> T_bundle_cam{
                {1, 0, 0, 0},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1},
        };
        YAML::Node cam0_extrinsic_data;
        cam0_extrinsic_data["T_bundle_camera"] = T_bundle_cam;
        cam0_extrinsic_data["rostopic"] = cam_name;
        example_extrinsics["cam0"] = cam0_extrinsic_data;
    }

    void TearDown() override {

    }
};

TEST_F(YamlTest, TestLoadCameraParameterPacks) {
    const auto loaded_camera_parameters = FetchIntrinsicsAsCameraPackFromYaml(example_kalibr_calibration);

    EXPECT_EQ(loaded_camera_parameters.size(), 1);
    const auto &loaded_name = loaded_camera_parameters.begin()->first;
    EXPECT_EQ(loaded_name, cam_name);
    expect_vector_and_array_near(intrinsics_data, loaded_camera_parameters.at(cam_name).fxfycxcy, 1e-5);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

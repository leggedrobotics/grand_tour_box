//
// Created by fu on 10/09/2024.
//

#include <gtest/gtest.h>
#include <gtboxcalibration/interpolation3d.h>


class InterpolationTest : public ::testing::Test {
protected:
    Eigen::Vector3d zero, one, correct_answer;
    unsigned long long t1, t2;
    double alpha;

    void SetUp() override {
        zero.setZero();
        one.setConstant(1.0);
        correct_answer.setConstant(0.5);
        t2 = 10000;
        alpha = double(5000) / double(t2);
    }
};

TEST_F(InterpolationTest, TestSimple) {
    Eigen::Vector3d result = lerp(zero, one, alpha);
    EXPECT_TRUE(result.isApprox(correct_answer));

    Eigen::Vector3d wrong_result = lerp(zero, one, alpha-.1);
    EXPECT_FALSE(wrong_result.isApprox(correct_answer));
}

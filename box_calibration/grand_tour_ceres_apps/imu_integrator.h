#pragma once

#include <Eigen/Geometry>

// Integrates raw IMU measurements into a navigation state using Euler integration.
// Biases are held constant (no bias evolution model).
// Gravity is applied in the world frame: g_world = [0, 0, -9.81] by default (z-up).
class ImuIntegrator {
public:
    struct State {
        Eigen::Vector3d    position;     // p_world
        Eigen::Vector3d    velocity;     // v_world
        Eigen::Quaterniond orientation;  // R_world_imu
        Eigen::Vector3d    acc_bias;     // b_a (body frame)
        Eigen::Vector3d    gyro_bias;    // b_g (body frame)
        double             timestamp_s;
    };

    explicit ImuIntegrator(const State& initial_state,
                           const Eigen::Vector3d& gravity_world = {0.0, 0.0, -9.81});

    // Feed one IMU sample. Measurements are expected in the IMU body frame.
    // angular_velocity: rad/s.  linear_acceleration: m/s².
    // Ignores samples with dt <= 0.
    void integrate(double timestamp_s,
                   const Eigen::Vector3d& angular_velocity,
                   const Eigen::Vector3d& linear_acceleration);

    const State& state() const { return state_; }

private:
    State          state_;
    Eigen::Vector3d gravity_;
};
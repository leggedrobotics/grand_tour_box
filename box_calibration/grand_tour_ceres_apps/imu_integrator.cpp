#include "imu_integrator.h"

ImuIntegrator::ImuIntegrator(const State& initial_state,
                             const Eigen::Vector3d& gravity_world)
    : state_(initial_state), gravity_(gravity_world) {}

void ImuIntegrator::integrate(double timestamp_s,
                              const Eigen::Vector3d& angular_velocity,
                              const Eigen::Vector3d& linear_acceleration) {
    const double dt = timestamp_s - state_.timestamp_s;
    if (dt <= 0.0) return;

    // Correct for biases.
    const Eigen::Vector3d gyro  = angular_velocity   - state_.gyro_bias;
    const Eigen::Vector3d accel = linear_acceleration - state_.acc_bias;

    // Integrate orientation: exact rotation for constant angular velocity over dt.
    const double angle = gyro.norm() * dt;
    if (angle > 1e-10) {
        state_.orientation = (state_.orientation *
                              Eigen::Quaterniond(Eigen::AngleAxisd(angle, gyro.normalized()))
                             ).normalized();
    }

    // Rotate specific force to world frame and add gravity to get true acceleration.
    const Eigen::Vector3d accel_world = state_.orientation * accel + gravity_;

    // Euler integration of position and velocity.
    state_.position  += state_.velocity * dt + 0.5 * accel_world * dt * dt;
    state_.velocity  += accel_world * dt;
    state_.timestamp_s = timestamp_s;
}
#pragma once

#include <Eigen/Dense>
#include "falcon/utils.h"
#include "falcon/math/common.h"
#include "attitude.h"
#include "velocity.h"


namespace falcon {
namespace state {

class StateManager {
private:
VelocitySensor sensor_velocity_;
AttitudeSensor sensor_attitude_;

double tm1_;

math::Accumulator<float, 3> acc_accelerometer_;
math::Accumulator<float, 3> acc_gyro_;
math::Accumulator<float, 3> acc_magnetometer_;
math::Accumulator<float, 3> acc_gps_velocity_;

public:
StateManager(AttitudeSensorParams attitude_params, VelocitySensorParams velocity_params, double t0);
void SpinOnce(double t);

void PostAccelerometer(const Eigen::Vector3f& acc);
void PostGyro(const Eigen::Vector3f& omega);
void PostMagnetometer(const Eigen::Vector3f& mag);
void PostGPSVelocity(const Eigen::Vector3f& vel);

const Eigen::Vector4f& GetAttitude();
const Eigen::Vector3f& GetLinearVelocity();
const Eigen::Vector3f& GetAngularVelocity();
};

}
}
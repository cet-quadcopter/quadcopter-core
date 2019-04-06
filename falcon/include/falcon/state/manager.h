#pragma once

#include <Eigen/Dense>
#include "falcon/utils.h"
#include "falcon/math/common.h"
#include "attitude.h"
#include "linear_velocity.h"
#include "angular_velocity.h"
#include "gravity.h"
#include "linear_acceleration.h"


namespace falcon {
namespace state {

struct SensorParams {
  AttitudeSensorParams attitude;
  LinearVelocitySensorParams linear_velocity;
  AngularVelocitySensorParams angular_velocity;
  GravitySensorParams gravity;
};


class StateManager {
private:
LinearVelocitySensor sensor_linear_velocity_;
AngularVelocitySensor sensor_angular_velocity_;
AttitudeSensor sensor_attitude_;
GravitySensor sensor_gravity_;
LinearAccelerationSensor sensor_linear_acceleration_;

double tm1_;

math::Accumulator<float, 3> acc_accelerometer_;
math::Accumulator<float, 3> acc_gyro_;
math::Accumulator<float, 3> acc_magnetometer_;
math::Accumulator<float, 3> acc_gps_velocity_;

public:
StateManager(SensorParams params, double t0);
void SpinOnce(double t);

void PostAccelerometer(const Eigen::Vector3f& acc);
void PostGyro(const Eigen::Vector3f& omega);
void PostMagnetometer(const Eigen::Vector3f& mag);
void PostGPSVelocity(const Eigen::Vector3f& vel);

const Eigen::Vector4f& GetAttitude();
const Eigen::Vector3f& GetLinearVelocity();
const Eigen::Vector3f& GetAngularVelocity();
const Eigen::Vector3f& GetGravity();
const Eigen::Vector3f& GetLinearAcceleration();
};

}
}
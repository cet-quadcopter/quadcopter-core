#define DEBUG_STATE_MANAGER = 0

#include "falcon/state/manager.h"

#include <math.h>
#include <iostream>

#include "falcon/math/quaternion.h"


using namespace falcon::math;
using namespace falcon::state;
using namespace Eigen;


StateManager::StateManager(SensorParams params, double t0)
: sensor_attitude_(params.attitude), sensor_linear_velocity_(params.linear_velocity), 
  sensor_angular_velocity_(params.angular_velocity), sensor_gravity_(params.gravity), 
  sensor_linear_acceleration_(), tm1_(t0) {}

void StateManager::SpinOnce(double t) {
  if (!acc_accelerometer_.HasData() || !acc_gyro_.HasData() || !acc_magnetometer_.HasData()) {
    return;
  }

  float dt = t - tm1_;
  Vector3f a_b = acc_accelerometer_.Get();
  Vector3f m_b = acc_magnetometer_.Get();
  Vector3f omega_b = acc_gyro_.Get();

  sensor_gravity_.PostInput(a_b, dt);
  sensor_attitude_.PostControlInput(omega_b, dt);
  sensor_attitude_.PostMeasurementInput(a_b, m_b);

  Vector4f attitude = GetAttitude();
  Vector3f gravity = GetGravity();

  sensor_linear_acceleration_.PostInput(a_b, gravity, attitude);

  Vector3f linear_acceleration = GetLinearAcceleration();

  sensor_linear_velocity_.PostControlInput(linear_acceleration, dt);
  sensor_angular_velocity_.PostInput(omega_b, attitude, dt);

  acc_accelerometer_.Reset();
  acc_magnetometer_.Reset();
  acc_gyro_.Reset();
  tm1_ = t;

  if (acc_gps_velocity_.HasData()) {
    Vector3f v_n = acc_gps_velocity_.Get();
    sensor_linear_velocity_.PostMeasurementInput(v_n);
    acc_gps_velocity_.Reset();
  }

#ifdef DEBUG_STATE_MANAGER
  std::cout << "Attitude" << std::endl;
  std::cout << QuaternionToEuler123(attitude) * 180 / M_PI << std::endl;

  std::cout << "Gravity" << std::endl;
  std::cout << gravity << std::endl;

  std::cout << "Linear acceleration" << std::endl;
  std::cout << linear_acceleration << std::endl;

  std::cout << "Linear velocity" << std::endl;
  std::cout << GetLinearVelocity() << std::endl;

  std::cout << "Angular velocity" << std::endl;
  std::cout << GetAngularVelocity() << std::endl;
#endif
}

void StateManager::PostAccelerometer(const Vector3f& a_b) {
  acc_accelerometer_.Post(a_b);
}

void StateManager::PostGyro(const Vector3f& omega_b) {
  acc_gyro_.Post(omega_b);
}

void StateManager::PostMagnetometer(const Vector3f& m_b) {
  acc_magnetometer_.Post(m_b);
}

void StateManager::PostGPSVelocity(const Vector3f& v_n) {
  acc_gps_velocity_.Post(v_n);
}

const Vector4f& StateManager::GetAttitude() const {
  return sensor_attitude_.GetAttitude();
}

const Vector3f& StateManager::GetLinearVelocity() const {
  return sensor_linear_velocity_.GetLinearVelocity();
}

const Vector3f& StateManager::GetAngularVelocity() const {
  return sensor_angular_velocity_.GetAngularVelocity();
}

const Vector3f& StateManager::GetGravity() const {
  return sensor_gravity_.GetGravity();
}

const Vector3f& StateManager::GetLinearAcceleration() const {
  return sensor_linear_acceleration_.GetLinearAcceleration();
}
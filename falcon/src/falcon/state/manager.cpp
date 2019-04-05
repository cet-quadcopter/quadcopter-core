#include "falcon/state/manager.h"

#include <spdlog/spdlog.h>


using namespace falcon::math;
using namespace falcon::state;
using namespace Eigen;


StateManager::StateManager(AttitudeSensorParams attitude_params, VelocitySensorParams velocity_params, double t0)
: sensor_attitude_(attitude_params), sensor_velocity_(velocity_params), tm1_(t0) {}

void StateManager::SpinOnce(double t) {
  if (!acc_accelerometer_.HasData() || !acc_gyro_.HasData() || !acc_magnetometer_.HasData() || !acc_gps_velocity_.HasData()) {
    spdlog::info("Skipping spin due to lack of data");
    return;
  }

  float dt = t - tm1_;
  Vector3f a_b = acc_accelerometer_.Get();
  Vector3f m_b = acc_magnetometer_.Get();
  Vector3f omega_b = acc_gyro_.Get();
  Vector3f v_n = acc_gps_velocity_.Get();

  sensor_attitude_.PostControlInput(omega_b, dt);
  sensor_attitude_.PostMeasurementInput(a_b, m_b);

  sensor_velocity_.PostMeasurementInput(a_b, omega_b, v_n, GetAttitude(), dt);

  acc_accelerometer_.Reset();
  acc_magnetometer_.Reset();
  acc_gyro_.Reset();
  acc_gps_velocity_.Reset();
  tm1_ = t;
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

const Vector4f& StateManager::GetAttitude() {
  return sensor_attitude_.GetAttitude();
}

const Vector3f& StateManager::GetLinearVelocity() {
  return sensor_velocity_.GetLinearVelocity();
}

const Vector3f& StateManager::GetAngularVelocity() {
  return sensor_velocity_.GetAngularVelocity();
}
#include <iostream>

#include "falcon/state/attitude.h"
#include "falcon/state/linear_velocity.h"
#include "falcon/state/angular_velocity.h"
#include "falcon/utils.h"


using namespace falcon::state;
using namespace falcon::utils;


void execAttitudeSensor(AttitudeSensor& sensor) {
  sensor.PostControlInput(Vector3f::Random(), 0.1);
  sensor.PostMeasurementInput(Vector3f::Random(), Vector3f::Random());
}


void execLinearVelocitySensor(LinearVelocitySensor& sensor) {
  sensor.PostControlInput(Vector3f::Random(), Vector4f::Random(), 0.1);
  sensor.PostMeasurementInput(Vector3f::Random());
}

void execAngularVelocitySensor(AngularVelocitySensor sensor) {
  sensor.PostInput(Vector3f::Random(), Vector4f::Random(), 0.1);
}


int main() {
  AttitudeSensor att_sensor(AttitudeSensorParams {
    .accelerometer_gain = 0.2,
    .magnetometer_gain = 0.5,
    .covariance_accelerometer = Matrix3f::Identity() * 0.0001,
    .covariance_magnetometer = Matrix3f::Identity() * 0.0001,
    .covariance_gyro = Matrix3f::Identity() * 0.0001
  });

  std::cout << "=== Attitude sensor ===" << std::endl;
  measureFunction(10000, execAttitudeSensor, att_sensor);

  LinearVelocitySensor linear_vel_sensor(LinearVelocitySensorParams {
    .covariance_accelerometer = Matrix3f::Identity() * 0.0001,
    .covariance_gps_velocity = Matrix3f::Identity() * 0.001
  });

  std::cout << "=== Linear Velocity sensor ===" << std::endl;
  measureFunction(10000, execLinearVelocitySensor, linear_vel_sensor);

  AngularVelocitySensor angular_vel_sensor(AngularVelocitySensorParams {
    .alpha = Vector3f(0.2, 0.2, 0.2)
  });

  std::cout << "=== Angular Velocity sensor ===" << std::endl;
  measureFunction(10000, execAngularVelocitySensor, angular_vel_sensor);

  return 0;
}
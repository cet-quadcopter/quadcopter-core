#include <iostream>

#include "falcon/state/attitude.h"
#include "falcon/state/velocity.h"
#include "falcon/utils.h"


using namespace falcon::state;
using namespace falcon::utils;


void execAttitudeSensor(AttitudeSensor& sensor) {
  sensor.PostControlInput(Vector3f::Random(), 0.1);
  sensor.PostMeasurementInput(Vector3f::Random(), Vector3f::Random());
}


void execVelocitySensor(VelocitySensor& sensor) {
  sensor.PostMeasurementInput(
    Vector3f::Random(),
    Vector3f::Random(),
    Vector3f::Random(),
    Vector4f::Random(),
    0.1
  );
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

  VelocitySensor vel_sensor(VelocitySensorParams {
    .alpha_linear_velocity = Vector3f(0.2, 0.2, 0.2),
    .alpha_angular_velocity = Vector3f(0.2, 0.2, 0.2)
  });

  std::cout << "=== Velocity sensor ===" << std::endl;
  measureFunction(10000, execVelocitySensor, vel_sensor);

  return 0;
}
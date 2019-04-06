#include <iostream>
#include <cmath>
#include <math.h>

#include "ros/ros.h"
#include "spdlog/spdlog.h"
#include "Eigen/Dense"
#include "falcon/state/manager.h"
#include "falcon/math/quaternion.h"
#include "drone_constants/FLIGHT_CONTROL.h"
#include "drone_constants/POSITION_CONTROL.h"
#include "drone_std_msgs/Accelerometer.h"
#include "drone_std_msgs/Gyro.h"
#include "drone_std_msgs/Compass.h"
#include "drone_std_msgs/GPS.h"


using namespace falcon::state;
using namespace falcon::math;
using namespace drone_constants;
using namespace drone_std_msgs;
using namespace Eigen;


class FalconROS {
  private:
  StateManager state_manager_;

  public:
  FalconROS(SensorParams params)
  : state_manager_(params, ros::Time::now().toSec()) {}

  void HandleAccelerometerMessage(const Accelerometer& msg) {
    state_manager_.PostAccelerometer(Vector3f(-msg.ax, msg.ay, -msg.az));
  }

  void HandleGyroMessage(const Gyro& msg) {
    state_manager_.PostGyro(Vector3f(-msg.gx, msg.gy, -msg.gz));
  }

  void HandleMagnetometerMessage(const Compass& msg) {
    state_manager_.PostMagnetometer(Vector3f(-msg.mx, msg.my, -msg.mz));
    state_manager_.SpinOnce(ros::Time::now().toSec());
  }

  void HandleGPSMessage(const GPS& msg) {
    state_manager_.PostGPSVelocity(Vector3f(msg.vn, msg.ve, msg.vd) / 100);
    // std::cout << QuaternionToEuler123(state_manager_.GetAttitude()) * 180 / M_PI << std::endl;
    // std::cout << state_manager_.GetLinearVelocity() << std::endl;
    // std::cout << state_manager_.GetGravity() << std::endl;
    std::cout << state_manager_.GetLinearAcceleration() << std::endl;
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "falcon");
  ros::NodeHandle node;

  auto attitude_sensor_params = AttitudeSensorParams {
    .accelerometer_gain = 0.2,
    .magnetometer_gain = 0.5,
    .covariance_accelerometer = Matrix3f::Identity() * 0.0001,
    .covariance_magnetometer = Matrix3f::Identity() * 0.0001,
    .covariance_gyro = Matrix3f::Identity() * 0.0001
  };

  auto linear_velocity_sensor_params = LinearVelocitySensorParams {
    .covariance_accelerometer = Matrix3f::Identity() * 0.0001,
    .covariance_gps_velocity = Matrix3f::Identity() * 0.001
  };

  auto angular_velocity_sensor_params = AngularVelocitySensorParams {
    .alpha = Vector3f(0.2, 0.2, 0.2)
  };

  auto gravity_sensor_params = GravitySensorParams {
    .threshold = 0.2,
    .alpha = Vector3f(0, 0, 0.99)
  };

  auto sensor_params = SensorParams {
    .attitude = attitude_sensor_params,
    .linear_velocity = linear_velocity_sensor_params,
    .angular_velocity = angular_velocity_sensor_params,
    .gravity = gravity_sensor_params
  };

  auto falcon = FalconROS(sensor_params);

  auto sub_acc = node.subscribe(FLIGHT_CONTROL::TOPIC_ACCELEROMETER, 10, &FalconROS::HandleAccelerometerMessage, &falcon);
  auto sub_gyro = node.subscribe(FLIGHT_CONTROL::TOPIC_GYRO, 10, &FalconROS::HandleGyroMessage, &falcon);
  auto sub_compass = node.subscribe(FLIGHT_CONTROL::TOPIC_COMPASS, 10, &FalconROS::HandleMagnetometerMessage, &falcon);
  auto sub_gps = node.subscribe(POSITION_CONTROL::TOPIC_GPS, 10, &FalconROS::HandleGPSMessage, &falcon);

  ros::spin();
  return 0;
}
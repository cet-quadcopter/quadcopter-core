#include <iostream>
#include <cmath>
#include <math.h>

#include "ros/ros.h"
#include "spdlog/spdlog.h"
#include "Eigen/Dense"
#include "falcon/state/manager.h"
#include "falcon/control/velocity_control.h"
#include "falcon/math/quaternion.h"
#include "drone_constants/FLIGHT_CONTROL.h"
#include "drone_constants/POSITION_CONTROL.h"
#include "drone_std_msgs/Accelerometer.h"
#include "drone_std_msgs/Gyro.h"
#include "drone_std_msgs/Compass.h"
#include "drone_std_msgs/GPS.h"
#include "drone_std_msgs/InputVelocity.h"
#include "drone_std_msgs/Propeller.h"


using namespace falcon::state;
using namespace falcon::math;
using namespace falcon::control;
using namespace drone_constants;
using namespace drone_std_msgs;
using namespace Eigen;


class FalconROS {
  private:
  StateManager state_manager_;
  VelocityControl control_;
  ros::Publisher pub_control_signal_;

  public:
  FalconROS(SensorParams params, VelocityControlParams control_params, ros::Publisher pub_control_signal)
  : state_manager_(params, ros::Time::now().toSec()), 
    control_(control_params, ros::Time::now().toSec()), pub_control_signal_(pub_control_signal) {}

  void HandleAccelerometerMessage(const Accelerometer& msg) {
    state_manager_.PostAccelerometer(Vector3f(msg.ax, -msg.ay, -msg.az));
  }

  void HandleGyroMessage(const Gyro& msg) {
    state_manager_.PostGyro(Vector3f(msg.gx, -msg.gy, -msg.gz));
  }

  void HandleMagnetometerMessage(const Compass& msg) {
    state_manager_.PostMagnetometer(Vector3f(msg.mx, -msg.my, -msg.mz));
    state_manager_.SpinOnce(ros::Time::now().toSec());

    auto control = control_.GetControlSignal(state_manager_, ros::Time::now().toSec());

    std::cout << control << std::endl;

    Propeller signal;
    signal.prop1 = control(3);
    signal.prop2 = control(1);
    signal.prop3 = control(2);
    signal.prop4 = control(0);

    pub_control_signal_.publish(signal);
  }

  void HandleGPSMessage(const GPS& msg) {
    state_manager_.PostGPSVelocity(Vector3f(msg.vn, msg.ve, msg.vd) / 100);
  }

  void HandleVelocityControlMessage(const InputVelocity& msg) {
    control_.SetVelocity(Vector4f(msg.v_n, msg.v_e, msg.v_d, msg.omega_d));
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

  auto control_params = VelocityControlParams {
    .force_kp = Vector3f(1, 1, 1) * 5,
    .force_ki = Vector3f(1, 1, 1) * 0,
    .force_kd = Vector3f(1, 1, 1) * 0,
    .torque_kp = Vector3f(1, 1, 1) * 0.01,
    .torque_ki = Vector3f(1, 1, 1) * 0,
    .torque_kd = Vector3f(1, 1, 1) * 0,
    .m = 2.5,
    .d = 0.015,
    .kT = 25,
    .kTau = 50
  };

  auto pub_control_signal = node.advertise<Propeller>(FLIGHT_CONTROL::TOPIC_PROPELLER, 10);

  auto falcon = FalconROS(sensor_params, control_params, pub_control_signal);

  auto sub_acc = node.subscribe(FLIGHT_CONTROL::TOPIC_ACCELEROMETER, 10, &FalconROS::HandleAccelerometerMessage, &falcon);
  auto sub_gyro = node.subscribe(FLIGHT_CONTROL::TOPIC_GYRO, 10, &FalconROS::HandleGyroMessage, &falcon);
  auto sub_compass = node.subscribe(FLIGHT_CONTROL::TOPIC_COMPASS, 10, &FalconROS::HandleMagnetometerMessage, &falcon);
  auto sub_gps = node.subscribe(POSITION_CONTROL::TOPIC_GPS, 10, &FalconROS::HandleGPSMessage, &falcon);
  auto sub_velocity_control = node.subscribe(
    FLIGHT_CONTROL::TOPIC_VELOCITY_CONTROL, 10, &FalconROS::HandleVelocityControlMessage, &falcon);

  ros::spin();
  return 0;
}
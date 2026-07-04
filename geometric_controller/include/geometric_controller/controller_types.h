#ifndef GEOMETRIC_CONTROLLER_CONTROLLER_TYPES_H
#define GEOMETRIC_CONTROLLER_CONTROLLER_TYPES_H

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace geometric_controller {

constexpr int kErrorQuaternion = 1;
constexpr int kErrorGeometric = 2;

enum class ControllerType {
  LEGACY_GEOMETRIC = 0,
  MAIN_GEOMETRIC = 1,
  MAIN_LEE = 2,
  MAIN_JOHNSON = 3,
  MAIN_SUN_DFBC = 4,
  MAIN_SUN_DFBC_INDI = 5,
  MAIN_TAL = 6,
  MAIN_GEOMETRIC_INDI = 7,
};

struct VehicleState {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
  Eigen::Vector4d attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  double yaw = 0.0;
};

struct FlatReference {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
  Eigen::Vector3d snap = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double yaw_rate = 0.0;
  double yaw_accel = 0.0;
};

struct ControllerParams {
  int ctrl_mode = kErrorQuaternion;
  bool feedthrough_enable = false;
  bool velocity_yaw = false;
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.8);
  Eigen::Vector3d drag = Eigen::Vector3d::Zero();
  Eigen::Vector3d Kp = Eigen::Vector3d(10.0, 10.0, 20.0);
  Eigen::Vector3d Kv = Eigen::Vector3d(5.0, 5.0, 10.0);
  Eigen::Vector3d KR = Eigen::Vector3d(4.0, 4.0, 4.0);
  double max_feedback_acc = 10.0;
  double mass = 0.75;
  // The controller computes thrust as acceleration. This scale is m / total_max_thrust.
  double normalizedthrust_constant = 0.0220;
  double normalizedthrust_offset = 0.0;
};

struct ControllerCommand {
  Eigen::Vector3d body_rate = Eigen::Vector3d::Zero();
  Eigen::Vector4d attitude = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
  Eigen::Vector3d reference_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d desired_acceleration = Eigen::Vector3d::Zero();
  double thrust_accel = 0.0;
  double thrust = 0.0;
};

inline double clampUnit(double value) {
  return std::max(0.0, std::min(1.0, value));
}

inline double normalizeThrust(double thrust_accel, const ControllerParams &params) {
  return clampUnit(params.normalizedthrust_constant * thrust_accel + params.normalizedthrust_offset);
}

}  // namespace geometric_controller

#endif

#include "geometric_controller/legacy_geometric_controller.h"

#include "geometric_controller/nonlinear_attitude_control.h"
#include "geometric_controller/nonlinear_geometric_control.h"

#include <cmath>

namespace geometric_controller {

ControllerCommand LegacyGeometricController::update(const VehicleState &state, const FlatReference &reference,
                                                    const ControllerParams &params, double dt) {
  updateAttitudeController(params);

  const double yaw = params.velocity_yaw ? std::atan2(state.velocity.y(), state.velocity.x()) : reference.yaw;
  const Eigen::Vector3d desired_acc =
      params.feedthrough_enable ? reference.acceleration : controlPosition(state, reference, params, yaw);
  const Eigen::Vector4d attitude_des = acc2quaternion(desired_acc, yaw);

  Eigen::Vector4d current_attitude = state.attitude;
  attitude_controller_->Update(current_attitude, attitude_des, desired_acc, reference.jerk);

  ControllerCommand command;
  command.body_rate = attitude_controller_->getDesiredRate();
  command.attitude = attitude_des;
  command.reference_position = reference.position;
  command.desired_acceleration = desired_acc;
  command.thrust_accel = attitude_controller_->getDesiredThrust().z();
  command.thrust = normalizeThrust(command.thrust_accel, params);
  return command;
}

void LegacyGeometricController::updateAttitudeController(const ControllerParams &params) {
  constexpr double kMinAttitudeGain = 1e-6;
  const Eigen::Vector3d attctrl_tau(1.0 / std::max(params.KR.x(), kMinAttitudeGain),
                                   1.0 / std::max(params.KR.y(), kMinAttitudeGain),
                                   1.0 / std::max(params.KR.z(), kMinAttitudeGain));
  if (attitude_controller_ && attitude_error_mode_ == params.ctrl_mode &&
      (attctrl_tau_ - attctrl_tau).norm() < 1e-12) {
    return;
  }

  attitude_error_mode_ = params.ctrl_mode;
  attctrl_tau_ = attctrl_tau;
  if (attitude_error_mode_ == kErrorGeometric) {
    attitude_controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau_);
  } else {
    attitude_controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau_);
  }
}

Eigen::Vector3d LegacyGeometricController::controlPosition(const VehicleState &state,
                                                           const FlatReference &reference,
                                                           const ControllerParams &params, double yaw) const {
  const Eigen::Vector3d a_ref = reference.acceleration;
  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - params.gravity, yaw);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

  const Eigen::Vector3d pos_error = reference.position - state.position;
  const Eigen::Vector3d vel_error = reference.velocity - state.velocity;
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error, params);
  const Eigen::Vector3d a_rd =
      R_ref * params.drag.asDiagonal() * R_ref.transpose() * reference.velocity;

  return a_fb + a_ref - a_rd - params.gravity;
}

Eigen::Vector3d LegacyGeometricController::poscontroller(const Eigen::Vector3d &pos_error,
                                                         const Eigen::Vector3d &vel_error,
                                                         const ControllerParams &params) const {
  Eigen::Vector3d a_fb = params.Kp.asDiagonal() * pos_error + params.Kv.asDiagonal() * vel_error;

  if (a_fb.norm() > params.max_feedback_acc) {
    a_fb = (params.max_feedback_acc / a_fb.norm()) * a_fb;
  }
  return a_fb;
}

Eigen::Vector4d LegacyGeometricController::acc2quaternion(const Eigen::Vector3d &vector_acc, double yaw) const {
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  return rot2Quaternion(rotmat);
}

}  // namespace geometric_controller

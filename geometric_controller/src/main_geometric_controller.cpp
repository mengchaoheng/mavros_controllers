#include "geometric_controller/main_geometric_controller.h"

namespace geometric_controller {

ControllerCommand MainGeometricController::update(const VehicleState &state, const FlatReference &reference,
                                                  const ControllerParams &params, double dt) {
  // Algorithm model/control law, matching main.m controllerPDGeometric:
  //   p_dot = v
  //   v_dot = gravity + c*b3, where b3 = R*e3 and c = T/m in this ROS/PX4 sign convention
  //   aCmd = a_d + Kp*(p_d-p) + Kv*(v_d-v)
  //   c*b3d = aCmd - gravity
  //   main.m torque form adds inner angular-rate feedback and alphaD before
  //   rigid-body inversion to tau.
  //
  // ROS/MAVROS adaptation: this node can command body rates and collective
  // thrust only. The inner angular-rate feedback and alphaD are inside PX4
  // here. The command sent by this controller is
  //   body_rate_sp = OmegaD_body + KR*Log(R'*Rd).

  const Eigen::Vector3d aCmd = main_math::outerLoopAcceleration(state, reference, params);
  const Eigen::Vector3d specific_force = aCmd - params.gravity;
  const double thrust_accel = specific_force.norm();
  const Eigen::Vector4d qd = main_math::attitudeFromSpecificForce(specific_force, reference.yaw);
  const Eigen::Matrix3d R = quat2RotMatrix(state.attitude);
  const Eigen::Matrix3d Rd = quat2RotMatrix(qd);

  const Eigen::Vector3d rErr = main_math::logSO3(R.transpose() * Rd);
  const Eigen::Vector3d specific_force_dot =
      main_math::closedLoopSpecificForceDerivative(state, reference, params, thrust_accel);
  const Eigen::Vector3d omega_ref =
      main_math::referenceBodyRateFromForce(specific_force, specific_force_dot, reference.yaw, reference.yaw_rate);

  ControllerCommand command;
  command.body_rate = omega_ref + main_math::attitudeRateFeedback(rErr, params);
  command.attitude = qd;
  command.reference_position = reference.position;
  command.desired_acceleration = aCmd;
  command.thrust_accel = thrust_accel;
  command.thrust = normalizeThrust(thrust_accel, params);
  if (!command.body_rate.allFinite()) {
    command.body_rate.setZero();
  }
  return command;
}

}  // namespace geometric_controller

#include "geometric_controller/main_tal_controller.h"

namespace geometric_controller {

ControllerCommand MainTalController::update(const VehicleState &state, const FlatReference &reference,
                                            const ControllerParams &params, double dt) {
  // Algorithm model/control law, matching main.m controllerTal:
  //   Paper model: v_dot = g*i_z + tau*b_z + f_ext/m, tau < 0 for lift.
  //   Benchmark model: v_dot = gravity + T/m*b_z in this ROS/PX4 adapter.
  //   aCmd = Kp*(p_d-p) + Kv*(v_d-v) + Ka*(a_d-a_f) + a_d
  //   (tau*b_z)_c = (tau*b_z)_f + aCmd - a_f
  //   xi_e comes from the incremental attitude command.
  //   main.m torque form:
  //     OmegaDotCmd = Ktheta*xi_e + inner angular-rate feedback
  //                   + alpha_ref
  //     tauCmd = mu_f + J*(OmegaDotCmd-OmegaDot_f).
  //
  // ROS/MAVROS adaptation: PX4 owns the body-rate loop and this node has no
  // filtered acceleration/actuator/moment feedback, so Eq. (20) and Eq. (31)
  // are not commanded here. The implemented command keeps Tal's outer-loop
  // acceleration and sends a body-rate setpoint:
  //   body_rate_sp = Omega_ref + Ktheta*attitude_error.

  // Eq. (17) without acceleration feedback Ka*(a_d-a_f), because a_f is not
  // available in this PX4 body-rate interface:
  //   aCmd = Kp*(p_d-p) + Kv*(v_d-v) + a_d.
  const Eigen::Vector3d aCmd = main_math::outerLoopAcceleration(state, reference, params);
  const Eigen::Vector3d thrustAccelCmd = aCmd - params.gravity;
  const double thrust_accel = thrustAccelCmd.norm();
  const Eigen::Vector4d qd = main_math::attitudeFromSpecificForce(thrustAccelCmd, reference.yaw);
  const Eigen::Matrix3d R = quat2RotMatrix(state.attitude);
  const Eigen::Matrix3d Rd = quat2RotMatrix(qd);

  // Eq. (14)-(15) in main.m solves Tal's flatness matrix for Omega_ref and
  // alpha_ref. Since alpha_ref belongs to the unavailable moment loop, this
  // adapter uses the same differentiated thrust-vector/yaw map as the other
  // body-rate controllers to obtain Omega_ref.
  const Eigen::Vector3d thrustAccelDot =
      main_math::closedLoopSpecificForceDerivative(state, reference, params, thrust_accel);
  const Eigen::Vector3d omega_ref =
      main_math::referenceBodyRateFromForce(thrustAccelCmd, thrustAccelDot, reference.yaw, reference.yaw_rate);

  const Eigen::Vector3d attitude_error = main_math::logSO3(R.transpose() * Rd);

  ControllerCommand command;
  command.body_rate = omega_ref + main_math::attitudeRateFeedback(attitude_error, params);
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

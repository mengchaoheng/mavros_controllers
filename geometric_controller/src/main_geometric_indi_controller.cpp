#include "geometric_controller/main_geometric_indi_controller.h"

namespace geometric_controller {

ControllerCommand MainGeometricINDIController::update(const VehicleState &state, const FlatReference &reference,
                                                      const ControllerParams &params, double dt) {
  // Algorithm model/control law, matching main.m controllerGeometricINDI:
  //   Plant model:
  //     v_dot = gravity + T/m*b3, angular dynamics handled by the rate loop.
  //   Outer loop:
  //     aCmd = Kp*(p_d-p) + Kv*(v_d-v) + a_d.
  //   Incremental thrust-vector law, GINDI Eq. (55):
  //     T*b3 = T0*b30 - m*(aCmd-vDot0).
  //   Attitude/rate law in main.m torque form:
  //     OmegaDotCmd = Ktheta*Log(R'*Rd)
  //                   + inner angular-rate feedback + alphaRef
  //     tau = tau0F + J*(OmegaDotCmd-OmegaDot0F).
  //
  // ROS/MAVROS adaptation: PX4 owns the angular-rate loop and this node has
  // no actuator/moment feedback, so the incremental moment part is not
  // commanded here. The command sent by this controller is the direct
  // reference-command portion:
  //   body_rate_sp = omegaRefBody + Ktheta*Log(R'*Rd).

  // Position errors: ep = p_d-p, ev = v_d-v.
  const Eigen::Vector3d ep = reference.position - state.position;
  const Eigen::Vector3d ev = reference.velocity - state.velocity;
  // GINDI Eq. (56): aCmd = Kp*ep + Kv*ev + a_d.
  const Eigen::Vector3d aCmd =
      params.Kp.asDiagonal() * ep + params.Kv.asDiagonal() * ev + reference.acceleration;

  // Direct inversion part:
  //   thrustAxisForce = aCmd - gravity, T = ||thrustAxisForce||.
  const Eigen::Vector3d thrustAxisForce = aCmd - params.gravity;
  const double thrust_accel = thrustAxisForce.norm();
  const Eigen::Vector4d qd = main_math::attitudeFromSpecificForce(thrustAxisForce, reference.yaw);
  const Eigen::Matrix3d R = quat2RotMatrix(state.attitude);
  const Eigen::Matrix3d Rd = quat2RotMatrix(qd);

  // Sun Eq. (18)-(24) gives the reference angular velocity used by this
  // GINDI path in main.m.
  const main_math::SunReferenceRates rates =
      main_math::sunFlatnessReferenceRates(state, reference, thrust_accel);

  const Eigen::Vector3d rErr = main_math::logSO3(R.transpose() * Rd);

  ControllerCommand command;
  command.body_rate = rates.omega + main_math::attitudeRateFeedback(rErr, params);
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

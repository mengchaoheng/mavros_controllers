#include "geometric_controller/main_lee_controller.h"

namespace geometric_controller {

ControllerCommand MainLeeController::update(const VehicleState &state, const FlatReference &reference,
                                            const ControllerParams &params, double dt) {
  // Algorithm model/control law, matching main.m controllerLee:
  //   p_dot = v
  //   v_dot = gravity + T/m*R*e3 in this ROS/PX4 sign convention
  //   Lee Eq. (19)-(23), rewritten for this plant:
  //   F = Kx*(p-p_d) + Kv*(v-v_d) + m*(g*e3-a_d) in main.m's NED form.
  //   Here the equivalent thrust-axis acceleration is
  //   specific_force = a_d + Kp*(p_d-p) + Kv*(v_d-v) - gravity.
  //   b3c = F/||F||, T = F'*(R*e3)
  //   eR = 0.5*vee(Rc'*R - R'*Rc)
  //   eOmega = Omega - R'*Rc*OmegaC
  //   main.m torque form adds inner angular-rate feedback and OmegaCDot
  //   before rigid-body inversion to tau.
  //
  // ROS/MAVROS adaptation: keep Lee's attitude error definition. The inner
  // angular-rate feedback and OmegaCDot are inside PX4 here. The command
  // sent by this controller is
  //   body_rate_sp = R'*Rc*OmegaC - KR*eR.

  const Eigen::Vector3d aCmd = main_math::outerLoopAcceleration(state, reference, params);
  const Eigen::Vector3d bodyZVector = aCmd - params.gravity;
  const Eigen::Matrix3d R = quat2RotMatrix(state.attitude);
  const double thrust_accel = bodyZVector.dot(R.col(2));

  const main_math::HeadingDerivatives heading = main_math::headingAxisFromYaw(reference);
  const Eigen::Vector3d b3d = bodyZVector / bodyZVector.norm();
  const Eigen::Matrix3d Rc = main_math::attitudeFromUnitBodyZAndHeading(b3d, heading.xC);

  // Desired attitude derivatives: differentiate the same body +z_B vector
  // and yaw-heading chain, then map Rc_dot/Rc_ddot to OmegaC.
  const Eigen::Vector3d bodyZVectorDot =
      main_math::closedLoopSpecificForceDerivative(state, reference, params, std::abs(thrust_accel));
  const Eigen::Vector3d bodyZVectorDDot =
      main_math::closedLoopSpecificForceSecondDerivative(state, reference, params, std::abs(thrust_accel),
                                                         bodyZVectorDot);
  const main_math::UnitVectorDerivatives b3 =
      main_math::unitVectorDerivativesFromVector(bodyZVector, bodyZVectorDot, bodyZVectorDDot);
  const main_math::AttitudeDerivatives Rc_der =
      main_math::attitudeDerivativesFromUnitBodyZAndHeading(b3.b, b3.bDot, b3.bDDot, heading.xC, heading.xCDot,
                                                            heading.xCDDot);
  const Eigen::Vector3d OmegaC = main_math::vee(Rc.transpose() * Rc_der.RDot);

  // Lee 2010 Eq. (97): OmegaC comes from Appendix F's analytic
  // Rc_dot/Rc_ddot chain. Attitude error:
  //   eR = 0.5*vee(Rc'*R - R'*Rc).
  const Eigen::Vector3d eR = main_math::leeSO3Error(R, Rc);
  const Eigen::Vector3d omegaDInBody = R.transpose() * Rc * OmegaC;

  ControllerCommand command;
  command.body_rate = omegaDInBody - main_math::attitudeRateFeedback(eR, params);
  command.attitude = main_math::matrixToQuaternion(Rc);
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

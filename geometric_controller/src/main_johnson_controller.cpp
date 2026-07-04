#include "geometric_controller/main_johnson_controller.h"

namespace geometric_controller {

void MainJohnsonController::reset(const VehicleState &state) {
  integral_error_.setZero();
  initialized_ = false;
}

ControllerCommand MainJohnsonController::update(const VehicleState &state, const FlatReference &reference,
                                                const ControllerParams &params, double dt) {
  // Algorithm model/control law, matching main.m controllerJohnson:
  //   p_dot = v
  //   v_dot = gravity + T/m*R*e3 in this ROS/PX4 sign convention
  //   Johnson and Beard Eq. (13), (18)-(21):
  //   ea = [p-p_d; v-v_d; integral(p-p_d)]
  //   fd = -K*ea + m*(a_d-g*e3)
  //   k_d = -fd/||fd||, T = ||fd||
  //   r = Log(R'*Rid)
  //   main.m torque form adds omegaDotD_B and inner angular-rate feedback
  //   before rigid-body inversion to tau.
  //
  // ROS/MAVROS adaptation: main.m's default Johnson positionGainMode is
  // "pd" with Ki=0. omegaDotD_B and the inner angular-rate feedback are
  // inside PX4 here. The command sent by this controller is
  //   body_rate_sp = omegaD_body + Jl(r)'*KR*r.

  const double h = std::max(0.0, dt);
  const Eigen::Vector3d ep = state.position - reference.position;
  if (!initialized_ || h <= 0.0) {
    integral_error_.setZero();
    initialized_ = true;
  } else {
    integral_error_ += h * ep;
  }

  const Eigen::Vector3d aCmd = main_math::outerLoopAcceleration(state, reference, params);
  const Eigen::Vector3d bodyZVector = aCmd - params.gravity;
  const double thrust_accel = bodyZVector.norm();

  const main_math::HeadingDerivatives heading = main_math::headingAxisFromYaw(reference);
  const Eigen::Vector3d b3d = bodyZVector / bodyZVector.norm();
  const Eigen::Matrix3d Rid = main_math::attitudeFromUnitBodyZAndHeading(b3d, heading.xC);

  // Desired attitude derivatives: differentiate the same body +z_B vector
  // and yaw-heading chain, then map Rid_dot/Rid_ddot to omega_d.
  const Eigen::Vector3d bodyZVectorDot =
      main_math::closedLoopSpecificForceDerivative(state, reference, params, thrust_accel);
  const Eigen::Vector3d bodyZVectorDDot =
      main_math::closedLoopSpecificForceSecondDerivative(state, reference, params, thrust_accel, bodyZVectorDot);
  const main_math::UnitVectorDerivatives b3 =
      main_math::unitVectorDerivativesFromVector(bodyZVector, bodyZVectorDot, bodyZVectorDDot);
  const main_math::AttitudeDerivatives Rid_der =
      main_math::attitudeDerivativesFromUnitBodyZAndHeading(b3.b, b3.bDot, b3.bDDot, heading.xC, heading.xCDot,
                                                            heading.xCDDot);
  const Eigen::Vector3d omegaD = main_math::vee(Rid.transpose() * Rid_der.RDot);

  const Eigen::Matrix3d R = quat2RotMatrix(state.attitude);
  const Eigen::Matrix3d Rbd = R.transpose() * Rid;
  // SO(3) attitude error: r = Log(R'*Rid). The omega error term is handled
  // by PX4's internal body-rate loop after we publish body_rate_sp.
  const Eigen::Vector3d r = main_math::johnsonLogSO3(Rbd);
  const Eigen::Vector3d omegaDInBody = Rbd * omegaD;
  const Eigen::Matrix3d Jl = main_math::johnsonLeftJacobianSO3(r);

  ControllerCommand command;
  command.body_rate = omegaDInBody + Jl.transpose() * main_math::attitudeRateFeedback(r, params);
  command.attitude = main_math::matrixToQuaternion(Rid);
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

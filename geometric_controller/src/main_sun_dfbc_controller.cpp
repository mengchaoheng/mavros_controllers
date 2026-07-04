#include "geometric_controller/main_sun_dfbc_controller.h"

#include <cmath>

namespace geometric_controller {

ControllerCommand MainSunDFBCController::update(const VehicleState &state, const FlatReference &reference,
                                                const ControllerParams &params, double dt) {
  // Algorithm model/control law, matching main.m sunDFBCCommand:
  //   Sun DFBC uses the same plant model as NMPC but computes the command
  //   explicitly:
  //     accD = Kxi*(xi_d-xi) + Kv*(v_d-v) + xi_ddot_d,
  //     T*R_d*e3 = m*(g*e3-accD) + R*f_a^B,
  //     alphaD = Kq_red*q_red + kq_yaw*q_yaw
  //              + inner angular-rate feedback + alphaR,
  //     tauD = J*alphaD + Omega x J*Omega.
  //
  // ROS/MAVROS adaptation: this package does not command alphaD or tauD.
  // The inner angular-rate feedback and alphaR belong to the angular-rate/
  // moment loop, which is inside PX4 here. The command sent by this
  // controller is
  //   body_rate_sp = OmegaR + Kq_red*q_red + kq_yaw*q_yaw.
  // The INDI variant keeps the same body-rate interface; actuator/moment
  // INDI Eq. (32)-(35) is not available without rotor/moment feedback.

  // Sun et al. Eq. (13): desired acceleration from PD position feedback:
  //   accD = Kxi*(xi_d-xi) + Kv*(v_d-v) + xi_ddot_d.
  const Eigen::Vector3d xiErr = reference.position - state.position;
  const Eigen::Vector3d vErr = reference.velocity - state.velocity;
  const Eigen::Vector3d accD = params.Kp.asDiagonal() * xiErr + params.Kv.asDiagonal() * vErr +
                               reference.acceleration;

  // Sun et al. Eq. (14)-(17), converted from the paper's convention to the
  // current ROS/PX4 body-rate interface. Aerodynamic force is omitted here;
  // it is disabled in main.m unless par.aero.enabled is set.
  const Eigen::Vector3d thrustAxisForce = accD - params.gravity;
  const double thrust_accel = thrustAxisForce.norm();
  const Eigen::Vector4d qd = main_math::attitudeFromSpecificForce(thrustAxisForce, reference.yaw);
  const Eigen::Matrix3d Rd = quat2RotMatrix(qd);

  // Sun et al. Eq. (18)-(24): use current attitude/angular velocity and
  // collective thrust to compute reference body rates.
  const main_math::SunReferenceRates rates =
      main_math::sunFlatnessReferenceRates(state, reference, thrust_accel);

  // Sun et al. Eq. (25): for B-to-I attitude, use q_e=q^{-1}*q_d so the
  // reduced/yaw tangent vectors below are body-local errors.
  const Eigen::Vector4d q = state.attitude / state.attitude.norm();
  const Eigen::Vector4d q_inv(q(0), -q(1), -q(2), -q(3));
  Eigen::Vector4d qe = main_math::quaternionMultiply(q_inv, qd);
  qe /= qe.norm();

  // Sun et al. Eq. (26)-(27): split reduced-attitude and yaw errors.
  const double den = std::sqrt(qe(0) * qe(0) + qe(3) * qe(3));
  Eigen::Vector3d qRed = Eigen::Vector3d::Zero();
  Eigen::Vector3d qYaw = Eigen::Vector3d::Zero();
  if (den > 1e-9) {
    qRed << (qe(0) * qe(1) - qe(2) * qe(3)) / den, (qe(0) * qe(2) + qe(1) * qe(3)) / den, 0.0;
    qYaw << 0.0, 0.0, qe(3) / den;
  }

  // Sun Eq. (28): tilt-prioritized attitude control. The main.m torque
  // controller uses KqRed=diag([2*KR_x,2*KR_y,0]) and kqYaw=KR_z; here those
  // gains produce a body-rate correction instead of angular acceleration.
  const double yawSign = qe(0) < 0.0 ? -1.0 : 1.0;
  Eigen::Vector3d attitude_feedback;
  attitude_feedback << 2.0 * params.KR.x() * qRed.x(), 2.0 * params.KR.y() * qRed.y(),
      params.KR.z() * yawSign * qYaw.z();

  ControllerCommand command;
  command.body_rate = rates.omega + attitude_feedback;
  command.attitude = qd;
  command.reference_position = reference.position;
  command.desired_acceleration = accD;
  command.thrust_accel = thrust_accel;
  command.thrust = normalizeThrust(thrust_accel, params);
  if (!command.body_rate.allFinite()) {
    command.body_rate.setZero();
  }
  return command;
}

}  // namespace geometric_controller

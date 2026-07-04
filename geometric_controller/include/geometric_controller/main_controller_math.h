#ifndef GEOMETRIC_CONTROLLER_MAIN_CONTROLLER_MATH_H
#define GEOMETRIC_CONTROLLER_MAIN_CONTROLLER_MATH_H

#include "geometric_controller/common.h"
#include "geometric_controller/controller_types.h"

namespace geometric_controller {
namespace main_math {

struct HeadingDerivatives {
  Eigen::Vector3d xC = Eigen::Vector3d::UnitX();
  Eigen::Vector3d xCDot = Eigen::Vector3d::Zero();
  Eigen::Vector3d xCDDot = Eigen::Vector3d::Zero();
};

struct UnitVectorDerivatives {
  Eigen::Vector3d b = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d bDot = Eigen::Vector3d::Zero();
  Eigen::Vector3d bDDot = Eigen::Vector3d::Zero();
};

struct AttitudeDerivatives {
  Eigen::Matrix3d RDot = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d RDDot = Eigen::Matrix3d::Zero();
};

struct SunReferenceRates {
  Eigen::Vector3d omega = Eigen::Vector3d::Zero();
  Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
};

Eigen::Matrix3d hat(const Eigen::Vector3d &w);
Eigen::Vector3d vee(const Eigen::Matrix3d &S);
Eigen::Vector3d logSO3(const Eigen::Matrix3d &R);
Eigen::Vector3d johnsonLogSO3(const Eigen::Matrix3d &R);
Eigen::Matrix3d johnsonLeftJacobianSO3(const Eigen::Vector3d &phi);
Eigen::Vector3d leeSO3Error(const Eigen::Matrix3d &R, const Eigen::Matrix3d &Rd);
Eigen::Vector3d quaternionAttitudeError(const Eigen::Vector4d &q, const Eigen::Vector4d &qd);
Eigen::Vector4d quaternionMultiply(const Eigen::Vector4d &q, const Eigen::Vector4d &p);
Eigen::Vector4d matrixToQuaternion(const Eigen::Matrix3d &R);
Eigen::Vector4d attitudeFromSpecificForce(const Eigen::Vector3d &specific_force, double yaw);
Eigen::Matrix3d attitudeFromUnitBodyZAndHeading(const Eigen::Vector3d &b3d, const Eigen::Vector3d &xC);
HeadingDerivatives headingAxisFromYaw(const FlatReference &reference);
UnitVectorDerivatives unitVectorDerivativesFromVector(const Eigen::Vector3d &v, const Eigen::Vector3d &vDot,
                                                      const Eigen::Vector3d &vDDot);
AttitudeDerivatives attitudeDerivativesFromUnitBodyZAndHeading(const Eigen::Vector3d &b3d,
                                                               const Eigen::Vector3d &b3dDot,
                                                               const Eigen::Vector3d &b3dDDot,
                                                               const Eigen::Vector3d &xC,
                                                               const Eigen::Vector3d &xCDot,
                                                               const Eigen::Vector3d &xCDDot);
Eigen::Vector3d closedLoopSpecificForceDerivative(const VehicleState &state, const FlatReference &reference,
                                                  const ControllerParams &params, double thrust_accel);
Eigen::Vector3d closedLoopSpecificForceSecondDerivative(const VehicleState &state, const FlatReference &reference,
                                                        const ControllerParams &params, double thrust_accel,
                                                        const Eigen::Vector3d &specific_force_dot);
Eigen::Vector3d referenceBodyRateFromForce(const Eigen::Vector3d &specific_force,
                                           const Eigen::Vector3d &specific_force_dot, double yaw,
                                           double yaw_rate);
SunReferenceRates sunFlatnessReferenceRates(const VehicleState &state, const FlatReference &reference,
                                            double thrust_accel);
Eigen::Vector3d attitudeRateFeedback(const Eigen::Vector3d &attitude_error, const ControllerParams &params);
Eigen::Vector3d saturatedFeedbackAcceleration(const Eigen::Vector3d &feedback, double max_feedback_acc);
Eigen::Vector3d outerLoopAcceleration(const VehicleState &state, const FlatReference &reference,
                                      const ControllerParams &params);

}  // namespace main_math
}  // namespace geometric_controller

#endif

#ifndef GEOMETRIC_CONTROLLER_LEGACY_GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_LEGACY_GEOMETRIC_CONTROLLER_H

#include <memory>

#include "geometric_controller/common.h"
#include "geometric_controller/control.h"
#include "geometric_controller/controller_base.h"

namespace geometric_controller {

class LegacyGeometricController : public ControllerBase {
 public:
  LegacyGeometricController() = default;
  ~LegacyGeometricController() override = default;

  std::string name() const override { return "legacy_geometric"; }

  ControllerCommand update(const VehicleState &state, const FlatReference &reference,
                           const ControllerParams &params, double dt) override;

 private:
  std::shared_ptr<Control> attitude_controller_;
  int attitude_error_mode_{0};
  Eigen::Vector3d attctrl_tau_ = Eigen::Vector3d::Zero();

  void updateAttitudeController(const ControllerParams &params);
  Eigen::Vector3d controlPosition(const VehicleState &state, const FlatReference &reference,
                                  const ControllerParams &params, double yaw) const;
  Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error,
                                const ControllerParams &params) const;
  Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, double yaw) const;
};

}  // namespace geometric_controller

#endif

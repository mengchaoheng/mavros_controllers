#ifndef GEOMETRIC_CONTROLLER_MAIN_JOHNSON_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_MAIN_JOHNSON_CONTROLLER_H

#include "geometric_controller/controller_base.h"
#include "geometric_controller/main_controller_math.h"

namespace geometric_controller {

class MainJohnsonController : public ControllerBase {
 public:
  MainJohnsonController() = default;
  ~MainJohnsonController() override = default;

  std::string name() const override { return "main_johnson"; }

  ControllerCommand update(const VehicleState &state, const FlatReference &reference,
                           const ControllerParams &params, double dt) override;
  void reset(const VehicleState &state) override;

 private:
  Eigen::Vector3d integral_error_ = Eigen::Vector3d::Zero();
  bool initialized_{false};
};

}  // namespace geometric_controller

#endif

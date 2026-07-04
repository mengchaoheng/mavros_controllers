#ifndef GEOMETRIC_CONTROLLER_MAIN_GEOMETRIC_INDI_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_MAIN_GEOMETRIC_INDI_CONTROLLER_H

#include "geometric_controller/controller_base.h"
#include "geometric_controller/main_controller_math.h"

namespace geometric_controller {

class MainGeometricINDIController : public ControllerBase {
 public:
  MainGeometricINDIController() = default;
  ~MainGeometricINDIController() override = default;

  std::string name() const override { return "main_geometric_indi"; }

  ControllerCommand update(const VehicleState &state, const FlatReference &reference,
                           const ControllerParams &params, double dt) override;
};

}  // namespace geometric_controller

#endif

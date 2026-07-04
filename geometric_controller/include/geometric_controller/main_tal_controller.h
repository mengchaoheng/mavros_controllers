#ifndef GEOMETRIC_CONTROLLER_MAIN_TAL_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_MAIN_TAL_CONTROLLER_H

#include "geometric_controller/controller_base.h"
#include "geometric_controller/main_controller_math.h"

namespace geometric_controller {

class MainTalController : public ControllerBase {
 public:
  MainTalController() = default;
  ~MainTalController() override = default;

  std::string name() const override { return "main_tal"; }

  ControllerCommand update(const VehicleState &state, const FlatReference &reference,
                           const ControllerParams &params, double dt) override;
};

}  // namespace geometric_controller

#endif

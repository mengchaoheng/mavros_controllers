#ifndef GEOMETRIC_CONTROLLER_MAIN_LEE_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_MAIN_LEE_CONTROLLER_H

#include "geometric_controller/controller_base.h"
#include "geometric_controller/main_controller_math.h"

namespace geometric_controller {

class MainLeeController : public ControllerBase {
 public:
  MainLeeController() = default;
  ~MainLeeController() override = default;

  std::string name() const override { return "main_lee"; }

  ControllerCommand update(const VehicleState &state, const FlatReference &reference,
                           const ControllerParams &params, double dt) override;
};

}  // namespace geometric_controller

#endif

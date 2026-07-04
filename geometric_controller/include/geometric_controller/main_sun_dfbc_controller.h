#ifndef GEOMETRIC_CONTROLLER_MAIN_SUN_DFBC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_MAIN_SUN_DFBC_CONTROLLER_H

#include "geometric_controller/controller_base.h"
#include "geometric_controller/main_controller_math.h"

namespace geometric_controller {

class MainSunDFBCController : public ControllerBase {
 public:
  explicit MainSunDFBCController(bool indi_variant = false) : indi_variant_(indi_variant) {}
  ~MainSunDFBCController() override = default;

  std::string name() const override { return indi_variant_ ? "main_sun_dfbc_indi" : "main_sun_dfbc"; }

  ControllerCommand update(const VehicleState &state, const FlatReference &reference,
                           const ControllerParams &params, double dt) override;

 private:
  bool indi_variant_{false};
};

}  // namespace geometric_controller

#endif

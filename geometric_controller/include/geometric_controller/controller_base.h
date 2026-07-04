#ifndef GEOMETRIC_CONTROLLER_CONTROLLER_BASE_H
#define GEOMETRIC_CONTROLLER_CONTROLLER_BASE_H

#include <string>

#include "geometric_controller/controller_types.h"

namespace geometric_controller {

class ControllerBase {
 public:
  virtual ~ControllerBase() = default;

  virtual std::string name() const = 0;

  virtual ControllerCommand update(const VehicleState &state, const FlatReference &reference,
                                   const ControllerParams &params, double dt) = 0;

  virtual void reset(const VehicleState &state) {}
};

}  // namespace geometric_controller

#endif

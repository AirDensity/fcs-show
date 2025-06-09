#include "optimization.h"
#include "mopso.h"

using namespace config::mpc;

// Public

Optimization::Optimization(const Aircraft *aircraft, const ControlMode *control_mode) :
  aircraft(aircraft),
  control_mode(control_mode) {
}

void Optimization::compute(float out[HORIZON][config::aircraft::MAX_OUTPUTS], const bool warm_start) const {
  switch (strategy) {
    case MOPSO:
    default:
      MOPSO::compute(aircraft, control_mode, out, warm_start && optimization::USE_WARM_START);
  }
}

// Protected

bool Optimization::dominates(const float f1[optimization::MAX_OBJECTIVES],
                             const float f2[optimization::MAX_OBJECTIVES], uint8_t num_objectives) {
  bool dominates = false;
  for (uint8_t i = 0; i < num_objectives; i++) {
    if (f1[i] > f2[i] + ap_constants::EPSILON) return false;
    if (f1[i] < f2[i] - ap_constants::EPSILON) dominates = true;
  }
  return dominates;
}

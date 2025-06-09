#ifndef AUTOPILOT_MPC_CONTROL_MODE_IDLE_H
#define AUTOPILOT_MPC_CONTROL_MODE_IDLE_H

#include "control_mode.h"
#include "../../aircraft/aircraft.h"
#include "../../guidance/guidance.h"

class Idle final : protected ControlMode {
public:
  static uint8_t num_objectives() {
    return 1;
  }

  static void calculate_cost(const References *refs, const State *state, float *cost);
};

#endif //AUTOPILOT_MPC_CONTROL_MODE_IDLE_H

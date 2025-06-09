#ifndef AUTOPILOT_MPC_CONTROL_MODE_CRUISE_H
#define AUTOPILOT_MPC_CONTROL_MODE_CRUISE_H

#include "control_mode.h"
#include "../../aircraft/aircraft.h"
#include "../../guidance/guidance.h"
#include "../optimization/optimization.h"

class Cruise final : protected ControlMode {
public:
  static uint8_t num_objectives() {
    return NUM_OBJECTIVES;
  }

  static void calculate_cost(const References *refs, const State *state, float *cost);

  static void input(Guidance::Setpoint *setpoint, Input *input);

  static void init_usage(Guidance::Usage *usage);

  static void update_dependencies(Guidance *guidance, const State *state);

private:
  static constexpr uint8_t NUM_OBJECTIVES = 0;

  // Compile checks
  CHECK_GT(config::mpc::optimization::MAX_OBJECTIVES, NUM_OBJECTIVES);
};

#endif // AUTOPILOT_MPC_CONTROL_MODE_CRUISE_H
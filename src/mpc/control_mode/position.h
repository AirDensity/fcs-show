#ifndef AUTOPILOT_MPC_CONTROL_MODE_POSITION_H
#define AUTOPILOT_MPC_CONTROL_MODE_POSITION_H

#include "control_mode.h"
#include "../../aircraft/aircraft.h"
#include "../../guidance/guidance.h"
#include "../optimization/optimization.h"

class Position final : protected ControlMode {
public:
  static uint8_t num_objectives() {
    return NUM_OBJECTIVES;
  }

  // See base-class
  static void calculate_cost(const References *refs, const State *state, float *cost);

  // See base-class
  static void input(Guidance::Setpoint *setpoint, Input *input);

  // See base-class
  static void init_usage(Guidance::Usage *usage);

  // See base-class
  static void update_dependencies(Guidance *guidance, const State *state) {
    // No-op: This function does nothing
  }

private:
  static constexpr uint8_t NUM_OBJECTIVES = 5;
  static constexpr float MAX_PITCH = 90;
  static constexpr float MAX_ROLL = 90;

  // Compile checks
  CHECK_GT(config::mpc::optimization::MAX_OBJECTIVES, NUM_OBJECTIVES);

  static float constraint_violations(const State *state);
};

#endif // AUTOPILOT_MPC_CONTROL_MODE_POSITION_H
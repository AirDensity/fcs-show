#ifndef AUTOPILOT_MPC_CONTROL_MODE_LOITER_H
#define AUTOPILOT_MPC_CONTROL_MODE_LOITER_H

#include "control_mode.h"
#include "../../aircraft/aircraft.h"
#include "../../guidance/guidance.h"
#include "../optimization/optimization.h"

/**
 * Pilot Controls:
 *   The pilot can manually control pitch and roll orientation, yaw-rate and vertical velocity (instead of thrust).
 *
 * Stabilization:
 *   When the sticks are released (input value is set to 0), the vehicle will come to a stop
 *   and maintain its current position (for roll- and pitch-orientation). As well as its
 *   z-axis orientation and altitude.
 *
 * The pilot can override stabilization by commanding new inputs.
 */
class Loiter final : protected ControlMode {
public:
  static uint8_t num_objectives() {
    return NUM_OBJECTIVES;
  }

  static void calculate_cost(const References *refs, const State *state, float *cost);

  static void input(Guidance::Setpoint *setpoint, Input *input);

  static void init_usage(Guidance::Usage *usage);

  static void update_dependencies(Guidance *guidance, const State *state);

private:
  static constexpr uint8_t NUM_OBJECTIVES = 5;
  static constexpr float MAX_PITCH = 90;
  static constexpr float MAX_ROLL = 90;
  static constexpr float MAX_YAW = 200;
  static constexpr float MAX_VERTICAL_VEL = 3;

  // Compile checks
  CHECK_GT(config::mpc::optimization::MAX_OBJECTIVES, NUM_OBJECTIVES);

  static float constraint_violations(const State *state, const References *refs);
};

#endif // AUTOPILOT_MPC_CONTROL_MODE_LOITER_H
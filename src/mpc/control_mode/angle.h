#ifndef AUTOPILOT_MPC_CONTROL_MODE_ANGLE_H
#define AUTOPILOT_MPC_CONTROL_MODE_ANGLE_H

#include "control_mode.h"
#include "../../aircraft/aircraft.h"
#include "../../guidance/guidance.h"
#include "../optimization/optimization.h"

/**
 * Pilot controls roll- and pitch-orientation, yaw-rate and thrust.
 * Roll and pitch inputs are limited to +-90 degrees each.
 * Yaw-rate is limited to +-200 degrees/s.
 * Thrust is limited to +-100% and directly controls the motor output.
 * When the thrust-stick is released / centered the vehicle will bring its vertical velocity to 0 m/s and
 * maintain the altitude it was at once it reached 0 m/s.
 */
class Angle final : protected ControlMode {
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

  static float constraint_violations(const State *state, const References *refs);

  // Compile checks
  CHECK_GT(config::mpc::optimization::MAX_OBJECTIVES, NUM_OBJECTIVES);
};

#endif // AUTOPILOT_MPC_CONTROL_MODE_ANGLE_H
#ifndef AUTOPILOT_MPC_CONTROL_MODE_ALT_HOLD_H
#define AUTOPILOT_MPC_CONTROL_MODE_ALT_HOLD_H

#include "control_mode.h"
#include "../../aircraft/aircraft.h"
#include "../../guidance/guidance.h"
#include "../optimization/optimization.h"

/**
 * The pilot controls roll- and pitch-orientation, yaw-rate and thrust.
 * Controlling the thrust will result in controlling the vertical velocity of the vehicle instead of the motor output.
 * When the thrust-stick is released / centered the vehicle will bring its vertical velocity to 0 and
 * maintain its current altitude.
 *
 * Example:
 *  Current input:
 *      Thrust = +25%, Roll = 20 deg, Pitch = 30 deg, Yaw = 0 deg/s
 *  Behaviour:
 *      Vehicle is ascending while maintaining a 20-degree roll-angle, a 30-degree pitch-angle and 0 deg/s yaw-rate.
 *
 *  New input:
 *      Thrust = 0%, Roll = 0 deg, Pitch = 0 deg, Yaw = 0 deg/s
 *  Behaviour:
 *      Vehicle is reducing its vertical velocity to 0, after which it will maintain its altitude.
 *      It will maintain a 0-degree roll- and pitch-angle and a yaw-rate of 0 deg/s.
 */
class AltHold final : protected ControlMode {
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
  static constexpr float MAX_YAW = 50;
  static constexpr float MAX_VERTICAL_VEL = 3; // m/s

  // Compile checks
  CHECK_GT(config::mpc::optimization::MAX_OBJECTIVES, NUM_OBJECTIVES);

  static float constraint_violations(const State *state, const References *refs);
};

#endif // AUTOPILOT_MPC_CONTROL_MODE_ALT_HOLD_H

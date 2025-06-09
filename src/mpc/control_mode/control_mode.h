#ifndef AUTOPILOT_MPC_CONTROL_MODE_H
#define AUTOPILOT_MPC_CONTROL_MODE_H

#include "../../defines.h"
#include "../../config.h"
#include "../../aircraft/aircraft.h"
#include "../../guidance/guidance.h"

class ControlMode {
public:
  typedef enum {
    IDLE,
    POSITION,
    ANGLE,
    ACRO,
    ALT_HOLD,
    LOITER,
    CRUISE,
    WAYPOINT,
  } Mode;

  ControlMode(const Aircraft *aircraft, const float *dt, Guidance *guidance);

  /**
   * Evaluate the performance of the given motor values over the prediction horizon.
   * @param cost Result will be saved here.
   * @param motors Contains a set of motor values for each step of the prediction.
   */
  void cost_function(float *cost, const float motors[config::mpc::HORIZON][config::aircraft::MAX_OUTPUTS]) const;

  /**
   * Processes the controller input and updates the setpoint of the vehicle to reflect this input.
   * @param input Controller input.
   */
  void input(Input *input) const;

  /**
   * Returns the number of objectives that the current mode uses.
   */
  uint8_t num_objectives() const;

  /**
   * Update all parameters that are associated with the stabilization of the vehicle.
   */
  void update() const;

  void set_mode(Mode mode);

  Mode get_mode() const {
    return mode;
  }

protected:
  typedef struct {
    const Aircraft *aircraft;
    const float *dt;
    Guidance *guidance;
  } References;

  static float add_cost(const float *setpoint, const float *current, const float *first_derivative, float over);
  static float static_thrust_cost(const Aircraft *a, int8_t target_thrust, uint8_t NUM_OUTPUTS);
  static Vector3 overshoot_error(Vector3 speed, Vector3 max_deceleration, Vector3 distance);

  static void stabilize_thrust(Guidance *guidance, const State *state);
  static void stabilize_velocity_x(Guidance *guidance, const State *state);
  static void stabilize_velocity_y(Guidance *guidance, const State *state);
  static void stabilize_velocity_z(Guidance *guidance, const State *state);
  static void stabilize_orientation_x(Guidance *guidance, const State *state);
  static void stabilize_orientation_y(Guidance *guidance, const State *state);
  static void stabilize_angular_rate_x(Guidance *guidance, const State *state);
  static void stabilize_angular_rate_y(Guidance *guidance, const State *state);
  static void stabilize_angular_rate_z(Guidance *guidance, const State *state);

private:
  typedef struct {
    void (*calculate_cost)(const References *refs, const State *state, float *cost);
    void (*input)(Guidance::Setpoint *setpoint, Input *input);
    void (*init_usage)(Guidance::Usage *usage);
    void (*update_dependencies)(Guidance *guidance, const State *state);
    uint8_t (*num_objectives)();
  } ModeInterface;

  static constexpr float INPUT_THRESHOLD = 1;
  static constexpr float VELOCITY_THRESHOLD = 0.1f;
  static constexpr float RATE_THRESHOLD = 5.0f;

  References refs;
  Mode mode;
  ModeInterface mode_interface;
};

#endif // AUTOPILOT_MPC_CONTROL_MODE_H

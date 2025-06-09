#include "angle.h"

#include "../../util.h"
#include "../../aircraft/aircraft.h"

void Angle::calculate_cost(const References *refs, const State *state, float *cost) {
  const Guidance::Usage *usage = refs->guidance->get_usage_ref();
  const Guidance::Setpoint *setpoint = refs->guidance->get_setpoint_ref();

  Vector3 overshoot = overshoot_error(
      state->angular_rates, refs->aircraft->get_max_angular_acc(), setpoint->orientation - state->orientation
  );
  Vector3 v = refs->aircraft->get_max_acc(false);

  cost[0] = constraint_violations(state, refs);

  // thrust -> velocity z -> position z
  if (usage->thrust) {
    cost[1] = static_thrust_cost(refs->aircraft, setpoint->thrust, refs->aircraft->get_output_size());
  } else if (usage->velocity.z) {
    float error = fabsf(state->velocity.z) - (v.z * (*refs->dt) * config::mpc::HORIZON);
    cost[1] = add_cost(&setpoint->velocity.z, &state->velocity.z, &state->acceleration.z, error);
  } else if (usage->position.z) {
    cost[1] = add_cost(&setpoint->position.z, &state->position.z, &state->velocity.z, -1);
  }

  cost[2] = add_cost(&setpoint->orientation.x, &state->orientation.x, &state->angular_rates.x, overshoot.x);
  cost[3] = add_cost(&setpoint->orientation.y, &state->orientation.y, &state->angular_rates.y, overshoot.y);

  // yaw rate -> yaw
  if (usage->rate.z) {
    cost[4] = add_cost(&setpoint->rate.z, &state->angular_rates.z, &state->angular_acc.z, -1);
  } else if (usage->orientation.z) {
    cost[4] = add_cost(&setpoint->orientation.z, &state->orientation.z, &state->angular_rates.z, overshoot.z);
  }
}

void Angle::input(Guidance::Setpoint *setpoint, Input *input) {
  setpoint->orientation.x = util::clamp_input(input->x2, -MAX_ROLL, MAX_ROLL);
  setpoint->orientation.y = util::clamp_input(input->y2, -MAX_PITCH, MAX_PITCH);
  setpoint->rate.z = util::clamp_input(input->x1, -MAX_YAW, MAX_YAW);
  setpoint->thrust = static_cast<int8_t>(util::clamp_input(input->y1, -100, 100));
}

void Angle::init_usage(Guidance::Usage *usage) {
  usage->orientation = {true, true, false};
  usage->rate.z = true;
  usage->thrust = true;
}

void Angle::update_dependencies(Guidance *guidance, const State *state) {
  stabilize_angular_rate_z(guidance, state);
  stabilize_thrust(guidance, state);
}

// Private methods

float Angle::constraint_violations(const State *state, const References *refs) {
  float violations = 0;

  if (fabsf(state->orientation.x) > MAX_ROLL) violations += fabsf(state->orientation.x) - MAX_ROLL;
  if (fabsf(state->orientation.y) > MAX_PITCH) violations += fabsf(state->orientation.y) - MAX_PITCH;
  if (fabsf(state->angular_rates.z) > MAX_YAW) violations += fabsf(state->angular_rates.z) - MAX_YAW;

  if (refs->guidance->get_usage_ref()->position.z) {
    const float error = refs->aircraft->get_max_acc(true).z;
    if (error < 0) violations += fabsf(error);
  }

  return violations;
}
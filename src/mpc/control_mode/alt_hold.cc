#include "alt_hold.h"

#include "../../util.h"
#include "../../aircraft/aircraft.h"

void AltHold::calculate_cost(const References *refs, const State *state, float *cost) {
  const Guidance::Usage *usage = refs->guidance->get_usage_ref();
  const Guidance::Setpoint *setpoint = refs->guidance->get_setpoint_ref();

  const Vector3 o_error = overshoot_error(
    state->angular_rates, refs->aircraft->get_max_angular_acc(),
    setpoint->orientation - state->orientation
  );
  const Vector3 direction = setpoint->position - state->position;
  const Vector3 p_error = overshoot_error(
    state->velocity, refs->aircraft->get_max_acc(false), direction
  );
  const Vector3 v_decel = refs->aircraft->get_max_acc(false);

  cost[0] = constraint_violations(state, refs);
  cost[1] = add_cost(&setpoint->orientation.x, &state->orientation.x, &state->angular_rates.x, o_error.x);
  cost[2] = add_cost(&setpoint->orientation.y, &state->orientation.y, &state->angular_rates.y, o_error.y);

  // velocity z -> position z
  if (usage->velocity.z) {
    float error = fabsf(state->velocity.z) - (v_decel.z * (*refs->dt) * config::mpc::HORIZON);
    cost[3] = add_cost(&setpoint->velocity.z, &state->velocity.z, &state->acceleration.z, error);
  } else if (usage->position.z) {
    cost[3] = add_cost(&setpoint->position.z, &state->position.z, &state->velocity.z, p_error.z);
  }

  // yaw rate -> yaw
  if (usage->rate.z) {
    cost[4] = add_cost(&setpoint->rate.z, &state->angular_rates.z, &state->angular_acc.z, -1);
  } else if (usage->orientation.z) {
    cost[4] = add_cost(&setpoint->orientation.z, &state->orientation.z, &state->angular_rates.z, o_error.z);
  }
}

void AltHold::input(Guidance::Setpoint *setpoint, Input *input) {
  setpoint->velocity.z = MAX_VERTICAL_VEL * (util::clamp_input(input->y1, -100, 100) / 100.0f);
  setpoint->orientation.x = util::clamp_input(input->x2, -MAX_ROLL, MAX_ROLL);
  setpoint->orientation.y = util::clamp_input(input->y2, -MAX_PITCH, MAX_PITCH);
  setpoint->rate.z = util::clamp_input(input->x1, -MAX_YAW, MAX_YAW);
}

void AltHold::init_usage(Guidance::Usage *usage) {
  usage->orientation = {true, true, false};
  usage->rate.z = true;
  usage->velocity.z = true;
}

void AltHold::update_dependencies(Guidance *guidance, const State *state) {
  stabilize_velocity_z(guidance, state);
  stabilize_angular_rate_z(guidance, state);
}

float AltHold::constraint_violations(const State *state, const References *refs) {
  float violations = 0;

  if (fabsf(state->orientation.x) > MAX_ROLL) violations += fabsf(state->orientation.x) - MAX_ROLL;
  if (fabsf(state->orientation.y) > MAX_PITCH) violations += fabsf(state->orientation.y) - MAX_PITCH;
  if (fabsf(state->angular_rates.z) > MAX_YAW) violations += fabsf(state->angular_rates.z) - MAX_YAW;

  if (refs->guidance->get_usage_ref()->position.z) {
    const float error = refs->aircraft->get_max_acc(true).z;
    if (error < 0) violations += fabsf(error);
  }

  return violations * violations;
}

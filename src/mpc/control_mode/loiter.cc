#include "loiter.h"

#include "../../util.h"
// TODO fix
void Loiter::calculate_cost(const References *refs, const State *state, float *cost) {
  const Vector3 o_error = overshoot_error(
    state->angular_rates, refs->aircraft->get_max_angular_acc(),
    refs->guidance->get_setpoint_ref()->orientation - state->orientation
  );

  const Vector3 direction = refs->guidance->get_setpoint_ref()->position - state->position;
  const Vector3 p_error = overshoot_error(
    state->velocity, refs->aircraft->get_max_acc(false), direction
  );

  Vector3 v_decel = refs->aircraft->get_max_acc(false);
  const Vector3 r_decel = refs->aircraft->get_max_angular_acc();
  const Guidance::Setpoint *setpoint = refs->guidance->get_setpoint_ref();
  const Guidance::Usage *usage = refs->guidance->get_usage_ref();

  cost[0] = constraint_violations(state, refs);

  // velocity z -> position z
  if (usage->velocity.z) {
    const float error = fabsf(state->velocity.z) - (v_decel.z * (*refs->dt) * config::mpc::HORIZON);
    cost[1] = add_cost(&setpoint->velocity.z, &state->velocity.z, &state->acceleration.z, error);
  } else if (usage->position.z) {
    cost[1] = add_cost(&setpoint->position.z, &state->position.z, &state->velocity.z, p_error.z);
  }

  // roll -> velocity y -> position y
  if (usage->orientation.x) {
    cost[2] = add_cost(&setpoint->orientation.x, &state->orientation.x, &state->angular_rates.x, o_error.x);
  } else if (usage->velocity.y) {
    const float remaining_steps = fabsf(state->velocity.y) / (fabsf(state->acceleration.y) * (*refs->dt));
    const float error = fabsf(state->orientation.x) - (r_decel.x * (*refs->dt) * (*refs->dt) * remaining_steps);
    cost[2] = add_cost(&setpoint->velocity.y, &state->velocity.y, &state->acceleration.y, error);
  } else if (usage->position.y) {
    cost[2] = add_cost(&setpoint->position.y, &state->position.y, &state->velocity.y, p_error.y);
  }

  // pitch -> velocity x -> position x
  if (usage->orientation.y) {
    cost[3] = add_cost(&setpoint->orientation.y, &state->orientation.y, &state->angular_rates.y, o_error.y);
  } else if (usage->velocity.x) {
    const float remaining_steps = fabsf(state->velocity.x) / (fabsf(state->acceleration.x) * (*refs->dt));
    const float error = fabsf(state->orientation.y) - (r_decel.y * (*refs->dt) * (*refs->dt) * remaining_steps);
    cost[3] = add_cost(&setpoint->velocity.x, &state->velocity.x, &state->acceleration.x, error);
  } else if (usage->position.x) {
    cost[3] = add_cost(&setpoint->position.x, &state->position.x, &state->velocity.x, p_error.x);
  }

  // yaw rate -> yaw
  if (usage->rate.z) {
    cost[4] = add_cost(&setpoint->rate.z, &state->angular_rates.z, &state->angular_acc.z, -1);
  } else if (usage->orientation.z) {
    cost[4] = add_cost(&setpoint->orientation.z, &state->orientation.z, &state->angular_rates.z, o_error.z);
  }
}

void Loiter::input(Guidance::Setpoint *setpoint, Input *input) {
  setpoint->velocity.z = MAX_VERTICAL_VEL * (util::clamp_input(input->y1, -100, 100) / 100.0f);
  setpoint->orientation.x = util::clamp_input(input->x2, -MAX_ROLL, MAX_ROLL);
  setpoint->orientation.y = util::clamp_input(input->y2, -MAX_PITCH, MAX_PITCH);
  setpoint->rate.z = util::clamp_input(input->x1, -MAX_YAW, MAX_YAW);
}

void Loiter::init_usage(Guidance::Usage *usage) {
  usage->orientation = {true, true, false};
  usage->rate.z = true;
  usage->velocity.z = true;
}

void Loiter::update_dependencies(Guidance *guidance, const State *state) {
  stabilize_orientation_x(guidance, state);
  stabilize_orientation_y(guidance, state);
  stabilize_angular_rate_z(guidance, state);
  stabilize_velocity_z(guidance, state);
}

// Private methods

float Loiter::constraint_violations(const State *state, const References *refs) {
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

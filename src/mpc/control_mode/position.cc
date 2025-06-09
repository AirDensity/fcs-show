#include "position.h"

#include "../../util.h"
#include "../../aircraft/aircraft.h"

void Position::calculate_cost(const References *refs, const State *state, float *cost) {
  const Guidance::Usage *usage = refs->guidance->get_usage_ref();
  const Guidance::Setpoint *setpoint = refs->guidance->get_setpoint_ref();

  const Vector3 direction = setpoint->position - state->position;
  const Vector3 p_error = overshoot_error(
    state->velocity, refs->aircraft->get_max_acc(false), direction
  );

  cost[0] = constraint_violations(state);

  cost[1] = fabsf(state->position.x - setpoint->position.x);
  if (p_error.x > ap_constants::EPSILON) cost[1] += fabsf(state->velocity.x);

  cost[2] = fabsf(state->position.y - setpoint->position.y);
  if (p_error.y > ap_constants::EPSILON) cost[2] += fabsf(state->velocity.y);

  cost[3] = fabsf(state->position.z - setpoint->position.z);
  if (p_error.z > ap_constants::EPSILON) cost[3] += fabsf(state->velocity.z);

  cost[4] = fabsf(state->orientation.z - setpoint->orientation.z);
}

void Position::input(Guidance::Setpoint *setpoint, Input *input) {
  setpoint->position = {
    static_cast<float>(input->x2),
    static_cast<float>(input->y2),
    static_cast<float>(input->y1),
  };
  setpoint->orientation.z = input->x1;
}

void Position::init_usage(Guidance::Usage *usage) {
  usage->position = {true, true, true};
  usage->orientation.z = true;
}

// Private methods

float Position::constraint_violations(const State *state) {
  float violations = 0;

  if (fabsf(state->orientation.x) > MAX_ROLL) violations += fabsf(state->orientation.x) - MAX_ROLL;
  if (fabsf(state->orientation.y) > MAX_PITCH) violations += fabsf(state->orientation.y) - MAX_ROLL;

  return violations;
}
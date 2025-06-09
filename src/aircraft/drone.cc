#include "drone.h"

bool Drone::mode_valid(uint8_t m) const {
  return true;
}

void Drone::simulate(const float dt, State *state) const {
  float relative[NUM_OUTPUTS];
  float relative_sum = 0;
  for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
    relative[i] = static_cast<float>(state->output[i]) / static_cast<float>(MAX_OUTPUT[i]);
    relative_sum += relative[i];
  }
  const float thrust = THRUST_COEFF * relative_sum * INV_MASS;

  state->acceleration = util::body_to_inertial({0, 0, thrust}, state->orientation);
  state->acceleration.z -= ap_constants::GRAVITY;

  // Don't simulate the aircraft if it's still on the ground and not getting airborne
  if (state->acceleration.z > 0) state->airborne = true;
  if (!state->airborne) return;

  state->position += (state->velocity * dt) + (state->acceleration * dt * dt * 0.5f);
  state->velocity += (state->acceleration * dt);

  const Vector3 torque = {
      THRUST_COEFF_X * (relative[0] + relative[2] - relative[1] - relative[3]),
      THRUST_COEFF_Y * (relative[2] + relative[3] - relative[0] - relative[1]),
      KT_opt * (-relative[0] + relative[1] + relative[2] - relative[3]),
  };

  state->angular_acc = torque * INV_INERTIA * ap_constants::RAD_TO_DEG;
  state->orientation += (state->angular_rates * dt) + (state->angular_acc * dt * dt * 0.5f);
  state->angular_rates += (state->angular_acc * dt);

  limit_to_180(state->orientation);
}

Vector3 Drone::get_max_acc(const bool specific) const {
  if (specific) {
    Vector3 out = util::body_to_inertial({0, 0, MAX_THRUST}, state.orientation);
    out.z -= ap_constants::GRAVITY;
    return out;
  }

  if (state.velocity.z <= 0)
    return {MAX_THRUST * 0.8f, MAX_THRUST * 0.8f, MAX_THRUST * 0.8f};

  return {MAX_THRUST * 0.8f, MAX_THRUST * 0.8f, ap_constants::GRAVITY * 0.8f};
}

Vector3 Drone::get_max_angular_acc() const {
  return {9000, 9000, 50};
}
#include "acro.h"

#include "../../util.h"

void Acro::calculate_cost(const References *refs, const State *state, float *cost) {
  const Vector3 MAX_DECELERATION = refs->aircraft->get_max_angular_acc();
  const Guidance::Setpoint *setpoint = refs->guidance->get_setpoint_ref();

  cost[0] = static_thrust_cost(refs->aircraft, setpoint->thrust, refs->aircraft->get_output_size());

  cost[1] = fabsf(setpoint->rate.x - state->angular_rates.x);
  const float error_x = fabsf(state->angular_rates.x) - (MAX_DECELERATION.x * (*refs->dt) * config::mpc::HORIZON);
  if (error_x > ap_constants::EPSILON) cost[1] += error_x;

  cost[2] = fabsf(setpoint->rate.y - state->angular_rates.y);
  const float error_y = fabsf(state->angular_rates.y) - (MAX_DECELERATION.y * (*refs->dt) * config::mpc::HORIZON);
  if (error_y > ap_constants::EPSILON) cost[2] += error_y;

  cost[3] = fabsf(setpoint->rate.z - state->angular_rates.z);
  const float error_z = fabsf(state->angular_rates.z) - (MAX_DECELERATION.z * (*refs->dt) * config::mpc::HORIZON);
  if (error_z > ap_constants::EPSILON) cost[3] += error_z;
}

void Acro::input(Guidance::Setpoint *setpoint, Input *input) {
  setpoint->rate = {
    static_cast<float>(input->x2),
    static_cast<float>(input->y2),
    static_cast<float>(input->x1),
  };
  setpoint->thrust = input->y1;
}

void Acro::init_usage(Guidance::Usage *usage) {
  usage->rate = {true, true, true};
  usage->thrust = true;
}

void Acro::update_dependencies(Guidance *guidance, const State *state) {
  //stabilize_angular_rate_x(guidance, state);
  //stabilize_angular_rate_y(guidance, state);
  //stabilize_angular_rate_z(guidance, state);
}
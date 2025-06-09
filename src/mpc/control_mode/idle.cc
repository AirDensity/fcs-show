#include "idle.h"

void Idle::calculate_cost(const References *refs, const State *state, float *cost) {
  const uint8_t OUTPUTS = refs->aircraft->get_output_size();

  for (uint8_t i = 0; i < OUTPUTS; i++) {
    const Motor min = refs->aircraft->get_min_output(i);
    if (state->output[i] > min) cost[0] += state->output[i] - min;
  }
}


#include "mpc.h"
#include "../util.h"

MPC::MPC(const Aircraft *aircraft, const float *dt, Guidance *guidance) :
    dt(dt),
    aircraft(aircraft),
    guidance(guidance),
    control_mode(aircraft, dt, guidance),
    optimization(aircraft, &control_mode) {
}

void MPC::input(Input *input) {
  control_mode.input(input);

  if (guidance->get_setpoint_deviation()) {
    warm_start_optimization = false;
  }
}

void MPC::set_control_mode(const ControlMode::Mode mode) {
  if (mode != control_mode.get_mode()) warm_start_optimization = false; // Reset warm-starts
  control_mode.set_mode(mode);
  elapsed_sampling_time = sampling_time; // Reset compute
}

void MPC::compute(Motor out[config::aircraft::MAX_OUTPUTS]) {
  control_mode.update();
  elapsed_sampling_time += *dt;

  if (elapsed_sampling_time >= static_cast<float>(sampling_time) * 0.001f) {
    // NOTE: currently assuming that dt doesn't change significantly over time
    optimization.compute(output, warm_start_optimization);
    util::float_to_motor(out, output[0], aircraft->get_output_size());
    elapsed_sampling_time = 0;
    iterations_skipped = 0;
  } else if (iterations_skipped < config::mpc::HORIZON - 1) { // Select output from last iteration if possible
    iterations_skipped++;
    util::float_to_motor(out, output[iterations_skipped], aircraft->get_output_size());
  }

  warm_start_optimization = true;

  if (iterations_skipped == config::mpc::HORIZON - 1) { // No more valid values in 'output'. Reset for next iteration
    elapsed_sampling_time = sampling_time;
    iterations_skipped = 0;
    warm_start_optimization = false;
  }
}

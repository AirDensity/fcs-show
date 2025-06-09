#ifndef AUTOPILOT_MPC_H
#define AUTOPILOT_MPC_H

#include "../defines.h"
#include "control_mode/control_mode.h"
#include "optimization/optimization.h"

class MPC {
public:
  MPC(const Aircraft *aircraft, const float *dt, Guidance *guidance);

  void compute(Motor out[config::aircraft::MAX_OUTPUTS]);

  void input(Input *input);

  void set_control_mode(ControlMode::Mode mode);

  ControlMode::Mode get_control_mode() const {
    return control_mode.get_mode();
  }

  void set_sampling_time(const uint16_t ms) {
    sampling_time = ms;
  }

  uint16_t get_sampling_time() const {
    return sampling_time;
  }

  void set_optimization_strategy(Optimization::Strategy s) {
    optimization.set_strategy(s);
  }

  Optimization::Strategy get_optimization_strategy() const {
    return optimization.get_strategy();
  }

private:
  const float *dt;
  const Aircraft *aircraft;
  const Guidance *guidance;

  ControlMode control_mode;
  Optimization optimization;

  float output[config::mpc::HORIZON][config::aircraft::MAX_OUTPUTS] = {0};

  uint16_t sampling_time = 0;         // in seconds
  float elapsed_sampling_time = 0.0f; // in milliseconds
  uint8_t iterations_skipped = 0;

  bool warm_start_optimization = false;
};

#endif // AUTOPILOT_MPC_H
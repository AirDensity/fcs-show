#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "defines.h"
#include "mpc/mpc.h"
#include "guidance/guidance.h"
#include "navigation/navigation.h"

class Autopilot {
public:
  Autopilot(Aircraft *aircraft, const float *dt, const Navigation::SensorData *sensor) :
      guidance(),
      navigation(aircraft, dt, sensor),
      control(aircraft, dt, &guidance) {}

  void compute(Motor out[config::aircraft::MAX_OUTPUTS]) {
    // TODO Navigation update
    // TODO Guidance update
    control.compute(out);
  }

  //***************************************************************************
  // Guidance
  //***************************************************************************

  void set_input_filter(const bool set) {
    guidance.set_input_filter(set);
  }

  bool get_input_filter() const {
    return guidance.get_input_filter();
  }

  //***************************************************************************
  // Navigation
  //***************************************************************************

  //***************************************************************************
  // Control
  //***************************************************************************

  void input(Input input) {
    control.input(&input);
  }

  void set_optimization_strategy(const Optimization::Strategy s) {
    control.set_optimization_strategy(s);
  }

  Optimization::Strategy get_optimization_strategy() const {
    return control.get_optimization_strategy();
  }

  void set_control_mode(const ControlMode::Mode mode) {
    control.set_control_mode(mode);
  }

  ControlMode::Mode get_control_mode() const {
    return control.get_control_mode();
  }

  void set_sampling_time(const uint16_t s) {
    control.set_sampling_time(s);
  }

  uint16_t get_sampling_time() const {
    return control.get_sampling_time();
  }

private:
  Guidance guidance;
  Navigation navigation;
  MPC control;
};

#endif // AUTOPILOT_H
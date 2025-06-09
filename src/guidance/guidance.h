#ifndef AUTOPILOT_GUIDANCE_H
#define AUTOPILOT_GUIDANCE_H

#include "../defines.h"

class Guidance {
public:
  typedef struct Setpoint {
    Vector3 position = {0, 0, 0};
    Vector3 velocity = {0, 0, 0};
    Vector3 orientation = {0, 0, 0};
    Vector3 rate = {0, 0, 0};
    int8_t thrust = 0;
  } Setpoint;

  typedef struct Usage {
    Bool position = {false, false, false};
    Bool velocity = {false, false, false};
    Bool orientation = {false, false, false};
    Bool rate = {false, false, false};
    bool thrust = false;
  } Usage;

  Guidance() = default;

  void set_input_filter(const bool set) {
    input_filter = set;
  }

  bool get_input_filter() const {
    return input_filter;
  }

  void set_usage(const Usage &usage) {
    this->usage = usage;
  }

  const Usage *get_usage_ref() const {
    return &usage;
  }

  Usage get_usage() const {
    return usage;
  }

  void set_setpoint(const Setpoint &setpoint, const float alpha = 0.1f) {
    setpoint_deviation(setpoint, config::mpc::optimization::MAX_DEVIATION);
    if (input_filter && !deviation) filter_input(setpoint, alpha);
    else this->setpoint = setpoint;
  }

  const Setpoint *get_setpoint_ref() const {
    return &setpoint;
  }

  Setpoint get_setpoint() const {
    return setpoint;
  }

  bool get_setpoint_deviation() const {
    return deviation;
  }

private:
  Setpoint setpoint;
  Usage usage;

  bool input_filter = false;
  bool deviation = false;

  void filter_input(const Setpoint &setpoint, float alpha);

  void setpoint_deviation(const Setpoint &setpoint, uint8_t threshold);

  friend class ControlMode;
};

#endif // AUTOPILOT_GUIDANCE_H

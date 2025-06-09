#include "control_mode.h"

#include "../../util.h"
#include "position.h"
#include "angle.h"
#include "acro.h"
#include "alt_hold.h"
#include "cruise.h"
#include "loiter.h"
#include "waypoint.h"
#include "idle.h"

// Public

ControlMode::ControlMode(const Aircraft *aircraft, const float *dt, Guidance *guidance) : refs(aircraft, dt, guidance) {
  set_mode(IDLE);
}

void ControlMode::cost_function(float *cost,
                                const float motors[config::mpc::HORIZON][config::aircraft::MAX_OUTPUTS]) const {
  if (mode_interface.calculate_cost == nullptr || mode_interface.num_objectives == nullptr) return;

  const uint8_t MAX_OUTPUTS = refs.aircraft->get_output_size();
  const uint8_t NUM_OBJECTIVES = mode_interface.num_objectives();

  State sim_state = refs.aircraft->state; // Create modifiable state

  float initial[config::mpc::optimization::MAX_OBJECTIVES];
  float step[config::mpc::optimization::MAX_OBJECTIVES];
  mode_interface.calculate_cost(&refs, &sim_state, initial);

  // Simulate the vehicle and evaluate the performance
  for (uint8_t i = 0; i < config::mpc::HORIZON; i++) {
    util::float_to_motor(sim_state.output, motors[i], MAX_OUTPUTS);
    refs.aircraft->simulate(*refs.dt, &sim_state);

    mode_interface.calculate_cost(&refs, &sim_state, step);

    // Sum up costs and add a penalty if an objective gets worse over time
    for (uint8_t j = 0; j < NUM_OBJECTIVES; j++) {
      if (step[j] > initial[j] + ap_constants::EPSILON) cost[j] += step[j] - initial[j];
      cost[j] += step[j];
      initial[j] = step[j];
    }
  }
}

void ControlMode::input(Input *input) const {
  if (mode_interface.input == nullptr) return;

  // Avoids overwriting values that are still required for stabilization
  Guidance::Setpoint setpoint = refs.guidance->get_setpoint();

  mode_interface.input(&setpoint, input);
  refs.guidance->set_setpoint(setpoint);
}

uint8_t ControlMode::num_objectives() const {
  if (mode_interface.num_objectives == nullptr) return 0;
  return mode_interface.num_objectives();
}

void ControlMode::update() const {
  if (mode_interface.update_dependencies == nullptr) return;
  mode_interface.update_dependencies(refs.guidance, &refs.aircraft->state);
}

void ControlMode::set_mode(const Mode mode) {
  if (!refs.aircraft->mode_valid(mode)) return;

  this->mode = mode;

  switch (this->mode) {
    case POSITION:
      mode_interface = {
        Position::calculate_cost, Position::input, Position::init_usage, Position::update_dependencies,
        Position::num_objectives
      };
      break;
    case ANGLE:
      mode_interface = {
        Angle::calculate_cost, Angle::input, Angle::init_usage, Angle::update_dependencies, Angle::num_objectives
      };
      break;
    case ACRO:
      mode_interface = {
        Acro::calculate_cost, Acro::input, Acro::init_usage, Acro::update_dependencies, Acro::num_objectives
      };
      break;
    case ALT_HOLD:
      mode_interface = {
        AltHold::calculate_cost, AltHold::input, AltHold::init_usage, AltHold::update_dependencies,
        AltHold::num_objectives
      };
      break;
    case LOITER:
      mode_interface = {
        Loiter::calculate_cost, Loiter::input, Loiter::init_usage, Loiter::update_dependencies, Loiter::num_objectives
      };
      break;
    case CRUISE:
      mode_interface = {
        Cruise::calculate_cost, Cruise::input, Cruise::init_usage, Cruise::update_dependencies, Cruise::num_objectives
      };
      break;
    case WAYPOINT:
      mode_interface = {
        Waypoint::calculate_cost, nullptr, Waypoint::init_usage, Waypoint::update_dependencies, Waypoint::num_objectives
      };
      break;
    case IDLE:
    default:
      mode_interface = {Idle::calculate_cost, nullptr, nullptr, nullptr, Idle::num_objectives};
  }

  // Reset setpoint
  constexpr Guidance::Setpoint s;
  refs.guidance->set_setpoint(s);

  if (mode_interface.init_usage != nullptr) {
    Guidance::Usage usage;
    mode_interface.init_usage(&usage);
    refs.guidance->set_usage(usage);
  }
  if (mode_interface.update_dependencies != nullptr) {
    mode_interface.update_dependencies(refs.guidance, &refs.aircraft->state);
  }
}

// Protected

float ControlMode::add_cost(const float *setpoint, const float *current, const float *first_derivative, float over) {
  float out = 0;
  const float diff = *setpoint - *current;
  out += fabsf(diff);

  // Penalise moving in the wrong direction
  if (diff * (*first_derivative) < -ap_constants::EPSILON) {
    out += fabsf(*first_derivative);
  }

  if (over > ap_constants::EPSILON) out += fabsf(diff) + 0.01f * fabsf(over);

  return out;
}

float ControlMode::static_thrust_cost(const Aircraft *a, const int8_t target_thrust, const uint8_t NUM_OUTPUTS) {
  float thrust = 0;

  for (uint8_t j = 0; j < NUM_OUTPUTS; j++) {
    thrust += static_cast<float>(a->state.output[j]) / static_cast<float>(a->get_max_output(j));
  }
  thrust /= static_cast<float>(NUM_OUTPUTS);

  return fabsf(thrust * 100 - static_cast<float>(target_thrust));
}

Vector3 ControlMode::overshoot_error(Vector3 speed, Vector3 max_deceleration, Vector3 distance) {
  distance = {
    fabsf(distance.x),
    fabsf(distance.y),
    fabsf(distance.z),
  };
  Vector3 speed_diff = (speed * speed) - (max_deceleration * distance * 0.25f * 2);

  if (speed_diff.x < 0) speed_diff.x = 0;
  if (speed_diff.y < 0) speed_diff.y = 0;
  if (speed_diff.z < 0) speed_diff.z = 0;

  return {
    speed_diff.x,
    speed_diff.y,
    speed_diff.z
  };
}

void ControlMode::stabilize_thrust(Guidance *guidance, const State *state) {
  if (guidance == nullptr) return;

  if (guidance->setpoint.thrust == 0) {
    if (guidance->usage.thrust) {
      guidance->usage.thrust = false;
      guidance->usage.velocity.z = true;
      guidance->setpoint.velocity.z = 0;
    }
    if (guidance->usage.velocity.z && fabsf(state->velocity.z) < VELOCITY_THRESHOLD) {
      guidance->usage.velocity.z = false;
      guidance->usage.position.z = true;
      guidance->setpoint.position.z = state->position.z;
    }
  } else {
    guidance->usage.thrust = true;
    guidance->usage.velocity.z = false;
    guidance->usage.position.z = false;
  }
}

void ControlMode::stabilize_velocity_x(Guidance *guidance, const State *state) {
  if (guidance == nullptr) return;

  if (fabsf(guidance->setpoint.velocity.x) < INPUT_THRESHOLD) {
    if (guidance->usage.velocity.x && fabsf(state->velocity.x) < VELOCITY_THRESHOLD) {
      guidance->usage.velocity.x = false;
      guidance->usage.position.x = true;
      guidance->setpoint.position.x = state->position.x;
    }
  } else {
    guidance->usage.velocity.x = true;
    guidance->usage.position.x = false;
  }
}

void ControlMode::stabilize_velocity_y(Guidance *guidance, const State *state) {
  if (guidance == nullptr) return;

  if (fabsf(guidance->setpoint.velocity.y) < INPUT_THRESHOLD) {
    if (guidance->usage.velocity.y && fabsf(state->velocity.y) < VELOCITY_THRESHOLD) {
      guidance->usage.velocity.y = false;
      guidance->usage.position.y = true;
      guidance->setpoint.position.y = state->position.y;
    }
  } else {
    guidance->usage.velocity.y = true;
    guidance->usage.position.y = false;
  }
}

void ControlMode::stabilize_velocity_z(Guidance *guidance, const State *state) {
  if (guidance == nullptr) return;

  if (fabsf(guidance->setpoint.velocity.z) < INPUT_THRESHOLD) {
    if (guidance->usage.velocity.z && fabsf(state->velocity.z) < VELOCITY_THRESHOLD) {
      guidance->usage.velocity.z = false;
      guidance->usage.position.z = true;
      guidance->setpoint.position.z = state->position.z;
    }
  } else {
    guidance->usage.velocity.z = true;
    guidance->usage.position.z = false;
  }
}

void ControlMode::stabilize_orientation_x(Guidance *guidance, const State *state) {
  if (guidance == nullptr) return;

  if (fabsf(guidance->setpoint.orientation.x) < INPUT_THRESHOLD) {
    if (guidance->usage.orientation.x) {
      guidance->usage.orientation.x = false;
      guidance->usage.velocity.y = true;
      guidance->setpoint.velocity.y = 0;
    }
    if (guidance->usage.velocity.y && fabsf(state->velocity.y) < VELOCITY_THRESHOLD) {
      guidance->usage.velocity.y = false;
      guidance->usage.position.y = true;
      guidance->setpoint.position.y = state->position.y;
    }
  } else {
    guidance->usage.orientation.x = true;
    guidance->usage.position.y = false;
    guidance->usage.velocity.y = false;
  }
}

void ControlMode::stabilize_orientation_y(Guidance *guidance, const State *state) {
  if (guidance == nullptr) return;

  if (fabsf(guidance->setpoint.orientation.y) < INPUT_THRESHOLD) {
    if (guidance->usage.orientation.y) {
      guidance->usage.orientation.y = false;
      guidance->usage.velocity.x = true;
      guidance->setpoint.velocity.x = 0;
    }
    if (guidance->usage.velocity.x && fabsf(state->velocity.x) < VELOCITY_THRESHOLD) {
      guidance->usage.velocity.x = false;
      guidance->usage.position.x = true;
      guidance->setpoint.position.x = state->position.x;
    }
  } else {
    guidance->usage.orientation.y = true;
    guidance->usage.position.x = false;
    guidance->usage.velocity.x = false;
  }
}

void ControlMode::stabilize_angular_rate_x(Guidance *guidance, const State *state) {
  if (guidance == nullptr) return;

  if (fabsf(guidance->setpoint.rate.x) < INPUT_THRESHOLD) {
    if (guidance->usage.rate.x && fabsf(state->angular_rates.x) < RATE_THRESHOLD) {
      guidance->usage.rate.x = false;
      guidance->usage.orientation.x = true;
      guidance->setpoint.orientation.x = state->orientation.x;
    }
  } else {
    guidance->usage.rate.x = true;
    guidance->usage.orientation.x = false;
  }
}

void ControlMode::stabilize_angular_rate_y(Guidance *guidance, const State *state) {
  if (guidance == nullptr) return;

  if (fabsf(guidance->setpoint.rate.y) < INPUT_THRESHOLD) {
    if (guidance->usage.rate.y && fabsf(state->angular_rates.y) < RATE_THRESHOLD) {
      guidance->usage.rate.y = false;
      guidance->usage.orientation.y = true;
      guidance->setpoint.orientation.y = state->orientation.y;
    }
  } else {
    guidance->usage.rate.y = true;
    guidance->usage.orientation.y = false;
  }
}

void ControlMode::stabilize_angular_rate_z(Guidance *guidance, const State *state) {
  if (guidance == nullptr) return;

  if (fabsf(guidance->setpoint.rate.z) < INPUT_THRESHOLD) {
    if (guidance->usage.rate.z && fabsf(state->angular_rates.z) < RATE_THRESHOLD) {
      guidance->usage.rate.z = false;
      guidance->usage.orientation.z = true;
      guidance->setpoint.orientation.z = state->orientation.z;
    }
  } else {
    guidance->usage.rate.z = true;
    guidance->usage.orientation.z = false;
  }
}

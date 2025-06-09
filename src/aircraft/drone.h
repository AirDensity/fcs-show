#ifndef AUTOPILOT_AIRCRAFT_DRONE_H
#define AUTOPILOT_AIRCRAFT_DRONE_H

#include "aircraft.h"
#include "../util.h"

/**
 * Quadcopter
 */
class Drone final : public Aircraft {
public:
  // See base-class
  void simulate(float dt, State *state) const override;

  // See base-class
  bool mode_valid(uint8_t m) const override;

  // See base-class
  uint8_t get_output_size() const override {
    return NUM_OUTPUTS;
  }

  // See base-class
  Motor get_min_output(uint8_t index) const override {
    return MIN_OUTPUT[static_cast<uint8_t >(util::clamp_input(index, 0, NUM_OUTPUTS - 1))];
  }

  // See base-class
  Motor get_max_output(uint8_t index) const override {
    return MAX_OUTPUT[static_cast<uint8_t >(util::clamp_input(index, 0, NUM_OUTPUTS - 1))];
  }
  // See base-class
  Vector3 get_max_acc(bool specific) const override;

  // See base-class
  Vector3 get_max_angular_acc() const override;

private:
  //********************************************************************************************************
  // Config Start
  //********************************************************************************************************

  constexpr static Motor MAX_OUTPUT[4] = {1000, 1000, 1000, 1000};
  constexpr static Motor MIN_OUTPUT[4] = {10, 10, 10, 10};

  constexpr static float MASS = 0.8f;              // kg
  constexpr static float DISTANCE_X = 0.1f;        // x distance from a motor to the center of mass in meters
  constexpr static float DISTANCE_Y = 0.1f;        // y distance from a motor to the center of mass in meters
  constexpr static float INERTIA_X = 0.00295f;     // kg * m²
  constexpr static float INERTIA_Y = 0.00295f;     // kg * m²
  constexpr static float INERTIA_Z = 0.0059f;      // kg * m²

  constexpr static float MAX_THRUST_P_M = 650.0f;  // maximum thrust of a motor in grams
  constexpr static float KV_RATING = 5000.0f;      // KV-Rating of the motor
  constexpr static float MAX_AMPS = 4.4f;          // Ampere consumption of a motor at 100% input

  //********************************************************************************************************
  // Config End
  //********************************************************************************************************

  // Constants. DO NOT CHANGE!
  constexpr static uint8_t NUM_OUTPUTS = 4;
  constexpr static float MAX_THRUST = MAX_THRUST_P_M * (9.81f / 1000) * NUM_OUTPUTS / MASS; // in m/s²
  constexpr static float THRUST_COEFF = MAX_THRUST_P_M * (9.81f / 1000);
  constexpr static float THRUST_COEFF_X = THRUST_COEFF * DISTANCE_X;
  constexpr static float THRUST_COEFF_Y = THRUST_COEFF * DISTANCE_Y;
  constexpr static float KT = 60.0f / (2.0f * ap_constants::PI * KV_RATING);
  constexpr static float KT_opt = KT * MAX_AMPS;

  constexpr static float INV_MASS = 1.0f / MASS;
  constexpr static Vector3 INV_INERTIA = {
      1.0f / INERTIA_X,
      1.0f / INERTIA_Y,
      1.0f / INERTIA_Z,
  };

  // Compile checks
  CHECK_GT(config::aircraft::MAX_OUTPUTS, NUM_OUTPUTS);
};

#endif // AUTOPILOT_AIRCRAFT_DRONE_H

#ifndef AUTOPILOT_AIRCRAFT_H
#define AUTOPILOT_AIRCRAFT_H

#include "../defines.h"
#include "../config.h"

class Aircraft {
public:
  virtual ~Aircraft() = default;

  /**
   * Simulate the vehicle for one timestep
   * @param dt Timestep
   * @param state Current state of the vehicle. Will be updated during simulation.
   */
  virtual void simulate(float dt, State *state) const = 0;

  /**
   * Returns true if a given Controlmode is supported by the vehicle. Otherwise, false.
   * @param m Mode that will be checked.
   */
  virtual bool mode_valid(uint8_t m) const = 0;

  /**
   * Returns the number of elements in the output array (Number of control variables).
   */
  virtual uint8_t get_output_size() const = 0;

  /**
   * Returns the min. possible value of an output variable.
   * @param index Position of the desired variable in the output array.
   */
  virtual Motor get_min_output(uint8_t index) const = 0;

  /**
   * Returns the max. possible value of an output variable.
   * @param index Position of the desired variable in the output array.
   */
  virtual Motor get_max_output(uint8_t index) const = 0;

  virtual Vector3 get_max_acc(bool specific) const = 0;

  virtual Vector3 get_max_angular_acc() const = 0;

  State state;

protected:
  /**
   * Limits a vector to +360 degrees
   * @param v Input vector
   */
  static void limit_to_360(Vector3 &v);

  /**
   * Limits a vector to +-180 degrees
   * @param v Input vector
   */
  static void limit_to_180(Vector3 &v);
};

#endif // AUTOPILOT_AIRCRAFT_H

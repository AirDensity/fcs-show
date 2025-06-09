#ifndef AUTOPILOT_UTIL_H
#define AUTOPILOT_UTIL_H

#include "defines.h"

namespace util {
  /**
   * Converts any float-array to a Motor-array.
   *
   * @param dest The Motor-array.
   * @param src The float-array.
   * @param size Number of values that will be copied.
   */
  void float_to_motor(Motor dest[], const float src[], uint8_t size);

  /**
   * Limit a value to a range from min to max.
   *
   * @param value Value that will be limited.
   * @param min Min. value.
   * @param max Max. value.
   */
  void clamp(float &value, float min, float max);
  float clamp_input(float value, float min, float max);

  /**
   * Returns a uniformly distributed random number in the range [min, max]
   *
   * @param min Smallest potential return value.
   * @param max Largest potential return value.
   * @return Uniformly distributed random number.
   */
  float uniform_random_number(float min, float max);

  /**
   * Returns a normal distributed random number.
   *
   * @param mu
   * @param sigma
   * @return Normal distributed random number.
   */
  float normal_distribution(float mu, float sigma);

  /**
   * Converts the input from the body-frame to the inertial-frame.
   *
   * @param input Measurement in the body-frame
   * @param orientation Orientation of the aircraft.
   * @return The resulting vector.
   */
  Vector3 body_to_inertial(Vector3 input, Vector3 orientation);
} // util

#endif // AUTOPILOT_UTIL_H
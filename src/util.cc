#include "util.h"
#include "config.h"
#include <cstdlib>


namespace util {
  constexpr float INV_RAND_MAX = 1.0f / RAND_MAX;

  void float_to_motor(Motor dest[], const float src[], uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
      if (src[i] < 0) dest[i] = 0;
      else dest[i] = static_cast<Motor>(roundf(src[i]));
    }
  }

  void clamp(float &value, const float min, const float max) {
    if (value < min) value = min;
    if (value > max) value = max;
  }

  float clamp_input(const float value, const float min, const float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
  }

  float uniform_random_number(const float min, const float max) {
    return min + (max - min) * (static_cast<float>(std::rand()) * INV_RAND_MAX);
  }

  float normal_distribution(const float mu, const float sigma) {
    float U1 = uniform_random_number(0.0000000001f, 1);
    const float U2 = uniform_random_number(0, 1);

    const float Z = sqrtf(-2 * logf(U1)) * cosf(2 * ap_constants::PI * U2);

    return mu + sigma * Z;
  }

  /**
   * Inertial-Coordinate-System definition:
   *  x+ := forward
   *  y+ := left
   *  z+ := upwards
   *  roll+ := rotate right (to y-)
   *  pitch+ := rotate forwards (to x+)
   *  yaw+ := rotate to the right on the z-axis
   */

  Vector3 body_to_inertial(Vector3 input, Vector3 orientation) {
    orientation = orientation * ap_constants::DEG_TO_RAD;
    const float cr = cosf(orientation.x);
    const float sr = sinf(orientation.x);
    const float cp = cosf(orientation.y);
    const float sp = sinf(orientation.y);
    const float cy = cosf(orientation.z);
    const float sy = sinf(orientation.z);

    // Rotation matrix R = Rz * Ry * Rx
    float R[3][3] = {
        {cy*cp,         cy*sp*sr - sy*cr,    cy*sp*cr + sy*sr},
        {sy*cp,         sy*sp*sr + cy*cr,    sy*sp*cr - cy*sr},
        {-sp,           cp*sr,                cp*cr}
    };

    Vector3 result;
    result.x = R[0][0] * input.x + R[0][1] * input.y + R[0][2] * input.z;
    result.y = R[1][0] * input.x + R[1][1] * input.y + R[1][2] * input.z;
    result.z = R[2][0] * input.x + R[2][1] * input.y + R[2][2] * input.z;

    return result;
  }
}

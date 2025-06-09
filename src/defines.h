#ifndef AUTOPILOT_DEFINES_H
#define AUTOPILOT_DEFINES_H

#include "config.h"

#include <cmath>
#include <cstdint>

#define CHECK_GT(X, Y) static_assert(X >= Y)

typedef uint16_t Motor;
typedef int8_t input_size_t;

namespace ap_constants {
  constexpr float GRAVITY = 9.80665f;
  constexpr float EPSILON = 1e-6;
  constexpr float PI = 3.14159265359f;
  constexpr float DEG_TO_RAD = PI / 180;
  constexpr float RAD_TO_DEG = 180 / PI;
} // namespace ap_constants

typedef struct {
  input_size_t x1; // Left stick: left <-> right
  input_size_t y1; // Left stick: down <-> up
  input_size_t x2; // Right stick: left <-> right
  input_size_t y2; // Right stick: down <-> up
} Input;

typedef struct {
  bool x;
  bool y;
  bool z;
} Bool;

typedef struct Vector3 {
  float x; // roll
  float y; // pitch
  float z; // yaw

  Vector3 operator+(const Vector3 v) const {
    return {x + v.x, y + v.y, z + v.z};
  }

  Vector3 operator-(const Vector3 v) const {
    return {x - v.x, y - v.y, z - v.z};
  }

  Vector3 operator*(const Vector3 v) const {
    return {x * v.x, y * v.y, z * v.z};
  }

  Vector3 operator*(const float c) const {
    return {x * c, y * c, z * c};
  }

  Vector3 operator/(const Vector3 v) const {
    return {x / v.x, y / v.y, z / v.z};
  }

  void operator+=(const Vector3 v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }

  float norm() const {
    return sqrtf(x * x + y * y + z * z);
  }

  Vector3 normalized() const {
    const float norm_ = norm();
    if (norm_ == 0) return {0, 0, 0};
    const float n = 1.0f / norm_;
    return {x * n, y * n, z * n};
  }

} Vector3;

typedef struct State {
  Motor output[config::aircraft::MAX_OUTPUTS] = {0}; // TODO split for idle mode (dont want to set gimbal servos to 0 all the time)
  bool airborne = false;
  Vector3 position = {0, 0, 0};      // m
  Vector3 velocity = {0, 0, 0};      // m/s
  Vector3 acceleration = {0, 0, 0};  // m/s²
  Vector3 orientation = {0, 0, 0};   // deg
  Vector3 angular_rates = {0, 0, 0}; // deg/s
  Vector3 angular_acc = {0, 0, 0};   // deg/s²
} State;

#endif // AUTOPILOT_DEFINES_H
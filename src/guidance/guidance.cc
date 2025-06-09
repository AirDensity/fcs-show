#include "guidance.h"

// Public

void Guidance::setpoint_deviation(const Setpoint &setpoint, const uint8_t threshold) {
  const auto t = static_cast<float>(threshold);

  deviation = (fabsf(this->setpoint.position.x - setpoint.position.x) > t ||
      fabsf(this->setpoint.position.y - setpoint.position.y) > t ||
      fabsf(this->setpoint.position.z - setpoint.position.z) > t ||
      fabsf(this->setpoint.velocity.x - setpoint.velocity.x) > t ||
      fabsf(this->setpoint.velocity.y - setpoint.velocity.y) > t ||
      fabsf(this->setpoint.velocity.z - setpoint.velocity.z) > t ||
      fabsf(this->setpoint.orientation.x - setpoint.orientation.x) > t ||
      fabsf(this->setpoint.orientation.y - setpoint.orientation.y) > t ||
      fabsf(this->setpoint.orientation.z - setpoint.orientation.z) > t ||
      fabsf(this->setpoint.rate.x - setpoint.rate.x) > t ||
      fabsf(this->setpoint.rate.y - setpoint.rate.y) > t ||
      fabsf(this->setpoint.rate.z - setpoint.rate.z) > t ||
      abs(this->setpoint.thrust - setpoint.thrust) > t);
}

// Private

void Guidance::filter_input(const Setpoint &setpoint, const float alpha) {
  this->setpoint.position = (setpoint.position * alpha) + (this->setpoint.position * (1 - alpha));
  this->setpoint.velocity = (setpoint.velocity * alpha) + (this->setpoint.velocity * (1 - alpha));
  this->setpoint.orientation = (setpoint.orientation * alpha) + (this->setpoint.orientation * (1 - alpha));
  this->setpoint.rate = (setpoint.rate * alpha) + (this->setpoint.rate * (1 - alpha));
  this->setpoint.thrust = static_cast<input_size_t>(
      static_cast<float>(setpoint.thrust) * alpha + static_cast<float>(this->setpoint.thrust) * (1 - alpha)
    );
}

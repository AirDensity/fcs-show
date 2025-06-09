#include "navigation.h"
#include "../util.h"

Navigation::Navigation(Aircraft *aircraft, const float *dt, const SensorData *sensors) : aircraft(aircraft),
  sensors(sensors),
  dt(dt) {
  // TODO determine better initial values and implement adaptive tuning based on quality measures.
  R.imu = {0.1f, 0.1f, 0.1f};
  R.gps = {25.0f, 25.0f, 100.0f};
  R.bmp = 2.25f;

  P.imu = {0.1f, 0.1f, 0.1f};
  P.gps = {0.1f, 0.1f, 0.1f};
  P.bmp = 0.1f;

  Q.imu = {0.01f, 0.01f, 0.01f};
  Q.gps = {0.01f, 0.01f, 0.01f};
  Q.bmp = 0.01f;
}

// TODO test
void Navigation::update() {
  // predict
  State predict = aircraft->state;
  aircraft->simulate(*dt, &predict);

  // Update orientation
  Vector3 prev_orientation = aircraft->state.orientation;
  // TODO get absolute orientation from sensor

  prev_orientation = aircraft->state.orientation - prev_orientation;
  aircraft->state.angular_rates += (prev_orientation * (1 / (*dt)));
  aircraft->state.angular_acc += (prev_orientation * (1 / ((*dt) * (*dt))));

  // update imu TODO deal with gravity
  if (use_imu) {
    Vector3 imu_accel = sensors->imu.accel; // Measure
    util::body_to_inertial(imu_accel, aircraft->state.orientation);
    P.imu = (Vector3{1, 1, 1} - K.imu) * P.imu + Q.imu; // Extrapolate Covariance
    K.imu = P.imu / (P.imu + R.imu); // Update Kalman Gain
    aircraft->state.acceleration = predict.acceleration + K.imu * (imu_accel - predict.acceleration);
    aircraft->state.position += (aircraft->state.velocity * (*dt)) +
                                (aircraft->state.acceleration * (*dt) * (*dt) * 0.5f);
    aircraft->state.velocity += aircraft->state.acceleration * (*dt);
  }

  // update bmp
  if (use_bmp) {
    float bmp_z = sensors->bmp.height; // Measure
    P.bmp = (1 - K.bmp) * P.bmp + Q.bmp; // Extrapolate Covariance
    K.bmp = P.bmp / (P.bmp + R.bmp); // Update Kalman Gain
    float prev_bmp = aircraft->state.position.z;
    aircraft->state.position.z = aircraft->state.position.z + K.bmp * (bmp_z - aircraft->state.position.z);
    prev_bmp = aircraft->state.position.z - prev_bmp;
    aircraft->state.velocity.z += (prev_bmp * (1 / (*dt)));
    aircraft->state.acceleration.z += (prev_bmp * (1 / ((*dt) * (*dt))));
  }

  // update gps
  if (use_gps) {
    Vector3 gps_pos = sensors->gps.position; // Measure
    P.gps = (Vector3{1, 1, 1} - K.gps) * P.gps + Q.gps; // Extrapolate Covariance
    K.gps = P.gps / (P.gps + R.gps); // Update Kalman Gain
    Vector3 prev_gps = aircraft->state.position;
    aircraft->state.position = aircraft->state.position + K.gps * (gps_pos - aircraft->state.position);
    prev_gps = aircraft->state.position - prev_gps;
    aircraft->state.velocity += (prev_gps * (1 / (*dt)));
    aircraft->state.acceleration += (prev_gps * (1 / ((*dt) * (*dt))));
  }
}

float Navigation::pressure_to_height(float current, float reference) {
  return 0.0f; // TODO
}

void Navigation::set_active_sensors(const bool imu, const bool gps, const bool bmp) {
  use_imu = imu;
  use_gps = gps;
  use_bmp = bmp;
}

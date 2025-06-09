#ifndef AUTOPILOT_NAVIGATION_H
#define AUTOPILOT_NAVIGATION_H

#include "../defines.h"
#include "../aircraft/aircraft.h"

class Navigation {
public:
  typedef struct {
    Vector3 position;
    //float latitude;           // deg
    //float longitude;          // deg
    //float altitude;           // m
    //float speed;              // m/s
    //float heading;            // deg (0-360)
    //uint8_t num_satellites;
    //float hdop;               // Horizontal Delution of Precision (quality)
    //bool valid_fix;           // (quality)
    //unsigned long timestamp;  // milliseconds
  } GPSData;

  typedef struct {
    Vector3 accel; // m/s²
    Vector3 gyro; // deg/s or rad/s
    Vector3 mag; // µT
    //unsigned long timestamp; // milliseconds
  } IMUData;

  typedef struct {
    float height;
    //float pressure;
    //float temperature;
    //unsigned long timestamp;        // milliseconds
  } BMPData;

  typedef struct {
    GPSData gps;
    IMUData imu;
    BMPData bmp;
  } SensorData;

  Navigation(Aircraft *aircraft, const float *dt, const SensorData *sensors);

  void update();

  float pressure_to_height(float current, float reference);

  void set_active_sensors(bool imu, bool gps, bool bmp);

  void disable_imu() { use_imu = false; }
  void disable_gps() { use_gps = false; }
  void disable_bmp() { use_bmp = false; }

private:
  const float *dt;
  Aircraft *aircraft;
  const SensorData *sensors;

  bool use_imu = true, use_gps = true, use_bmp = true;

  // Kalman Filter
  typedef struct Sensors {
    Vector3 imu = {0, 0, 0};
    Vector3 gps = {0, 0, 0};
    float bmp = 0;
  } Sensors;

  Sensors K; // Kalman Gain
  Sensors R; // Measurement Variance
  Sensors P; // Process Variance
  Sensors Q; // Process Noise Covariance
};

#endif //AUTOPILOT_NAVIGATION_H

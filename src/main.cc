#include <cstdio>
#include <iostream>
#include <fstream>
#include <chrono>

#include "mpc/control_mode/control_mode.h"
#include "aircraft/drone.h"
#include "autopilot.h"
#include "util.h"


// Data
Drone drone;
float elapsed_time;

void print(const Aircraft &a) {
  printf("Position:\n\tX: %f\t,Y: %f\t,Z: %f\n", a.state.position.x, a.state.position.y, a.state.position.z);
  printf("Velocity:\n\tX: %f\t,Y: %f\t,Z: %f\n", a.state.velocity.x, a.state.velocity.y, a.state.velocity.z);
  printf("Acceleration:\n\tX: %f\t,Y: %f\t,Z: %f\n", a.state.acceleration.x, a.state.acceleration.y, a.state.acceleration.z);
  printf("Orientation:\n\tRoll: %f\t,Pitch: %f\t,Yaw: %f\n", a.state.orientation.x, a.state.orientation.y, a.state.orientation.z);
  printf("Rate:\n\tRoll: %f\t,Pitch: %f\t,Yaw: %f\n", a.state.angular_rates.x, a.state.angular_rates.y, a.state.angular_rates.z);
  printf("a.state. Acc:\n\tRoll: %f\t,Pitch: %f\t,Yaw: %f\n", a.state.angular_acc.x, a.state.angular_acc.y, a.state.angular_acc.z);
  printf("Rotors:\n\tFL: %d\t,FR: %d\t,BL: %d\t,BR: %d\n\n", a.state.output[0], a.state.output[1], a.state.output[2], a.state.output[3]);
}

int main() {
  std::ofstream data_file("Data.txt");

  elapsed_time = 0.01f;
  bool use_real_time = false;
  drone.state.airborne = true;
  float noise = 0;

  Navigation::SensorData *sensor_data = nullptr;

  Autopilot ap(&drone, &elapsed_time, sensor_data);

  int iterations = 2000;
  ap.set_control_mode(ControlMode::ALT_HOLD);
  ap.set_sampling_time(0);
  ap.set_input_filter(true);
  Vector3 setpoint = {-30, 20, 0};
  float thrust = 0;

  float max_p_x = 0, max_p_y = 0, max_p_z = 0;
  float max_roll = 0, max_pitch = 0, max_yaw = 0;
  float max_r_r = 0, max_p_r = 0, max_y_r = 0;
  float max_r_a = 0, max_p_a = 0, max_y_a = 0;
  float max_v_x = 0, max_v_y = 0, max_v_z = 0;
  float max_acc_x = 0, max_acc_y = 0, max_acc_z = 0;
  int rotors[4] = {0, 0, 0, 0};
  int its_to_h = 0;
  uint8_t its = 1;
  auto start = std::chrono::high_resolution_clock::now();
  auto start_2 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; i++) {
    auto new_thrust = static_cast<input_size_t>(thrust + util::uniform_random_number(-noise, noise));
    Vector3 new_setpoint = {
        setpoint.x + util::uniform_random_number(-noise, noise),
        setpoint.y + util::uniform_random_number(-noise, noise),
        setpoint.z + util::uniform_random_number(-noise, noise),
    };

    ap.input({
      static_cast<input_size_t>(setpoint.z),
      static_cast<input_size_t>(new_thrust),
      static_cast<input_size_t>(new_setpoint.x),
      static_cast<input_size_t>(new_setpoint.y)
    });
    ap.compute(drone.state.output);
    rotors[0] += drone.state.output[0];
    rotors[1] += drone.state.output[1];
    rotors[2] += drone.state.output[2];
    rotors[3] += drone.state.output[3];
    drone.simulate(elapsed_time, &drone.state);

    if (fabsf(drone.state.position.x) > fabsf(max_p_x)) max_p_x = drone.state.position.x;
    if (fabsf(drone.state.position.y) > fabsf(max_p_y)) max_p_y = drone.state.position.y;
    if (fabsf(drone.state.position.z) > fabsf(max_p_z)) max_p_z = drone.state.position.z;
    if (fabsf(drone.state.velocity.x) > fabsf(max_v_x)) max_v_x = drone.state.velocity.x;
    if (fabsf(drone.state.velocity.y) > fabsf(max_v_y)) max_v_y = drone.state.velocity.y;
    if (fabsf(drone.state.velocity.z) > fabsf(max_v_z)) max_v_z = drone.state.velocity.z;
    if (fabsf(drone.state.acceleration.x) > fabsf(max_acc_x)) max_acc_x = drone.state.acceleration.x;
    if (fabsf(drone.state.acceleration.y) > fabsf(max_acc_y)) max_acc_y = drone.state.acceleration.y;
    if (fabsf(drone.state.acceleration.z) > fabsf(max_acc_z)) max_acc_z = drone.state.acceleration.z;
    if (fabsf(drone.state.orientation.x) > fabsf(max_roll)) max_roll = drone.state.orientation.x;
    if (fabsf(drone.state.orientation.y) > fabsf(max_pitch)) max_pitch = drone.state.orientation.y;
    if (fabsf(drone.state.orientation.z) > fabsf(max_yaw)) max_yaw = drone.state.orientation.z;
    if (fabsf(drone.state.angular_rates.x) > fabsf(max_r_r)) max_r_r = drone.state.angular_rates.x;
    if (fabsf(drone.state.angular_rates.y) > fabsf(max_p_r)) max_p_r = drone.state.angular_rates.y;
    if (fabsf(drone.state.angular_rates.z) > fabsf(max_y_r)) max_y_r = drone.state.angular_rates.z;
    if (fabsf(drone.state.angular_acc.x) > fabsf(max_r_a)) max_r_a = drone.state.angular_acc.x;
    if (fabsf(drone.state.angular_acc.y) > fabsf(max_p_a)) max_p_a = drone.state.angular_acc.y;
    if (fabsf(drone.state.angular_acc.z) > fabsf(max_y_a)) max_y_a = drone.state.angular_acc.z;

    if ((i + 1) % 10 == 0) {
      /*printf("Iteration: %d\tTime: %fs\n", i + 1, (float) (i + 1) * elapsed_time);
      Setpoint u = *ap.guidance.wp_manager.get_current_waypoint();
      printf("Pos: %f, %f, %f\n", u.position.x, u.position.y, u.position.z);
      printf("Vel: %f, %f, %f\n", u.velocity.x, u.velocity.y, u.velocity.z);
      printf("Ori: %f, %f, %f\n", u.orientation.x, u.orientation.y, u.orientation.z);
      printf("Rat: %f, %f, %f\n", u.rate.x, u.rate.y, u.rate.z);*/
    }

    if (i == 200) {
      setpoint = {10, -5, 0};
      thrust = 0;
    }
    if (i == 500) {
      setpoint = {0, 0, 0};
      thrust = 0;
    }

    data_file << drone.state.acceleration.x << "," << drone.state.acceleration.y << "," << drone.state.acceleration.z << ";";
    data_file << drone.state.velocity.x << "," << drone.state.velocity.y << "," << drone.state.velocity.z << ";";
    data_file << drone.state.position.x << "," << drone.state.position.y << "," << drone.state.position.z << ";";
    data_file << drone.state.angular_acc.x << "," << drone.state.angular_acc.y << "," << drone.state.angular_acc.z << ";";
    data_file << drone.state.angular_rates.x << "," << drone.state.angular_rates.y << "," << drone.state.angular_rates.z << ";";
    data_file << drone.state.orientation.x << "," << drone.state.orientation.y << "," << drone.state.orientation.z << ";";
    data_file << drone.state.output[0] << "," << drone.state.output[1] << "," << drone.state.output[2] << "," << drone.state.output[3] << "\n";
    if(use_real_time) {
      auto end = std::chrono::high_resolution_clock::now();
      elapsed_time = std::chrono::duration<float, std::milli>(end - start_2).count() / 1000.0f;
      start_2 = std::chrono::high_resolution_clock::now();
      //printf("%f\n", elapsed_time);
    }
  }
  printf("%f\n", elapsed_time);
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - start;

  data_file.close();
  printf("Elapsed Time: %f / %f\n", duration.count(), (float) (iterations) * elapsed_time);
  printf("MAX Values:\n");
  printf("Iterations to height: %d / %d\n", its_to_h, iterations);
  printf("Position:\n\tX: %f\t,Y: %f\t,Z: %f\n", max_p_x, max_p_y, max_p_z);
  printf("Velocity:\n\tX: %f\t,Y: %f\t,Z: %f\n", max_v_x, max_v_y, max_v_z);
  printf("Acceleration:\n\tX: %f\t,Y: %f\t,Z: %f\n", max_acc_x, max_acc_y, max_acc_z);
  printf("Orientation:\n\tRoll: %f\t,Pitch: %f\t,Yaw: %f\n", max_roll, max_pitch, max_yaw);
  printf("Rates:\n\tRoll: %f\t,Pitch: %f\t,Yaw: %f\n", max_r_r, max_p_r, max_y_r);
  printf("A. Acc:\n\tRoll: %f\t,Pitch: %f\t,Yaw: %f\n", max_r_a, max_p_a, max_y_a);
  printf("Avg. Rotors:\n\tFL: %f\tFR: %f\tBL: %f\tBR: %f\n", (float) rotors[0] / (float) iterations,
         (float) rotors[1] / (float) iterations, (float) rotors[2] / (float) iterations,
         (float) rotors[3] / (float) iterations);

  return 0;
}
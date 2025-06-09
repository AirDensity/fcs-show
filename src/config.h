#ifndef AUTOPILOT_CONFIG_H
#define AUTOPILOT_CONFIG_H

#include <cstdint>

namespace config {
  namespace aircraft {
    constexpr uint8_t MAX_OUTPUTS = 4;
  } // namespace aircraft

  namespace mpc {
    constexpr uint8_t HORIZON = 5;

    namespace optimization {
      constexpr uint8_t MAX_OBJECTIVES = 10;
      constexpr bool USE_WARM_START = true;
      constexpr uint8_t MAX_DEVIATION = 4;

      namespace mopso {
        constexpr uint8_t PARTICLES = 30; // Decrease for faster execution. Increase for better results.
        constexpr uint8_t ITERATIONS = PARTICLES; // Decrease for faster execution. Increase for better results.
        constexpr uint8_t PARETO_FRONT_SIZE = PARTICLES;

        // Terminate if global-best hasn't been updated for a certain number of iterations.
        constexpr uint8_t STAGNATION_LIMIT = ITERATIONS; // Set to ITERATIONS to disarm.

        // Mutation constants for particle update
        constexpr float PARTICLE_PROBABILITY = 0.0f;
        constexpr float INDIVIDUAL_PROBABILITY = 0.0f;

        // Define the percentage of particles that will be warm-started
        constexpr float WARM_START_FACTOR = 0.5f; // [0 - 1]; 0: No warm starts; 1: Only warm starts
      } // namespace mopso
    } // namespace optimization
  } // namespace mpc
} // namespace config

#endif // AUTOPILOT_CONFIG_H
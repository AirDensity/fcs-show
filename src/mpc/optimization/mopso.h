#ifndef AUTOPILOT_MPC_OPTIMIZATION_MOPSO_H
#define AUTOPILOT_MPC_OPTIMIZATION_MOPSO_H

#include "optimization.h"
#include "../../aircraft/aircraft.h"
#include "../control_mode/control_mode.h"

/**
 * Multi Objective Particle Swarm Optimization
 * - Using a pareto front to maintain an archive of possible solutions.
 * - Using crowding distance to replace solutions within the archive once it is full.
 * - The particle leader during the optimization is selected at random from the archive for each particle.
 */
class MOPSO final : protected Optimization {
public:
  static void compute(const Aircraft *aircraft, const ControlMode *mode,
                      float global_best[config::mpc::HORIZON][config::aircraft::MAX_OUTPUTS], bool warm_start);

private:
  typedef struct ArchiveSolution {
    float position[config::mpc::HORIZON][config::aircraft::MAX_OUTPUTS] = {0};
    float fitness[config::mpc::optimization::MAX_OBJECTIVES] = {0};
    float crowding_distance = 0;
  } ArchiveSolution;

  typedef struct Archive {
    ArchiveSolution solution[config::mpc::optimization::mopso::PARETO_FRONT_SIZE] = {0};
    uint8_t size = 0;
  } Archive;

  typedef struct Particle {
    float position[config::mpc::HORIZON][config::aircraft::MAX_OUTPUTS] = {0};
    float fitness[config::mpc::optimization::MAX_OBJECTIVES] = {0};
    float p_best[config::mpc::HORIZON][config::aircraft::MAX_OUTPUTS] = {0};
    float p_best_fitness[config::mpc::optimization::MAX_OBJECTIVES] = {0};
  } Particle;

  static void init_particle(const Aircraft *aircraft, const ControlMode *mode, Particle *p,
                            float initial[config::mpc::HORIZON][config::aircraft::MAX_OUTPUTS], bool warm_start);

  static void update_particle(const Aircraft *aircraft, const ControlMode *mode, Particle *p,
                              const ArchiveSolution *g_best);

  static bool update_archive(Archive *archive, const Particle *p, uint8_t NUM_OBJECTIVES);

  static void calculate_crowding_distance(Archive *archive, uint8_t NUM_OBJECTIVES);

  static uint8_t select_final_solution(const Archive *archive, uint8_t NUM_OBJECTIVES);

  static uint8_t choose_global_guide(const Archive *archive);
};

#endif // AUTOPILOT_MPC_OPTIMIZATION_MOPSO_H

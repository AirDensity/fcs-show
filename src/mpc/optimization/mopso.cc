#include "mopso.h"
#include "../../util.h"
#include <cstring>

using namespace config::mpc;
using namespace config::mpc::optimization::mopso;

// Public methods

void MOPSO::compute(const Aircraft *aircraft, const ControlMode *mode,
                    float global_best[HORIZON][config::aircraft::MAX_OUTPUTS], bool warm_start) {
  if (aircraft == nullptr || mode == nullptr || global_best == nullptr) return;

  const uint8_t NUM_OBJECTIVES = mode->num_objectives();

  Particle swarm[PARTICLES];
  Archive archive;

  // Initialise swarm
  for (uint8_t i = 0; i < PARTICLES; i++) {
    if (i >= PARTICLES * WARM_START_FACTOR) warm_start = false;
    init_particle(aircraft, mode, &swarm[i], global_best, warm_start);
    update_archive(&archive, &swarm[i], NUM_OBJECTIVES);
  }

  // Optimize
  uint8_t stagnation_counter = 0;
  for (uint8_t iter = 0; iter < ITERATIONS; iter++) {
    bool improved = false;
    Archive temp_archive = archive;
    calculate_crowding_distance(&temp_archive, NUM_OBJECTIVES);

    // Update particles
    for (auto &particle: swarm) {
      const uint8_t guide = choose_global_guide(&archive);
      update_particle(aircraft, mode, &particle, &archive.solution[guide]);
    }

    // Update archive
    for (auto &particle: swarm) {
      improved |= update_archive(&archive, &particle, NUM_OBJECTIVES);
    }

    if (improved) stagnation_counter = 0;
    else stagnation_counter++;

    // Early exit condition if the optimization does not significantly improve the result anymore.
    if (stagnation_counter >= STAGNATION_LIMIT) {
      break;
    }
  }

  const uint8_t index = select_final_solution(&archive, NUM_OBJECTIVES);
  memcpy(global_best, archive.solution[index].position, GLOBAL_BEST_SIZE);
}

// Private methods

void MOPSO::init_particle(const Aircraft *aircraft, const ControlMode *mode, Particle *p, float initial[HORIZON][config::aircraft::MAX_OUTPUTS], const bool warm_start) {
  const uint8_t OUTPUTS = aircraft->get_output_size();

  for (uint8_t i = 0; i < OUTPUTS; i++) {
    const Motor MIN = aircraft->get_min_output(i);
    const Motor MAX = aircraft->get_max_output(i);

    if (warm_start) {
      for (uint8_t j = 1; j < HORIZON; j++) {
        p->position[j - 1][i] = initial[j][i];
      }
      p->position[HORIZON - 1][i] = util::uniform_random_number(MIN, MAX);
    } else {
      for (uint8_t j = 0; j < HORIZON; j++) {
        p->position[j][i] = util::uniform_random_number(MIN, MAX);
      }
    }
  }

  // Update fitness
  mode->cost_function(p->fitness, p->position);
  memcpy(p->p_best, p->position, GLOBAL_BEST_SIZE);
  memcpy(p->p_best_fitness, p->fitness, FITNESS_SIZE);
}

void MOPSO::update_particle(const Aircraft *aircraft, const ControlMode *mode, Particle *p, const ArchiveSolution *g_best) {
  const uint8_t OUTPUTS = aircraft->get_output_size();
  const uint8_t NUM_OBJECTIVES = mode->num_objectives();

  // Update position
  const bool mutate = util::uniform_random_number(0, 1) < PARTICLE_PROBABILITY;
  for (uint8_t i = 0; i < OUTPUTS; i++) {
    const Motor MIN = aircraft->get_min_output(i);
    const Motor MAX = aircraft->get_max_output(i);

    for (uint8_t j = 0; j < HORIZON; j++) {
      const float mean = 0.5f * (p->p_best[j][i] + g_best->position[j][i]);
      const float stddev = fabsf(p->p_best[j][i] - g_best->position[j][i]);

      p->position[j][i] = util::normal_distribution(mean, stddev);

      // Mutate the particle if the overall mutation is active and the particle gets selected.
      if (mutate && util::uniform_random_number(0, 1) < INDIVIDUAL_PROBABILITY) {
        p->position[j][i] = util::uniform_random_number(MIN, MAX);
      }

      util::clamp(p->position[j][i], MIN, MAX);
    }
  }

  // Update fitness
  for (uint8_t i = 0; i < NUM_OBJECTIVES; i++) p->fitness[i] = 0;
  mode->cost_function(p->fitness, p->position);

  // Update personal-best
  if (dominates(p->fitness, p->p_best_fitness, NUM_OBJECTIVES) ||
      (!dominates(p->p_best_fitness, p->fitness, NUM_OBJECTIVES) && util::uniform_random_number(0, 1) < 0.5f)) {
    memcpy(p->p_best, p->position, GLOBAL_BEST_SIZE);
    memcpy(p->p_best_fitness, p->fitness, FITNESS_SIZE);
  }
}

uint8_t MOPSO::choose_global_guide(const Archive *archive) {
  const auto i = static_cast<uint8_t>(util::uniform_random_number(0, archive->size - 1));
  auto j = static_cast<uint8_t>(util::uniform_random_number(0, archive->size - 1));

  // There is only one solution if the archive-size is <= 1, so we can already stop here.
  if (archive->size <= 1) {
    return i;
  }

  // If i and j are equal, we choose j as a neighbour of i.
  if (i == j) {
    if (j == archive->size - 1) j--;
    else if (j == 0) j++;
    else { // 50-50 that j = i - 1 or j = i + 1
      if (util::uniform_random_number(0, 1) < 0.5) j--;
      else j++;
    }
  }

  if (archive->solution[i].crowding_distance > archive->solution[j].crowding_distance) return i;
  if (archive->solution[i].crowding_distance < archive->solution[j].crowding_distance) return j;
  return util::uniform_random_number(0, 1) < 0.5f ? i : j;
}

bool MOPSO::update_archive(Archive *archive, const Particle *p, const uint8_t NUM_OBJECTIVES) {
  for (uint8_t i = 0; i < archive->size;) {
    // Check if particle is equal to an existing solution in the archive by comparing their fitness.
    bool is_equal = true;
    for (uint8_t j = 0; j < NUM_OBJECTIVES; j++) {
      if (fabsf(archive->solution[i].fitness[j] - p->fitness[j]) >= ap_constants::EPSILON) {
        is_equal = false;
        break;
      }
    }

    if (is_equal || dominates(archive->solution[i].fitness, p->fitness, NUM_OBJECTIVES)) {
      return false;
    }

    if (dominates(p->fitness, archive->solution[i].fitness, NUM_OBJECTIVES)) {
      archive->solution[i] = archive->solution[--archive->size];
      continue;
    }

    i++;
  }

  uint8_t index = 0;
  if (archive->size < PARETO_FRONT_SIZE) {
    index = archive->size;
    archive->size++;
  } else {
    for (uint8_t i = 0; i < archive->size; i++) {
      // Prefer selecting old solutions
      if (archive->solution[index].crowding_distance < -ap_constants::EPSILON && archive->solution[i].crowding_distance > ap_constants::EPSILON) {
        index = i;
        continue;
      }

      // Select the solution with lower crowding distance
      if (archive->solution[i].crowding_distance < archive->solution[index].crowding_distance - ap_constants::EPSILON) {
        index = i;
      }
    }
  }

  memcpy(archive->solution[index].position, p->position, GLOBAL_BEST_SIZE);
  memcpy(archive->solution[index].fitness, p->fitness, FITNESS_SIZE);
  archive->solution[index].crowding_distance = -1;

  return true;
}

void MOPSO::calculate_crowding_distance(Archive *archive, const uint8_t NUM_OBJECTIVES) {
  // Reset crowding distance to zero
  for (uint8_t i = 0; i < archive->size; i++) {
    archive->solution[i].crowding_distance = 0;
  }

  // Process each objective
  for (uint8_t objIndex = 0; objIndex < NUM_OBJECTIVES; objIndex++) {
    // In-place selection sort
    for (uint8_t i = 0; i < archive->size - 1; i++) {
      uint8_t minIndex = i;
      for (uint8_t j = i + 1; j < archive->size; j++) {
        if (archive->solution[j].fitness[objIndex] < archive->solution[minIndex].fitness[objIndex]) {
          minIndex = j;
        }
      }
      if (minIndex != i) {
        const ArchiveSolution temp = archive->solution[i];
        archive->solution[i] = archive->solution[minIndex];
        archive->solution[minIndex] = temp;
      }
    }

    const float min_value = archive->solution[0].fitness[objIndex];
    const float max_value = archive->solution[archive->size - 1].fitness[objIndex];
    const float range = max_value - min_value <= ap_constants::EPSILON ? ap_constants::EPSILON : max_value - min_value;

    // Assign crowding distance to the boundary solutions
    archive->solution[0].crowding_distance = 1e10f;
    archive->solution[archive->size - 1].crowding_distance = 1e10f;

    // Calculate crowding distance for the rest of the solutions
    for (uint8_t i = 1; i < archive->size - 1; i++) {
      const float distance = (archive->solution[i + 1].fitness[objIndex] - archive->solution[i - 1].fitness[objIndex]);
      archive->solution[i].crowding_distance += distance / range;
    }
  }
}

uint8_t MOPSO::select_final_solution(const Archive *archive, const uint8_t NUM_OBJECTIVES) {
  float max_fitness[optimization::MAX_OBJECTIVES];
  float min_fitness[optimization::MAX_OBJECTIVES];

  // Find the maximum and minimum fitness for each objective.
  for (uint8_t i = 0; i < archive->size; i++) {
    for (uint8_t j = 0; j < NUM_OBJECTIVES; j++) {
      if (i == 0 || archive->solution[i].fitness[j] > max_fitness[j]) {
        max_fitness[j] = archive->solution[i].fitness[j];
      }
      if (i == 0 || archive->solution[i].fitness[j] < min_fitness[j]) {
        min_fitness[j] = archive->solution[i].fitness[j];
      }
    }
  }

  // Find solution with minimum distance to the ideal solution.
  uint8_t index = 0;
  float min_distance = 0;
  for (uint8_t i = 0; i < archive->size; i++) {
    float distance = 0;
    for (uint8_t j = 0; j < NUM_OBJECTIVES; j++) {
      const float fit = archive->solution[i].fitness[j] - min_fitness[j];
      const float div = max_fitness[j] - min_fitness[j];
      if (div != 0) distance += powf(fit / div, 2); // euclidian distance
    }
    distance = sqrtf(distance);

    if (i == 0 || distance < min_distance) {
      index = i;
      min_distance = distance;
    }
  }

  return index;
}
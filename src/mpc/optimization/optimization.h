#ifndef AUTOPILOT_MPC_OPTIMIZATION_H
#define AUTOPILOT_MPC_OPTIMIZATION_H

#include "../../config.h"
#include "../../aircraft/aircraft.h"

class ControlMode;

class Optimization {
public:
  typedef enum {
    MOPSO,
  } Strategy;

  Optimization(const Aircraft *aircraft, const ControlMode *control_mode);

  void compute(float out[config::mpc::HORIZON][config::aircraft::MAX_OUTPUTS], bool warm_start) const;

  void set_strategy(const Strategy strategy) {
    this->strategy = strategy;
  }

  Strategy get_strategy() const {
    return strategy;
  }

protected:
  static constexpr uint16_t GLOBAL_BEST_SIZE = config::mpc::HORIZON * config::aircraft::MAX_OUTPUTS * sizeof(float);
  static constexpr uint16_t FITNESS_SIZE = config::mpc::optimization::MAX_OBJECTIVES * sizeof(float);

  /**
   * Return whether f1 dominates f2 or not.
   * @param f1 Fitness of solution 1
   * @param f2 Fitness of solution 2
   * @param num_objectives Number of objectives to be evaluated.
   * @return Returns true if f1 not worse than f2 in any objective and better in at least one objective. Else false.
   */
  static bool dominates(const float f1[config::mpc::optimization::MAX_OBJECTIVES],
                        const float f2[config::mpc::optimization::MAX_OBJECTIVES],
                        uint8_t num_objectives);

private:
  const Aircraft *aircraft;
  const ControlMode *control_mode;

  Strategy strategy = MOPSO;
};

#endif // AUTOPILOT_MPC_OPTIMIZATION_H
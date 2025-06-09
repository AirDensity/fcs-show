#include "waypoint.h"

#include "../../util.h"

void Waypoint::calculate_cost(const References *refs, const State *state, float *cost) {

}

void Waypoint::init_usage(Guidance::Usage *usage) {

}

void Waypoint::update_dependencies(Guidance *guidance, const State *state) {

}

// Private methods

uint8_t Waypoint::constraint_violations(const Aircraft *a) {
  return 0;
}
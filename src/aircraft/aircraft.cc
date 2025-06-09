#include "aircraft.h"


void Aircraft::limit_to_360(Vector3 &v) {
  v.x = fmodf(v.x, 360.0f);
  v.y = fmodf(v.y, 360.0f);
  v.z = fmodf(v.z, 360.0f);
}

void Aircraft::limit_to_180(Vector3 &v) {
  limit_to_360(v);

  if (v.x > 180) v.x -= 360;
  else if (v.x < -180) v.x += 360;

  if (v.y > 180) v.y -= 360;
  else if (v.y < -180) v.y += 360;

  if (v.z > 180) v.z -= 360;
  else if (v.z < -180) v.z += 360;
}

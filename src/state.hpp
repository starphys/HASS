#pragma once

#include <vector>

#include "vec2.hpp"

struct VehicleState{
  Vec2 position;
  double e;
  double t;
  double current_drill_time_spent;
};

using VehicleHistory = std::vector<VehicleState>;




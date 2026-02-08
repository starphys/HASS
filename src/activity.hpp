#pragma once

#include <variant>
#include <vector>

#include "vec2.h"

struct DrivingActivity{
  // TODO: path planning
  // std::vector<Vec2> path;
  // std::vector<Vec2>::iterator current_goal;
  Vec2 position;
};

struct DrillingActivity{
  double total_time;    // Seconds
};

struct RechargeActivity{
  double target_energy;     // Joules
  double abandon_threshold; // Joules
};

struct HibernationActivity{
  double wakeup_time;       // seconds
};

using Activity = std::variant<
  DrivingActivity,
  DrillingActivity,
  RechargeActivity,
  HibernationActivity
>;
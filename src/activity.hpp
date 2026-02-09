#pragma once

#include <variant>
#include <vector>

#include "vec2.hpp"

struct DrivingActivity{
  // TODO: path planning
  // std::vector<Vec2> path;
  // std::vector<Vec2>::iterator current_goal;
  Vec2 position;
  double smg = 0.1;
  double power = -10.0;
};

struct DrillingActivity{
  double total_time;    // Seconds
  double power = -30.0;
};

struct RechargeActivity{
  double target_energy;     // Joules
  double abandon_threshold; // Joules
  double power = 15.0;
};

struct HibernationActivity{
  double wakeup_time;       // seconds
  double power = -1.0;
};

using Activity = std::variant<
  DrivingActivity,
  DrillingActivity,
  RechargeActivity,
  HibernationActivity
>;
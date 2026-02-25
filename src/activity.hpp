#pragma once

#include <variant>
#include <vector>

#include "path_planner.hpp"
#include "vec2.hpp"

struct PathPlanningActivity{
  Vec2 goal;
};

struct DrivingActivity{
  std::vector<GraphNode> path;
  int current_goal_index;
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
  PathPlanningActivity,
  DrivingActivity,
  DrillingActivity,
  RechargeActivity,
  HibernationActivity
>;
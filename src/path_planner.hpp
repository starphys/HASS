#pragma once

#include "map.hpp"
#include "state.hpp"
#include "vec2.hpp"

#include <vector>

struct GraphNode {
  Vec2 position;
  double time;
};

struct MissionContext {

};

struct VehicleContext {
  double smg(GraphNode& node, MapContext& maps, MissionContext& mission) { 
    // Eventually we'll calculate velocity based on position, maps, and activities
    return 1.0; 
  };
};

struct PlannerConfig {
  MapContext maps;
  WorldContext world;
  MissionContext mission;
  VehicleContext vehicle;
};

std::vector<GraphNode> find_path(VehicleState&, Vec2&, PlannerConfig&);

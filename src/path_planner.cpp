#include "path_planner.hpp"

#include <algorithm>
#include <queue>
#include <unordered_map>

namespace {
  int get_key(GraphNode& node, WorldContext& context) {
    // We only want one node per XYT voxel
    return ((std::floor(node.time) / context.seconds_per_timestep) * context.width * context.height) + (node.position.y * context.width) + (node.position.x);
  }

  double h(GraphNode& node, Vec2& goal, PlannerConfig& config) {
    return (goal - node.position).length() / config.vehicle.smg(node, config.maps, config.mission);
  }

  double d(GraphNode& nodeA, GraphNode& nodeB, PlannerConfig& config) {
    return (nodeB.position - nodeA.position).length() / config.vehicle.smg(nodeB, config.maps, config.mission);
  }

  std::vector<GraphNode> reconstruct_path(std::unordered_map<int, GraphNode>& nodes, std::unordered_map<int, int>& cameFrom, int currentIndex) {
    std::vector<GraphNode> path{};

    while(currentIndex > -1) {
      path.emplace_back(nodes[currentIndex]);
      if(!cameFrom.contains(currentIndex)) { break; }
      currentIndex = cameFrom[currentIndex];
    }

    return path;
  }

  std::vector<GraphNode> get_valid_neighbors(GraphNode& current, PlannerConfig& config) {
    std::vector<GraphNode> neighbors{};

    int x = current.position.x;
    int y = current.position.y;

    // Spatial nodes
    static constexpr std::array<int, 3> offsets{-1, 0, 1};
    for(auto dx : offsets) {
      for(auto dy : offsets) {
        if(dx == 0 && dy == 0) { continue; }
        int newX = x + dx;
        int newY = y + dy;
        
        // Node must be in world bounds
        if(newX < 0 || newX >= config.world.width || newY < 0 || newY >= config.world.height) { continue; }
        
        Vec2 newPosition{static_cast<double>(newX),static_cast<double>(newY)};
        GraphNode newNode = GraphNode(newPosition, current.time);
        int t = current.time + ((current.position - newPosition).length() / config.vehicle.smg(newNode, config.maps, config.mission)); // seconds
        newNode.time = t;

        // Node must be in world bounds
        if(t < 0 || t >= config.world.timesteps) { continue; }
        
        neighbors.push_back(newNode);
      }
    }

    // Wait node, wait only until the exact boundary of the next timestep.
    auto spt = config.world.seconds_per_timestep;
    neighbors.push_back({{static_cast<double>(x), static_cast<double>(y)}, (current.time / spt) * spt + spt});

    return neighbors;
  }

  std::vector<GraphNode> a_star(GraphNode& start, Vec2& goal, PlannerConfig& config) {
    int start_key = get_key(start, config.world);
    std::unordered_map<int, GraphNode> nodes{{start_key, start}};

    using OpenEntry = std::pair<double, int>;
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<OpenEntry>> openSet{}; // fCost, index
    openSet.emplace(std::make_pair(h(start, goal, config), start_key)); 

    std::unordered_map<int, int> cameFrom{{start_key, -1}};

    std::unordered_map<int, double> gScore{{start_key, 0}};
    std::unordered_map<int, double> fScore{{start_key, h(start, goal, config)}};

    while(!openSet.empty()) {
      auto [storedCost, nodeIndex] = openSet.top();
      GraphNode& current = nodes[nodeIndex];
      auto currentScore = gScore[nodeIndex];
      openSet.pop();

      if(storedCost > fScore[nodeIndex]) { continue; }
      if((goal - current.position).length() < 1.0) {
        return reconstruct_path(nodes, cameFrom, nodeIndex);
      }

      auto neighbors = get_valid_neighbors(current, config);

      for(auto & neighbor : neighbors) {
        auto key = get_key(neighbor, config.world);
        
        double tentative_gScore = currentScore + d(current, neighbor, config);
        if(!gScore.contains(key) || tentative_gScore < gScore[key]) {
          nodes[key] = neighbor;
          cameFrom[key] = nodeIndex;
          gScore[key] = tentative_gScore;
          fScore[key] = tentative_gScore + h(neighbor, goal, config);
          openSet.emplace(std::make_pair(fScore[key], key)); 
        }
      }
    
    }

    return {};
  }
}

std::vector<GraphNode> find_path(VehicleState& start, Vec2& goal, PlannerConfig& config) {
  // All GraphNode positions and times should be floored to whole numbers.
  GraphNode startNode{start.position, start.t};
  auto vec = a_star(startNode, goal, config);
  std::reverse(vec.begin(), vec.end());
  return vec;
}
#include <cmath>
#include <deque>
#include <format>
#include <fstream>
#include <iterator>
#include <iostream> 
#include <variant>
#include <vector>

#include "activity.hpp"
#include "map.hpp"
#include "path_planner.hpp"
#include "state.hpp"
#include "vec2.hpp"
#include "vehicle.hpp"

template<class... Ts>
struct overloaded : Ts... { using Ts::operator()...; };

int main() {
  double dt = 1;
  SimpleRover vehicle{};

  VehicleState s{
    {0.0, 0.0},
    vehicle.max_energy,
    0.0,
    0.0
  };

  VehicleHistory states{s};
  std::vector<Activity> activity_history{};

  std::deque<Activity> activities{ 
    PathPlanningActivity{{0.0, 5.0}},
    DrillingActivity{30.0},       // Drain 30 W * 30 Seconds, .25% of battery
    PathPlanningActivity{{1.0, 1.0}},
    RechargeActivity{vehicle.max_energy}, // Recharge fully
    PathPlanningActivity{{2.0, 12.0}},
    HibernationActivity{2400} // Wait until 2400th second of mission
  };

  SlopeMap slope{};
  ShadowMap shadow{};

  auto config = PlannerConfig{
    {
      slope,
      shadow
    },
    {
      256,
      256,
      360,
      1,
      3600,
    },
    {/*TODO*/},
    {/*TODO*/}
  };
  
  while(true) { // Main driver, all state updates happen at this level
    VehicleState next_state{states.back()};
    next_state.t += dt;

    if(activities.empty()) {
      std::cout << std::format("All activities completed by time: {}!\n", next_state.t); 
      break; 
    }
    
    auto activity = activities.front();
    activities.pop_front();

    bool activity_completed = std::visit(overloaded{
        [&](PathPlanningActivity& act) -> bool {
          auto path = find_path(next_state, act.goal, config);
          activities.push_front(DrivingActivity{
            path,
            0
          });

          //TODO: this isn't very nice, maybe bookkeeping activities are distinct from simulation activities
          // alternatively things like updating the time of the simulation belong in activities
          next_state.t -= dt;
          return true;
        },
        [&](DrivingActivity& act) -> bool { 
          auto goal = act.path[act.current_goal_index];
          auto heading = goal.position - next_state.position;
          std::cout << std::format("Goal: {}, Current: {}, Heading: {}\n", goal.position, next_state.position, heading);

          if(heading.length() <= act.smg * dt) { 
            next_state.position = goal.position; // Drop smg as necessary to avoid compounding position error
            ++act.current_goal_index;
          } else {
            next_state.position += heading.normalized() * act.smg * dt;
          }

          next_state.e += act.power * dt;

          return act.current_goal_index >= act.path.size();
        },
        [&](DrillingActivity& act) -> bool {
          next_state.current_drill_time_spent += dt;
          next_state.e += act.power * dt;
          if(next_state.current_drill_time_spent >= act.total_time) {
            next_state.current_drill_time_spent = 0;
            return true;
          } else {
            return false;
          }
        },
        [&](RechargeActivity& act) -> bool { 
          if(next_state.e >= (vehicle.max_energy - (act.power * dt))) { // Never overcharge the battery, even if it means waiting in place for partial dT
            next_state.e = vehicle.max_energy;
            return true;
          }

          next_state.e += act.power * dt;
          return next_state.e >= act.target_energy;
        },
        [&](HibernationActivity act) -> bool { 
          next_state.e += act.power * dt;
          return next_state.t >= act.wakeup_time;
        },
      },
      activity
    );

    states.push_back(next_state);

    if(next_state.e <= 0.0) { 
      std::cout << std::format("failed at time: {}\n", next_state.t);
      break; 
    } // Failed!

    if(!activity_completed) {
      activities.push_front(activity);
    }

    if(activity_completed) { 
      activity_history.push_back(activity);
      std::cout << std::format("position: {}, SoC: {}, time: {}\n", next_state.position, next_state.e, next_state.t);
    }
  }

  
  std::cout << std::format("Simulation finished! Total states: {}\n", states.size());
  for(auto& activity : activity_history) {
    std::visit(overloaded{
        [&](PathPlanningActivity& act) -> void {
          std::cout << std::format("PathPlanningActivity {}\n", act.goal); 
        },
        [&](DrivingActivity& act) -> void { 
          std::cout << std::format("DrivingActivity with {} waypoints ending at {}\n", act.path.size(), act.path.back().position); 
        },
        [&](DrillingActivity& act) -> void {
          std::cout << std::format("DrillingActivity for {}\n", act.total_time); 
        },
        [&](RechargeActivity& act) -> void { 
          std::cout << std::format("RechargeActivity to {}\n", act.target_energy); 
        },
        [&](HibernationActivity act) -> void { 
          std::cout << std::format("HibernationActivity until {}\n", act.wakeup_time); 
        },
      }, activity
    );
  }

  std::cout << "\n";

  std::ofstream csv("/home/starp/projects/HASS/maps/traverse.csv");
  for(auto & s : states) {
    csv << std::format("{}, {}, {},\n", s.position.x, s.position.y, s.t);
  }

  return 0;
}
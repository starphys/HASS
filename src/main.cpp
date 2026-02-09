#include <cmath>
#include <deque>
#include <format>
#include <iterator>
#include <iostream> 
#include <variant>
#include <vector>

#include "activity.hpp"
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

  std::deque<Activity> activities{ 
    DrivingActivity{{0.0, 5.0}},
    DrillingActivity{30.0},       // Drain 30 W * 30 Seconds, .25% of battery
    DrivingActivity{{0.0, 0.0}},
    RechargeActivity{vehicle.max_energy}, // Recharge fully
    DrivingActivity{{0.0, 12.0}},
    HibernationActivity{2400} // Wait until 2400th second of mission
  };
  
  while(true) { // Main driver, all state updates happen at this level
    VehicleState next_state{states.back()};
    next_state.t += dt;

    if(activities.empty()) {
      std::cout << std::format("All activities completed by time: {}!\n", next_state.t); 
      break; 
    }
    
    auto& activity = activities.front();

    bool activity_completed = std::visit(overloaded{
        [&](DrivingActivity& act) -> bool { 
          auto heading = act.position - next_state.position;

          if(heading.length() <= act.smg * dt) { 
            next_state.position = act.position; // Drop smg as necessary to avoid compounding position error
          } else {
            next_state.position += heading.normalized() * act.smg * dt;
          }

          next_state.e += act.power * dt;

          return act.position == next_state.position;
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

    if(activity_completed) { 
      activities.pop_front();
      std::cout << std::format("position: {}, SoC: {}, time: {}\n", next_state.position, next_state.e, next_state.t);
    }


  }

  
  std::cout << std::format("Simulation finished! Total states: {}\n", states.size());

  return 0;
}
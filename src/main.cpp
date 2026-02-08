#include <cmath>
#include <deque>
#include <format>
#include <iterator>
#include <iostream> 
#include <variant>
#include <vector>

#include "activity.hpp"
#include "vec2.h"

template<class... Ts>
struct overloaded : Ts... { using Ts::operator()...; };

int main() {
  double smg = 0.1;                 // m/s
  Vec2 position = {0.0, 0.0};       // m
  double dt = 1;                    // s
  double t = 0.0;                   // s
  double acceptance_radius = smg * dt; // This can be no larger than smg * dt
  double max_charge = 1 * 60 * 60;  // 1 Wh, 3600 J
  double current_charge = max_charge;
  double driving_drain = -10;        // W
  double drilling_drain = -30;       // W
  double hibernation_drain = -1;    // W
  double recharge_rate =  15;       // W
  double drilling_time = 0;         // s

  std::deque<Activity> activities{ 
    DrivingActivity{{0.0, 5.0}},
    DrillingActivity{30.0},       // Drain 30 W * 30 Seconds, .25% of battery
    DrivingActivity{{0.0, 0.0}},
    RechargeActivity{max_charge}, // Recharge fully
    DrivingActivity{{0.0, 12.0}},
    HibernationActivity{2400} // Wait until 2400th second of mission
  };
  
  for(; ; t += dt) { // Main driver, all state updates happen at this level
    if(activities.empty()) {
      std::cout << std::format("All activities completed by time: {}!\n", t); 
      break; 
    }
    auto& activity = activities.front();
    // Check if activity is done
    bool activity_completed = std::visit(overloaded{
        [&](DrivingActivity& act) -> bool { 
          auto heading = act.position - position;

          if(heading.length() <= acceptance_radius) { 
            position = act.position; // Drop smg as necessary to avoid compounding position error
          } else {
            position += heading.normalized() * smg * dt;
          }

          current_charge += driving_drain * dt;

          return act.position == position;
        },
        [&](DrillingActivity& act) -> bool {
          drilling_time += dt;
          current_charge += drilling_drain * dt;
          if(drilling_time >= act.total_time) {
            drilling_time = 0;
            return true;
          } else {
            return false;
          }
        },
        [&](RechargeActivity& act) -> bool { 
          if(current_charge >= (max_charge - (driving_drain * dt))) { // Never overcharge the battery, even if it means waiting in place for partial dT
            current_charge = max_charge;
            return true;
          }

          current_charge += recharge_rate * dt;
          return current_charge >= act.target_energy;
        },
        [&](HibernationActivity act) -> bool { 
          current_charge += hibernation_drain * dt;
          return t >= act.wakeup_time;
        },
      },
      activity
    );

    if(current_charge <= 0.0) { 
      std::cout << std::format("failed at time: {}\n", t);
      break; 
    } // Failed!

    if(activity_completed) { 
      activities.pop_front();
      std::cout << std::format("position: {}, SoC: {}, time: {}\n", position, current_charge, t);
    }
  }

  return 0;
}
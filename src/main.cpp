#include <cmath>
#include <format>
#include <iterator>
#include <iostream> 
#include <vector>

#include "vec2.h"

int main() {
  std::vector<Vec2> tgts{
    {0.0, 5.0},
    {1.0, 1.0},
    {12.0, 0.0}
  };

  double smg = 0.1;                 // m/s
  Vec2 position = {0.0, 0.0};       // m
  double dt = 1;                    // s
  double t = 0.0;                   // s
  double acceptance_radius = smg * dt; // This can be no larger than smg * dt

  for(const auto& tgt : tgts) {
    for(; ; t += dt) {
      auto heading = tgt - position;

      // If we are within the acceptance radius, mark target arrived
      if(heading.length() <= acceptance_radius) { 
        position = tgt; // Drop smg as necessary to avoid compounding position error
        t += dt;
        break; 
      }

      position += heading.normalized() * smg * dt;
    }
    std::cout << std::format("Arrived at tgt: {} at time: {} and position is: {}\n", tgt, t, position);
  }

  return 0;
}
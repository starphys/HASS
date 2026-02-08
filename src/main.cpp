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

  float smg = 0.1;                // m/s
  Vec2 position = {0.0, 0.0};    // m
  float dt = 1;           // s
  float t = 0.0;          // s
  float acceptance_radius = smg * dt;

  for(const auto& tgt : tgts) {
    for(; ; t += dt) {
      auto heading = tgt - position;
      position += heading.normalized() * smg * dt;
      // If we are strictly within one step of the target, mark it arrived
      if(heading.length() < acceptance_radius) { break; }
    }
    std::cout << std::format("Arrived at tgt: {} at time: {} and position is: {}\n", tgt, t, position);
  }

  return 0;
}
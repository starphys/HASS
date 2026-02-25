#pragma once

#include <iostream>
#include <cstdint>
#include <fstream>
#include <vector>

struct WorldContext {
  int width;
  int height;
  int timesteps;
  int meters_per_pixel;
  int seconds_per_timestep;
  // PROJ strings/transforms
};

class SlopeMap {
public:
  SlopeMap() {
    std::ifstream f("/home/starp/projects/HASS/maps/slope.bin", std::ios::binary);

    f.read(reinterpret_cast<char *>(&width), 4);
    f.read(reinterpret_cast<char *>(&height), 4);
    std::cout << width << " " << height << '\n';
    data = std::vector<float>(width * height);
    f.read(reinterpret_cast<char*>(data.data()), width * height * sizeof(float));
  }

  // Caller must floor x and y to the pixel boundary
  float at(int x, int y) const { return data[y * width + x]; }
private:
  int32_t width;
  int32_t height;
  std::vector<float> data;
};

class ShadowMap {
public:
  ShadowMap(){
    std::ifstream f("/home/starp/projects/HASS/maps/shadow.bin", std::ios::binary);

    f.read(reinterpret_cast<char *>(&width), 4);
    f.read(reinterpret_cast<char *>(&height), 4);
    f.read(reinterpret_cast<char *>(&timesteps), 4);

    data = std::vector<uint8_t>(width * height * timesteps);

    f.read(reinterpret_cast<char*>(data.data()), width * height * timesteps * sizeof(uint8_t));
  }

  // Caller must floor x, y, and t to the pixel and timestep boundary
  uint8_t at(int x, int y, int t) const {
    return data[t * width * height + y * width + x];
  }
private:
  int32_t width;
  int32_t height;
  int32_t timesteps;
  std::vector<uint8_t> data;
};

struct MapContext {
  SlopeMap slope;
  ShadowMap shadow;
};
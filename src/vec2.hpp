#pragma once

#include <cmath>
#include <format>

struct Vec2 {
  float x, y;

  // Constructors
  Vec2() : x(0.0f), y(0.0f) {}
  Vec2(float x, float y) : x(x), y(y) {}
  Vec2(float scalar) : x(scalar), y(scalar) {}

  // Addition
  Vec2 operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }
  Vec2& operator+=(const Vec2& other) { x += other.x; y += other.y; return *this; }

  // Subtraction
  Vec2 operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }
  Vec2& operator-=(const Vec2& other) { x -= other.x; y -= other.y; return *this; }

  // Scalar Multiplication
  Vec2 operator*(float scalar) const { return {x * scalar, y * scalar}; }

  // Division
  Vec2 operator/(float scalar) const { return {x / scalar, y / scalar}; }

  // Equality
  bool operator==(const Vec2& other) const { return x == other.x && y == other.y; }

  // Utilities
  float lengthSquared() const { return x * x + y * y; }
  float length() const { return std::sqrt(lengthSquared()); }

  Vec2 normalized() const {
    float len = length();
    if (len > 0) return *this / len;
    return {0, 0};
  }
};

template <>
struct std::formatter<Vec2> : std::formatter<float> {
  auto format(const Vec2& v, std::format_context& ctx) const {
    auto out = ctx.out();
    *out++ = '(';
    ctx.advance_to(out);
    out = std::formatter<float>::format(v.x, ctx);
    *out++ = ',';
    *out++ = ' ';
    ctx.advance_to(out);
    out = std::formatter<float>::format(v.y, ctx);
    *out++ = ')';
    return out;
  }
};
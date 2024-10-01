#pragma once

#include <iostream>
#include <cmath>

namespace nmpc_nav_control {

inline float dist(float x1, float y1, float x2, float y2) {
    return sqrtf((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

inline double dist(double x1, double y1, double x2, double y2) {
    return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

// Source: https://stackoverflow.com/a/11498248
inline float normAngDeg(float angle) {
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0) {
    angle += 360.0f;
  }
  return angle - 180.0f;
}

inline double normAngDeg(double angle) {
  angle = fmod(angle + 180.0, 360.0);
  if (angle < 0) {
    angle += 360.0;
  }
  return angle - 180.0;
}

inline float normAngRad(float angle) {
  angle = fmodf(angle + M_PIf32, M_PIf32 * 2.0f);
  if (angle < 0) {
    angle += M_PIf32 * 2.0f;
  }
  return angle - M_PIf32;
}

inline double normAngRad(double angle) {
  angle = fmod(angle + M_PIf64, M_PIf64 * 2.0);
  if (angle < 0) {
    angle += M_PIf64 * 2.0;
  }
  return angle - M_PIf64;
}

} // namespace nmpc_nav_control
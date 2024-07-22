#pragma once
#include <ostream>

#include "tracer/Vector.h"

typedef Vector Color;
typedef Vector Albedo;

struct PPMColor {
  unsigned short red;
  unsigned short green;
  unsigned short blue;
  explicit PPMColor(const Color &color);
  friend std::ostream &operator<<(std::ostream &os, const struct PPMColor &color);
};
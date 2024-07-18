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

enum MaterialType { Diffuse, Reflective, Constant, Refractive };

struct Material {
  Albedo albedo;
  MaterialType type;
  bool smoothShading;
  float ior;

  Material();
  explicit Material(const Albedo &albedo, const MaterialType &type = Diffuse, const bool smoothShading = false,
                    const float ior = 0);
};
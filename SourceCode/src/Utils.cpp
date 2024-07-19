#include "tracer/Utils.h"

#include <algorithm>

PPMColor::PPMColor(const Color &color) {
  this->red = static_cast<unsigned short>(std::clamp(color[0], 0.0f, 1.0f) * 255);
  this->green = static_cast<unsigned short>(std::clamp(color[1], 0.0f, 1.0f) * 255);
  this->blue = static_cast<unsigned short>(std::clamp(color[2], 0.0f, 1.0f) * 255);
}

std::ostream &operator<<(std::ostream &os, const struct PPMColor &color) {
  os << color.red << " " << color.green << " " << color.blue;
  return os;
}

Material::Material() : albedo{0.82f, 0.82f, 0.82f}, type{Diffuse}, smoothShading{false} {};

Material::Material(const Albedo &albedo, const MaterialType &type, const bool smoothShading, const float ior)
    : albedo{albedo}, type(type), smoothShading(smoothShading), ior{ior} {};
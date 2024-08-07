#include "tracer/Utils.h"

#include <algorithm>

PPMColor::PPMColor(const Color &color) {
  this->red = static_cast<unsigned short>(std::clamp(0.0f, 1.0f, color[0]) * 255);
  this->green = static_cast<unsigned short>(std::clamp(0.0f, 1.0f, color[1]) * 255);
  this->blue = static_cast<unsigned short>(std::clamp(0.0f, 1.0f, color[2]) * 255);
}

std::ostream &operator<<(std::ostream &os, const struct PPMColor &color) {
  os << color.red << " " << color.green << " " << color.blue;
  return os;
}

Material::Material() : albedo{0.82f, 0.82f, 0.82f}, type{Diffuse}, smoothShading{false} {};

Material::Material(const Albedo &albedo, const MaterialType &type, const bool smoothShading)
    : albedo{albedo}, type(type), smoothShading(smoothShading){};
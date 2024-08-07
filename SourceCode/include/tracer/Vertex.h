#pragma once

#include "tracer/Vector.h"
class Vertex {
 private:
 public:
  Vector position;
  Vector normal;

#if (defined USE_TEXTURES) && USE_TEXTURES
  Vector UV;

 public:
  explicit Vertex(const Vector &position, const Vector &normal = Vector(), const Vector &UV = Vector());

#else
 public:
  explicit Vertex(const Vector &position, const Vector &normal = Vector());
#endif  // USE_TEXTURES

  float &operator[](const unsigned short index);

  const float &operator[](const unsigned short index) const;
};
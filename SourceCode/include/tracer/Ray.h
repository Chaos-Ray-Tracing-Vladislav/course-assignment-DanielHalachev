#pragma once

#include <optional>

#include "Vector.h"
#include "tracer/Triangle.h"
#include "tracer/Utils.h"

struct Intersection {
  Vector hitPoint;
  Vector hitNormal;

// Ugly, I know :-D
#if defined(BARYCENTRIC) && BARYCENTRIC
  float u;
  float v;
#endif  // BARYCENTRIC
};

enum RayType { Primary, Shadow, Reflection, Refraction };

class Ray {
 public:
  Vector origin;
  Vector direction;
  RayType rayType = Primary;

  Ray();
  Ray(const Vector &origin, const Vector &direction, const RayType &rayType = Primary);

  // TODO (maybe move to Triangle.h)
  std::optional<Intersection> intersectWithTriangle(const Triangle &triangle, const MaterialType = Diffuse,
                                                    const bool smoothShading = false) const;
};
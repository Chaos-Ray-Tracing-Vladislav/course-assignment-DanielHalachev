#pragma once

#include <optional>

#include "Vector.h"
#include "tracer/Triangle.h"

struct Intersection {
  Vector hitPoint;
  Vector hitNormal;

// Ugly, I know :-D
#if (defined(BARYCENTRIC) && BARYCENTRIC) || (defined(USE_TEXTURES) && USE_TEXTURES)
  float u;
  float v;
#endif  // BARYCENTRIC
};

enum RayType { PrimaryRay, ShadowRay, ReflectionRay, RefractionRay, DiffuseRay };

class Ray {
 public:
  Vector origin;
  Vector direction;
  RayType rayType = PrimaryRay;

  Ray();
  Ray(const Vector &origin, const Vector &direction, const RayType &rayType = PrimaryRay);

  // TODO (maybe move to Triangle.h)
  std::optional<Intersection> intersectWithTriangle(const Triangle &triangle, const bool smoothShading = false) const;
};
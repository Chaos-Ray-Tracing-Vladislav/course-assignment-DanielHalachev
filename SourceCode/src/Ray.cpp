#include <tracer/Ray.h>

// #include <limits>
#include <optional>

#include "tracer/Triangle.h"
#include "tracer/Vector.h"

Ray::Ray() = default;
Ray::Ray(const Vector &origin, const Vector &direction, const RayType &rayType)
    : origin(origin), direction(direction), rayType(rayType){};

std::optional<Intersection> Ray::intersectWithTriangle(const Triangle &triangle, const bool smoothShading) const {
  Vector triangleNormal = triangle.getTriangleNormal();
  float normalDotRayDirection = this->direction.dot(triangleNormal);

  if (this->rayType == Primary && normalDotRayDirection == 0) {
    return {};
  }

  float distanceToPlane = -(triangle[0].position).dot(triangleNormal);

  float t = -(triangleNormal.dot(this->origin) + distanceToPlane) / normalDotRayDirection;
  if (t < 0) {
    return {};
  }
  Vector intersectionPoint = this->origin + this->direction * t;

  if (!triangle.pointIsInTriangle(intersectionPoint)) {
    return {};
  }

  Vector hitNormal = triangleNormal;
  float u = 0;
  float v = 0;
  Vector v0p = intersectionPoint - triangle[0].position;
  Vector v0v1 = triangle[1].position - triangle[0].position;
  Vector v0v2 = triangle[2].position - triangle[0].position;
  float area = (v0v1 * v0v2).length();
  u = (v0p * v0v2).length() / area;
  v = (v0v1 * v0p).length() / area;
  if (smoothShading) {
    hitNormal = (triangle[1].normal * u + triangle[2].normal * v + triangle[0].normal * (1 - u - v));
  }

#if defined(BARYCENTRIC) && BARYCENTRIC
  return Intersection{intersectionPoint, hitNormal, u, v};
#endif  // BARYCENTRIC
  return Intersection{intersectionPoint, hitNormal};
}
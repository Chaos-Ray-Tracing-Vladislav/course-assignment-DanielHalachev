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

  // if N . R ~= 0, the ray is parallel to the plane - no intersection or too far away
  //   if (std::fabs(normalDotRayDirection) < std::numeric_limits<float>::epsilon()) {
  //     return {};
  //   }

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
  if (smoothShading) {
    Vector v0P = intersectionPoint - triangle[0].position;
    u = (v0P * (triangle[2].position - triangle[0].position)).length() / triangle.area();
    v = ((triangle[1].position - triangle[0].position) * v0P).length() / triangle.area();

    hitNormal = (triangle[1].normal * u + triangle[2].normal * v + triangle[0].normal * (1 - u - v));
    // hitNormal.normalize();
  }

#if defined(BARYCENTRIC) && BARYCENTRIC
  return Intersection{intersectionPoint, hitNormal, u, v};
#endif  // BARYCENTRIC
  return Intersection{intersectionPoint, hitNormal};
}
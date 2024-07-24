#include "tracer/Ray.h"

#include <optional>

Ray::Ray() = default;
Ray::Ray(const Vector &origin, const Vector &direction, const RayType &rayType)
    : origin(origin), direction(direction), rayType(rayType){};

std::optional<Intersection> Ray::intersectWithTriangle(const Triangle &triangle) const {
  Vector triangleNormal = triangle.getTriangleNormal();
  float normalDotRayDirection = this->direction.dot(triangleNormal);

  // material shouldn't be refractive, because refraction direction doesn't matter for intersection
  // its debatable whether it should be reflective
  if (this->rayType == PrimaryRay && normalDotRayDirection >= 0
      // std::fabs(normalDotRayDirection) < std::numeric_limits<float>::epsilon()  //
  ) {
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

  return Intersection{t, intersectionPoint, hitNormal};
}
#include <tracer/Ray.h>

// #include <limits>
#include <optional>

#include "tracer/Triangle.h"
#include "tracer/Vector.h"

#if 0

std::optional<Vector> Ray::intersectWithTriangle(const Triangle& triangle, const Vector& cameraPosition) const {
  Vector triangleNormal = triangle.getTriangleNormal();
  float normalDotRayDirection = this->direction.dot(triangleNormal);

  //   if (normalDotRayDirection >= 0) {
  //     return {};
  //   }

  // if N . R ~= 0, the ray is parallel to the plane - no intersection or too far away
  // if (std::fabs(normalDotRayDirection) < std::numeric_limits<float>::epsilon()) {
  //   return {};
  // }

  float distanceToPlane = (triangle[0] - cameraPosition).dot(triangleNormal);

  float t = distanceToPlane / normalDotRayDirection;

  Vector intersectionPoint = this->origin + t * this->direction;

  if (!triangle.pointIsInTriangle(intersectionPoint)) {
    return {};
  }

  return intersectionPoint;
}

std::optional<Vector> ShadowRay::intersectWithTriangle(const Triangle& triangle, const Vector& cameraPosition) const {
  Vector triangleNormal = triangle.getTriangleNormal();
  float normalDotRayDirection = this->direction.dot(triangleNormal);

  if (normalDotRayDirection == 0) {
    return {};
  }

  // if N . R ~= 0, the ray is parallel to the plane - no intersection or too far away
  // if (std::fabs(normalDotRayDirection) < std::numeric_limits<float>::epsilon()) {
  //   return {};
  // }

  float distanceToPlane = (triangle[0] - cameraPosition).dot(triangleNormal);

  float t = distanceToPlane / normalDotRayDirection;

  Vector intersectionPoint = this->origin + t * this->direction;

  if (!triangle.pointIsInTriangle(intersectionPoint)) {
    return {};
  }

  return intersectionPoint;
}

#else
template <>
std::optional<Intersection> Ray<Primary>::intersectWithTriangle(const Triangle& triangle,
                                                                const bool smoothShading) const {
  Vector triangleNormal = triangle.getTriangleNormal();
  float normalDotRayDirection = this->direction.dot(triangleNormal);

  if (normalDotRayDirection >= 0) {
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
  if (smoothShading) {
    Vector v0P = intersectionPoint - triangle[0].position;
    float u = (v0P * (triangle[2].position - triangle[0].position)).length() / triangle.area();
    float v = ((triangle[1].position - triangle[0].position) * v0P).length() / triangle.area();

    hitNormal = (triangle[1].normal * u + triangle[2].normal * v + triangle[0].normal * (1 - u - v));
    // hitNormal.normalize();
  }

  return Intersection{intersectionPoint, hitNormal};
}

template <>
std::optional<Intersection> Ray<Shadow>::intersectWithTriangle(const Triangle& triangle,
                                                               const bool smoothShading) const {
  Vector triangleNormal = triangle.getTriangleNormal();
  float normalDotRayDirection = this->direction.dot(triangleNormal);

  // if N . R ~= 0, the ray is parallel to the plane - no intersection or too far away
  // if (std::fabs(normalDotRayDirection) < std::numeric_limits<float>::epsilon()) {
  //   return {};
  // }

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
  if (smoothShading) {
    Vector v0P = intersectionPoint - triangle[0].position;
    float u = (v0P * (triangle[2].position - triangle[0].position)).length() / triangle.area();
    float v = ((triangle[1].position - triangle[0].position) * v0P).length() / triangle.area();

    hitNormal = (triangle[1].normal * u + triangle[2].normal * v + triangle[0].normal * (1 - u - v));
    // hitNormal.normalize();
  }

  return Intersection{intersectionPoint, hitNormal};
}
#endif
#include "tracer/Ray.h"

#include <optional>

Ray::Ray() = default;
Ray::Ray(const Vector &origin, const Vector &direction, const RayType &rayType)
    : origin(origin), direction(direction), rayType(rayType){};

std::optional<Intersection> Ray::intersectWithTriangle(const Triangle &triangle, const bool smoothShading) const {
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

// if we calculate barycentric colors or use textures, we need
// to calculate U and V and pass them to the structure
// otherwise, we only need to calculate U and V for smooth shading and we don't pass them
#if (defined(BARYCENTRIC) && BARYCENTRIC) || (defined(USE_TEXTURES) && USE_TEXTURES)
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
    hitNormal.normalize();
  }
  return Intersection{intersectionPoint, hitNormal, u, v};
#else
  if (smoothShading) {
    Vector v0p = intersectionPoint - triangle[0].position;
    Vector v0v1 = triangle[1].position - triangle[0].position;
    Vector v0v2 = triangle[2].position - triangle[0].position;
    float area = (v0v1 * v0v2).length();
    float u = (v0p * v0v2).length() / area;
    float v = (v0v1 * v0p).length() / area;
    hitNormal = (triangle[1].normal * u + triangle[2].normal * v + triangle[0].normal * (1 - u - v));
    hitNormal.normalize();
  }

  return Intersection{intersectionPoint, hitNormal};
#endif  // BARYCENTRIC || USE_TEXTURES
}
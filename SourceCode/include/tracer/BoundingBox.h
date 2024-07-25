#pragma once
#include <algorithm>
#include <limits>

#include "tracer/Scene.h"
#include "tracer/Triangle.h"
#include "tracer/Vector.h"

class BoundingBox {
 private:
  Vector minPoint;
  Vector maxPoint;

  void initializeMinMaxPoints() {
    minPoint =
        Vector(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    maxPoint = Vector(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest());
  }

 public:
  BoundingBox(const Vector &minPoint, const Vector &maxPoint) : minPoint{minPoint}, maxPoint{maxPoint} {}

  explicit BoundingBox(const std::vector<Triangle> &triangles) {
    initializeMinMaxPoints();
    for (auto &triangle : triangles) {
      for (auto *vertex : triangle.getVertices()) {
        for (auto i = 0; i < 3; i++) {
          this->minPoint[i] = std::min(this->minPoint[i], vertex->position[i]);
          this->maxPoint[i] = std::max(this->maxPoint[i], vertex->position[i]);
        }
      }
    }
  }

  explicit BoundingBox(const Scene &scene) {
    initializeMinMaxPoints();
    for (auto &object : scene.objects) {
      for (auto &triangle : object.triangles) {
        for (auto *vertex : triangle.getVertices()) {
          for (auto i = 0; i < 3; i++) {
            this->minPoint[i] = std::min(this->minPoint[i], vertex->position[i]);
            this->maxPoint[i] = std::max(this->maxPoint[i], vertex->position[i]);
          }
        }
      }
    }
  }

  explicit BoundingBox(const Triangle &triangle) {
    initializeMinMaxPoints();
    for (const auto *vertex : triangle.getVertices()) {
      for (int i = 0; i < 3; ++i) {
        minPoint[i] = std::min(minPoint[i], vertex->position[i]);
        maxPoint[i] = std::max(maxPoint[i], vertex->position[i]);
      }
    }
  }

  std::pair<BoundingBox, BoundingBox> split(const unsigned short axis) const {
    float middle = (this->maxPoint[axis] - this->minPoint[axis]) / 2;
    float splitPlaneCoordinate = this->minPoint[axis] + middle;
    BoundingBox first(*this);
    BoundingBox second(*this);

    first.maxPoint[axis] = splitPlaneCoordinate;
    second.minPoint[axis] = splitPlaneCoordinate;
    return {first, second};
  }

  bool intersects(const Triangle &triangle) const {
    return this->intersects(BoundingBox(triangle));
  }

  bool intersects(const BoundingBox &box) const {
    for (auto i = 0; i < 3; i++) {
      bool notOverlapI = (this->minPoint[i] > box.maxPoint[i]) || (this->maxPoint[i] < box.minPoint[i]);
      if (notOverlapI) {
        return false;
      }
    }
    return true;
  }

  bool hasIntersection(const Ray &ray) const {
    float t0 = std::numeric_limits<float>::lowest();
    float t1 = std::numeric_limits<float>::max();

    constexpr float epsilon = 1e-8;

    for (int i = 0; i < 3; ++i) {
      if (std::abs(ray.direction[i]) < epsilon) {
        if (ray.origin[i] < minPoint[i] || ray.origin[i] > maxPoint[i]) {
          return false;
        }
      } else {
        float invD = 1.0f / ray.direction[i];
        float tNear = (minPoint[i] - ray.origin[i]) * invD;
        float tFar = (maxPoint[i] - ray.origin[i]) * invD;

        if (tNear > tFar) std::swap(tNear, tFar);

        t0 = std::max(t0, tNear);
        t1 = std::min(t1, tFar);

        if (t0 > t1) return false;
      }
    }
    return true;
  }
};

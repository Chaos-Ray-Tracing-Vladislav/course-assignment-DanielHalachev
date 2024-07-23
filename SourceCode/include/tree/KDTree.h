#include <algorithm>
#include <limits>
#include <optional>
#include <stack>
#include <vector>

#include "tracer/Ray.h"
#include "tracer/Scene.h"
#include "tracer/Triangle.h"
#include "tracer/Vector.h"

class BoundingBox {
 private:
  Vector minPoint;
  Vector maxPoint;

 public:
  BoundingBox(const Vector &minPoint, const Vector &maxPoint) : minPoint{minPoint}, maxPoint{maxPoint} {}
  explicit BoundingBox(const Scene &scene) {
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
  std::pair<BoundingBox, BoundingBox> split(const unsigned short axis) const {
    float middle = (this->maxPoint[axis] - this->minPoint[axis]) / 2;
    float splitPlaneCoordinate = this->minPoint[axis] + middle;
    BoundingBox first(*this);
    BoundingBox second(*this);

    first.maxPoint[axis] = splitPlaneCoordinate;
    second.minPoint[axis] = splitPlaneCoordinate;
    return {first, second};
  }

  bool contains(const Triangle &triangle) {
    Vector minCoordinates(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                          std::numeric_limits<float>::max());
    Vector maxCoordinates(std::numeric_limits<float>::min(), std::numeric_limits<float>::min(),
                          std::numeric_limits<float>::min());
    for (auto &vertex : triangle.getVertices()) {
      for (auto i = 0; i < 3; i++) {
        maxCoordinates[i] = std::max(maxCoordinates[i], vertex->position[i]);
        minCoordinates[i] = std::min(minCoordinates[i], vertex->position[i]);
      }
    }
    for (auto i = 0; i < 3; i++) {
      if (minCoordinates[i] > this->maxPoint[i] || maxCoordinates[i] < this->minPoint[i]) {
        return false;
      }
    }
    return true;
  }

  bool hasIntersection(const Ray &ray) {
    Vector tMax;  // tXmin, tYmin, tZmin
    Vector tMin;  // tXmax, tYmax, tZmax
    for (auto i = 0; i < 3; i++) {
      if (ray.direction[i] == 0) {
        if (ray.origin[i] < minPoint[i] || ray.origin[i] > maxPoint[i]) {
          return false;
        }
        tMax[i] = std::numeric_limits<float>::infinity();
        tMin[i] = -std::numeric_limits<float>::infinity();
      } else {
        tMax[i] = (maxPoint[i] - ray.origin[i]) / ray.direction[i];
        tMin[i] = (minPoint[i] - ray.origin[i]) / ray.direction[i];
      }
    }

    for (auto i = 0; i < 3; i++) {
      if (tMin[i] > 0) {
        Vector intersectionPoint = ray.origin + tMin[i] * ray.direction;
        if (intersectionPoint[0] >= this->minPoint[0] && intersectionPoint[0] <= this->maxPoint[0] &&
            intersectionPoint[1] >= this->minPoint[1] && intersectionPoint[1] <= this->maxPoint[1] &&
            intersectionPoint[2] >= this->minPoint[2] && intersectionPoint[2] <= this->maxPoint[2]) {
          return true;
        }
      } else if (tMax[i] > 0) {
        Vector intersectionPoint = ray.origin + tMax[i] * ray.direction;
        if (intersectionPoint[0] >= this->minPoint[0] && intersectionPoint[0] <= this->maxPoint[0] &&
            intersectionPoint[1] >= this->minPoint[1] && intersectionPoint[1] <= this->maxPoint[1] &&
            intersectionPoint[2] >= this->minPoint[2] && intersectionPoint[2] <= this->maxPoint[2]) {
          return true;
        }
      }
    }
    return false;
  }
};

class KDTree {
 private:
  struct TreeNode {
    BoundingBox box;
    int children[2];
    int parent;
    std::vector<Triangle> triangles;

    std::optional<Intersection> intersect(const Ray &ray) {
      float minDistance = std::numeric_limits<float>::infinity();
      const Mesh *intersectedObject = nullptr;
      const Triangle *intersectedTriangle = nullptr;
      Intersection intersection;
      for (auto &triangle : this->triangles) {
        std::optional<Intersection> tempIntersection =
            ray.intersectWithTriangle(triangle, object.material.smoothShading);
        if (tempIntersection.has_value()) {
          float distance = (tempIntersection.value().hitPoint - ray.origin).length();
          if (distance < minDistance) {
            minDistance = distance;
            intersectedObject = &object;
            intersectedTriangle = &triangle;
            intersection = tempIntersection.value();
          }
        }
      }
      if (intersectedObject != nullptr) {
        //
#if (defined(BARYCENTRIC) && BARYCENTRIC) || (defined(USE_TEXTURES) && USE_TEXTURES)
        IntersectionInformation temp{intersectedObject,      intersectedTriangle, intersection.hitPoint,
                                     intersection.hitNormal, intersection.u,      intersection.v};
        return temp;
#endif  // BARYCENTRIC
        return IntersectionInformation{intersectedObject, intersectedTriangle, intersection.hitPoint,
                                       intersection.hitNormal};
      }
      return {};
    }
  };

  const unsigned short MAX_TRIANGLES_IN_LEAF = 16;
  const unsigned short MAX_DEPTH = 25;
  const unsigned short AXIS_COUNT = 3;
  std::vector<TreeNode> nodes;

  unsigned int createNode(const BoundingBox &box, int firstChildIndex, int secondChildIndex, int parentIndex) {
    unsigned int newNodeIndex = this->nodes.size();
    this->nodes.push_back(TreeNode{box, {firstChildIndex, secondChildIndex}, parentIndex, {}});
    return newNodeIndex;
  }

  void build(const int parentIndex, const unsigned short depth, const std::vector<Triangle> &triangles) {
    if (depth >= MAX_DEPTH || triangles.size() <= MAX_TRIANGLES_IN_LEAF) {
      this->nodes[parentIndex].triangles = triangles;
      return;
    }

    std::pair<BoundingBox, BoundingBox> childBoxes = this->nodes[parentIndex].box.split(depth % AXIS_COUNT);
    std::vector<Triangle> firstChildTriangles;
    std::vector<Triangle> secondChildTriangles;
    firstChildTriangles.reserve(triangles.size() / 2);
    firstChildTriangles.reserve(triangles.size() / 2);

    for (auto &triangle : triangles) {
      if (childBoxes.first.contains(triangle)) {
        firstChildTriangles.push_back(triangle);
      }
      if (childBoxes.second.contains(triangle)) {
        secondChildTriangles.push_back(triangle);
      }
    }
    firstChildTriangles.shrink_to_fit();
    secondChildTriangles.shrink_to_fit();

    if (!firstChildTriangles.empty()) {
      this->nodes[parentIndex].children[0] = this->createNode(childBoxes.first, -1, -1, parentIndex);
      build(this->nodes[parentIndex].children[0], depth + 1, firstChildTriangles);
    }

    if (!secondChildTriangles.empty()) {
      this->nodes[parentIndex].children[1] = this->createNode(childBoxes.second, -1, -1, parentIndex);
      build(this->nodes[parentIndex].children[1], depth + 1, secondChildTriangles);
    }
  }

  std::optional<Intersection> intersect(const Ray &ray) {
    std::vector<Intersection> intersections;
    std::stack<unsigned int> indexesToCheck;
    indexesToCheck.push(0);
    while (!indexesToCheck.empty()) {
      unsigned int currentIndex = indexesToCheck.top();
      TreeNode &currentNode = this->nodes[currentIndex];
      indexesToCheck.pop();
      if (currentNode.box.hasIntersection(ray)) {
        if (!currentNode.triangles.empty()) {
          std::optional<Intersection> currentIntersection = currentNode.intersect(ray);
          if (currentIntersection.has_value()) {
            intersections.push_back(currentIntersection.value());
          }
        } else {
          if (currentNode.children[0] != -1) {
            indexesToCheck.push(currentNode.children[0]);
          }
          if (currentNode.children[1]) {
            indexesToCheck.push(currentNode.children[1]);
          }
        }
      }
    }
    Intersection closestIntersection;
    float minDistance = std::numeric_limits<float>::infinity();
    for (auto &intersection : intersections) {
      if (intersection.distance < minDistance) {
        minDistance = intersection.distance;
        closestIntersection = intersection;
      }
    }
    if (minDistance == std::numeric_limits<float>::infinity()) {
      return {};
    }
    return closestIntersection;
  }

 public:
  explicit KDTree(const Scene &scene) {
    unsigned int rootIndex = this->createNode(BoundingBox(scene), -1, -1, -1);
    build(rootIndex, 0, );
  }
};
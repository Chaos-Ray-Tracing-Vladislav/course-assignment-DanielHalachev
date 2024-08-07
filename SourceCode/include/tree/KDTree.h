#include <algorithm>
#include <cstddef>
#include <limits>
#include <numeric>
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
    for (auto &triangle : scene.triangles) {
      for (auto *vertex : triangle.getVertices()) {
        for (auto i = 0; i < 3; i++) {
          this->minPoint[i] = std::min(this->minPoint[i], vertex->position[i]);
          this->maxPoint[i] = std::max(this->maxPoint[i], vertex->position[i]);
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
    std::vector<size_t> triangleIndexes;
  };

  const Scene &scene;
  const unsigned short MAX_TRIANGLES_IN_LEAF = 16;
  const unsigned short MAX_DEPTH = 25;
  const unsigned short AXIS_COUNT = 3;
  std::vector<TreeNode> nodes;

  unsigned int createNode(const BoundingBox &box, int firstChildIndex, int secondChildIndex, int parentIndex) {
    unsigned int newNodeIndex = this->nodes.size();
    this->nodes.push_back(TreeNode{box, {firstChildIndex, secondChildIndex}, parentIndex, {}});
    return newNodeIndex;
  }

  void build(const int parentIndex, const unsigned short depth, const std::vector<size_t> &triangleIndexes) {
    if (depth >= MAX_DEPTH || triangleIndexes.size() <= MAX_TRIANGLES_IN_LEAF) {
      this->nodes[parentIndex].triangleIndexes = triangleIndexes;
      return;
    }

    std::pair<BoundingBox, BoundingBox> childBoxes = this->nodes[parentIndex].box.split(depth % AXIS_COUNT);
    std::vector<size_t> firstChildTriangles;
    std::vector<size_t> secondChildTriangles;
    firstChildTriangles.reserve(triangleIndexes.size() / 2);
    firstChildTriangles.reserve(triangleIndexes.size() / 2);

    for (auto &triangleIndex : triangleIndexes) {
      if (childBoxes.first.contains(this->scene.triangles[triangleIndex])) {
        firstChildTriangles.push_back(triangleIndex);
      }
      if (childBoxes.second.contains(this->scene.triangles[triangleIndex])) {
        secondChildTriangles.push_back(triangleIndex);
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

  std::optional<IntersectionInformation> intersect(const Ray &ray) {
    std::vector<IntersectionInformation> intersections;
    std::stack<unsigned int> indexesToCheck;
    indexesToCheck.push(0);
    while (!indexesToCheck.empty()) {
      unsigned int currentIndex = indexesToCheck.top();
      TreeNode &currentNode = this->nodes[currentIndex];
      indexesToCheck.pop();
      if (currentNode.box.hasIntersection(ray)) {
        if (!currentNode.triangleIndexes.empty()) {
          std::optional<IntersectionInformation> currentIntersection = scene.trace(ray, currentNode.triangleIndexes);
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
    IntersectionInformation closestIntersection;
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
  explicit KDTree(const Scene &scene) : scene(scene) {
    unsigned int rootIndex = this->createNode(BoundingBox(scene), -1, -1, -1);
    std::vector<size_t> indexes(this->scene.triangles.size());
    std::iota(indexes.begin(), indexes.end(), 0);
    build(rootIndex, 0, indexes);
  }
};
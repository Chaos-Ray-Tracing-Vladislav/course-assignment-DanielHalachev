#include "tree/KDTree.h"

#include <stack>

#include "tracer/Ray.h"
#include "tree/AccelerationStructure.h"

template <>
void KDTree<Triangle>::build(const unsigned int parentIndex, const unsigned short depth,
                             const std::vector<size_t> &elementIndexes) {
  if (depth >= MAX_DEPTH || elementIndexes.size() <= MAX_ELEMENTS_IN_LEAF) {
    this->nodes[parentIndex].indexes = elementIndexes;
    return;
  }

  std::pair<BoundingBox, BoundingBox> childBoxes = this->nodes[parentIndex].box.split(depth % AXIS_COUNT);
  std::vector<size_t> firstChildElements;
  std::vector<size_t> secondChildElements;
  firstChildElements.reserve(elementIndexes.size() / 2);
  secondChildElements.reserve(elementIndexes.size() / 2);

  for (auto elementIndex : elementIndexes) {
    if (childBoxes.first.intersects(this->container->at(elementIndex))) {
      firstChildElements.push_back(elementIndex);
    }
    if (childBoxes.second.intersects(this->container->at(elementIndex))) {
      secondChildElements.push_back(elementIndex);
    }
  }
  firstChildElements.shrink_to_fit();
  secondChildElements.shrink_to_fit();

  if (!firstChildElements.empty()) {
    this->nodes[parentIndex].children[0] =
        this->createNode(childBoxes.first, INVALID_INDEX, INVALID_INDEX, parentIndex);
    build(this->nodes[parentIndex].children[0], depth + 1, firstChildElements);
  }

  if (!secondChildElements.empty()) {
    this->nodes[parentIndex].children[1] =
        this->createNode(childBoxes.second, INVALID_INDEX, INVALID_INDEX, parentIndex);
    build(this->nodes[parentIndex].children[1], depth + 1, secondChildElements);
  }
}

template <>
std::optional<IntersectionInformation> KDTree<Triangle>::intersect(const Ray &ray) const {
  std::vector<IntersectionInformation> intersections;
  std::stack<unsigned int> indexesToCheck;
  indexesToCheck.push(0);
  while (!indexesToCheck.empty()) {
    unsigned int currentIndex = indexesToCheck.top();
    const TreeNode &currentNode = this->nodes[currentIndex];
    indexesToCheck.pop();
    if (currentNode.box.hasIntersection(ray)) {
      if (!currentNode.indexes.empty()) {
        for (auto triangleIndex : currentNode.indexes) {
          std::optional<Intersection> intersection = ray.intersectWithTriangle(this->container->at(triangleIndex));
          if (intersection.has_value()) {
            intersections.emplace_back(nullptr, &this->container->at(triangleIndex), intersection.value());
          }
        }
      } else {
        if (currentNode.children[0] != INVALID_INDEX) {
          indexesToCheck.push(currentNode.children[0]);
        }
        if (currentNode.children[1] != INVALID_INDEX) {
          indexesToCheck.push(currentNode.children[1]);
        }
      }
    }
  }
  if (intersections.empty()) {
    return {};
  }
  IntersectionInformation closestIntersection;
  float minDistance = std::numeric_limits<float>::infinity();
  for (auto &intersectionInformation : intersections) {
    if (intersectionInformation.intersection.distance < minDistance) {
      minDistance = intersectionInformation.intersection.distance;
      closestIntersection = intersectionInformation;
    }
  }
  return closestIntersection;
}

template <>
void KDTree<ObjectKDTreeSubTree>::build(const unsigned int parentIndex, const unsigned short depth,
                                        const std::vector<size_t> &elementIndexes) {
  if (depth >= MAX_DEPTH || elementIndexes.size() <= MAX_ELEMENTS_IN_LEAF) {
    this->nodes[parentIndex].indexes = elementIndexes;
    return;
  }

  std::pair<BoundingBox, BoundingBox> childBoxes = this->nodes[parentIndex].box.split(depth % AXIS_COUNT);
  std::vector<size_t> firstChildElements;
  std::vector<size_t> secondChildElements;
  firstChildElements.reserve(elementIndexes.size() / 2);
  secondChildElements.reserve(elementIndexes.size() / 2);

  for (auto &elementIndex : elementIndexes) {
    if (childBoxes.first.intersects(this->container->at(elementIndex).getBoundingBox())) {
      firstChildElements.push_back(elementIndex);
    }
    if (childBoxes.second.intersects(this->container->at(elementIndex).getBoundingBox())) {
      secondChildElements.push_back(elementIndex);
    }
  }
  firstChildElements.shrink_to_fit();
  secondChildElements.shrink_to_fit();

  if (!firstChildElements.empty()) {
    this->nodes[parentIndex].children[0] =
        this->createNode(childBoxes.first, INVALID_INDEX, INVALID_INDEX, parentIndex);
    build(this->nodes[parentIndex].children[0], depth + 1, firstChildElements);
  }

  if (!secondChildElements.empty()) {
    this->nodes[parentIndex].children[1] =
        this->createNode(childBoxes.second, INVALID_INDEX, INVALID_INDEX, parentIndex);
    build(this->nodes[parentIndex].children[1], depth + 1, secondChildElements);
  }
}

template <>
std::optional<IntersectionInformation> KDTree<ObjectKDTreeSubTree>::intersect(const Ray &ray) const {
  std::vector<IntersectionInformation> intersections;
  std::stack<unsigned int> indexesToCheck;
  indexesToCheck.push(0);
  while (!indexesToCheck.empty()) {
    unsigned int currentIndex = indexesToCheck.top();
    const TreeNode &currentNode = this->nodes[currentIndex];
    indexesToCheck.pop();
    if (currentNode.box.hasIntersection(ray)) {
      if (!currentNode.indexes.empty()) {
        for (auto subTreeIndex : currentNode.indexes) {
          std::optional<IntersectionInformation> intersection = this->container->at(subTreeIndex).tree.intersect(ray);
          if (intersection.has_value()) {
            IntersectionInformation info = intersection.value();
            info.object = &this->container->at(subTreeIndex).mesh;
            intersections.push_back(info);
          }
        }
      } else {
        if (currentNode.children[0] != INVALID_INDEX) {
          indexesToCheck.push(currentNode.children[0]);
        }
        if (currentNode.children[1] != INVALID_INDEX) {
          indexesToCheck.push(currentNode.children[1]);
        }
      }
    }
  }
  if (intersections.empty()) {
    return {};
  }
  IntersectionInformation closestIntersection;
  float minDistance = std::numeric_limits<float>::infinity();
  for (auto &intersectionInformation : intersections) {
    if (intersectionInformation.intersection.distance < minDistance) {
      minDistance = intersectionInformation.intersection.distance;
      closestIntersection = intersectionInformation;
    }
  }
  const Mesh *intersectedObject = closestIntersection.object;
  const Triangle *intersectedTriangle = closestIntersection.triangle;
  bool calculateUV = intersectedObject->material.smoothShading;
  float u = 0;
  float v = 0;
#if (defined(BARYCENTRIC) && BARYCENTRIC) || (defined(USE_TEXTURES) && USE_TEXTURES)
  calculateUV = true;
#endif  // BARYCENTRIC || USE_TEXTURES
  if (calculateUV) {
    std::pair<float, float> UV =
        intersectedTriangle->getBarycentricCoordinates(closestIntersection.intersection.hitPoint);
    u = UV.first;
    v = UV.second;
    if (intersectedObject->material.smoothShading) {
      closestIntersection.intersection.hitNormal =
          (intersectedTriangle[1].getTriangleNormal() * u + intersectedTriangle[2].getTriangleNormal() * v +
           intersectedTriangle[0].getTriangleNormal() * (1 - u - v));
      closestIntersection.intersection.hitNormal.normalize();
    }
  }
#if (defined(BARYCENTRIC) && BARYCENTRIC) || (defined(USE_TEXTURES) && USE_TEXTURES)
  closestIntersection.u = u;
  closestIntersection.v = v;
#endif  // BARYCENTRIC || USE_TEXTURES
  return closestIntersection;
}
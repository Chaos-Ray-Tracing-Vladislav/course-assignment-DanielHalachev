#include "tree/AccelerationStructure.h"

#include <chrono>
#include <iostream>
#include <mutex>
#include <numeric>
#include <stack>
#include <thread>

#include "threadpool/ThreadManager.h"

TriangleKDTree::TriangleKDTree(const Mesh &mesh, const unsigned short maxDepth, const unsigned short maxElementsInLeaf)
    : KDTree<Triangle>(maxDepth, maxElementsInLeaf) {
  this->container = &mesh.triangles;
  unsigned int rootIndex = this->createNode(BoundingBox(mesh.triangles), INVALID_INDEX, INVALID_INDEX, INVALID_INDEX);
  std::vector<size_t> indexes(mesh.triangles.size());
  std::iota(indexes.begin(), indexes.end(), 0);
  build(rootIndex, 0, indexes);
}

BoundingBox ObjectKDTreeSubTree::getBoundingBox() const {
  return this->tree.getBoundingBox();
}

ObjectKDTree::ObjectKDTree() : KDTree<ObjectKDTreeSubTree>(){};

ObjectKDTree::ObjectKDTree(const Scene &scene, const unsigned short maxDepth, const unsigned short maxElementsInLeaf)
    : KDTree<ObjectKDTreeSubTree>(maxDepth, maxElementsInLeaf) {
  // auto startTime = std::chrono::high_resolution_clock::now();
  std::vector<ObjectKDTreeSubTree> *subTrees = new std::vector<ObjectKDTreeSubTree>;
  subTrees->reserve(scene.objects.size());
  // ThreadManager manager(std::thread::hardware_concurrency());
  // std::mutex mutex;
  for (auto &object : scene.objects) {
    // manager.doJob([subTrees, &object, &mutex]() {
    ObjectKDTreeSubTree subTree(object);
    // std::lock_guard<std::mutex> lock(mutex);
    subTrees->push_back(subTree);
    // });
  }
  // manager.waitForAll();
  this->container = subTrees;
  unsigned int rootIndex = this->createNode(BoundingBox(scene), INVALID_INDEX, INVALID_INDEX, INVALID_INDEX);
  std::vector<size_t> indexes(scene.objects.size());
  std::iota(indexes.begin(), indexes.end(), 0);
  build(rootIndex, 0, indexes);
  // auto endTime = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> duration = endTime - startTime;
  // std::cout << duration.count() << "s\n";
}

ObjectKDTree::~ObjectKDTree() {
  delete this->container;
}

bool ObjectKDTree::checkForIntersection(const Ray &ray, const float distanceToLight, const bool useGI) const {
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
          if (!useGI) {
            if (ray.rayType == ShadowRay && this->container->at(subTreeIndex).mesh.material.type == Refractive) {
              continue;
            }
          }
          std::optional<IntersectionInformation> intersection = this->container->at(subTreeIndex).tree.intersect(ray);
          if (intersection.has_value() &&
              (intersection->intersection.hitPoint - ray.origin).length() <= distanceToLight) {
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
    return false;
  }
  return true;
}
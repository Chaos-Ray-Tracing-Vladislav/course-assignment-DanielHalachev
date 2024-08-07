#pragma once

#include "tracer/Triangle.h"
#include "tree/KDTree.h"

class TriangleKDTree : public KDTree<Triangle> {
  using KDTree<Triangle>::KDTree;

 public:
  explicit TriangleKDTree(Mesh &mesh, const unsigned short maxDepth = 25,
                          const unsigned short maxElementsInLeaf = 8);
};

struct ObjectKDTreeSubTree {
  const Mesh &mesh;
  TriangleKDTree tree;
  explicit ObjectKDTreeSubTree(Mesh &mesh) : mesh(mesh), tree(mesh) {}
  BoundingBox getBoundingBox() const;
};

class ObjectKDTree : public KDTree<ObjectKDTreeSubTree> {
 public:
  ObjectKDTree();

  explicit ObjectKDTree(Scene &scene, const unsigned short maxDepth = 25, const unsigned short maxElementsInLeaf = 4);

  ~ObjectKDTree();

  bool checkForIntersection(const Ray &ray, const float distanceToLight) const;
};

typedef ObjectKDTree AccelerationStructure;
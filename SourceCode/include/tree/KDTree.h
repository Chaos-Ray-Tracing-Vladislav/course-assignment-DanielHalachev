#pragma once
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "tracer/BoundingBox.h"
#include "tracer/Ray.h"
#include "tracer/Scene.h"

const unsigned int INVALID_INDEX = std::numeric_limits<unsigned int>::max();

template <typename LeafType>
class KDTree {
 protected:
  struct TreeNode {
    BoundingBox box;
    unsigned int children[2];
    unsigned int parent;
    std::vector<size_t> indexes;

    std::optional<IntersectionInformation> intersect(const Ray &ray) const;
  };

  bool requireBuild = true;
  const unsigned short MAX_ELEMENTS_IN_LEAF;
  const unsigned short MAX_DEPTH;
  const unsigned short AXIS_COUNT = 3;
  std::vector<TreeNode> nodes;
  std::vector<LeafType> *container;

 protected:
  unsigned int createNode(const BoundingBox &box, unsigned int firstChildIndex, unsigned int secondChildIndex,
                          unsigned int parentIndex) {
    unsigned int newNodeIndex = this->nodes.size();
    this->nodes.push_back(TreeNode{box, {firstChildIndex, secondChildIndex}, parentIndex, {}});
    return newNodeIndex;
  }

  void build(const unsigned int parentIndex, const unsigned short depth, const std::vector<size_t> &elementIndexes);

 public:
  explicit KDTree(const unsigned short maxDepth = 20, unsigned short maxElementsInLeaf = 8)
      : MAX_ELEMENTS_IN_LEAF(maxElementsInLeaf), MAX_DEPTH(maxDepth), container(nullptr){};

  std::optional<IntersectionInformation> intersect(const Ray &ray) const;
  BoundingBox getBoundingBox() const {
    return this->nodes.at(0).box;
  }
};
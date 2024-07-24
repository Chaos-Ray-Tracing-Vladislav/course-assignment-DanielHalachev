#include "tracer/Scene.h"

#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

Mesh::Mesh(const Material &material, const std::vector<Vertex> &vertices, const size_t beginIterator,
           const size_t endIterator)
    : material{material}, vertices{vertices}, beginIncluded{beginIterator}, endExcluded{endIterator} {}

Mesh::Mesh(Mesh &&other) noexcept
    : material(other.material),
      vertices{std::move(other.vertices)},
      beginIncluded{other.beginIncluded},
      endExcluded{other.endExcluded} {}

Mesh::Mesh(const Mesh &other)
    : material(other.material),
      vertices{other.vertices},
      beginIncluded{other.beginIncluded},
      endExcluded{other.endExcluded} {}

bool Mesh::containsTriangle(const size_t triangleIndex) const {
  return (this->beginIncluded <= triangleIndex && triangleIndex < this->endExcluded);
}

Scene::Scene() : lastObjectIndex{0} {};

Scene::Scene(Scene &&other) noexcept {
  this->sceneSettings = other.sceneSettings;
  this->camera = other.camera;
#if (defined USE_TEXTURES) && USE_TEXTURES
  this->textures = std::move(other.textures);
#endif  // USE_TEXTURES
  this->materials = std::move(other.materials);
  this->lights = std::move(other.lights);
  this->objects = std::move(other.objects);
  this->triangles = std::move(other.triangles);
  this->lastObjectIndex = other.lastObjectIndex.load();
}

Scene &Scene::operator=(Scene &&other) noexcept {
  if (this != &other) {
    this->sceneSettings = other.sceneSettings;
    this->camera = other.camera;
#if (defined USE_TEXTURES) && USE_TEXTURES
    this->textures = std::move(other.textures);
#endif  // USE_TEXTURES
    this->materials = std::move(other.materials);
    this->lights = std::move(other.lights);
    this->objects = std::move(other.objects);
    this->triangles = std::move(other.triangles);
    this->lastObjectIndex = other.lastObjectIndex.load();
  }
  return *this;
}

Scene::~Scene() {
#if (defined USE_TEXTURES) && USE_TEXTURES
  for (Texture *texture : this->textures) {
    delete texture;
  }
#endif  // USE_TEXTURES
}

std::optional<IntersectionInformation> Scene::trace(const Ray &ray, const std::vector<size_t> &triangleIndexes) const {
  float minDistance = std::numeric_limits<float>::infinity();
  size_t intersectedTriangleIndex = -1;  // technically legit, because -1 = (0xffffffffffffffff) :-D
  Intersection intersection;

  for (auto triangleIndex : triangleIndexes) {
    std::optional<Intersection> tempIntersection = ray.intersectWithTriangle(this->triangles[triangleIndex]);
    if (tempIntersection.has_value()) {
      float distance = (tempIntersection.value().hitPoint - ray.origin).length();
      if (distance < minDistance) {
        minDistance = distance;
        intersectedTriangleIndex = triangleIndex;
        intersection = tempIntersection.value();
      }
    }
  }

  if (intersectedTriangleIndex != -1) {
#if (defined(BARYCENTRIC) && BARYCENTRIC) || (defined(USE_TEXTURES) && USE_TEXTURES)
    IntersectionInformation temp{intersectedTriangleIndex, intersection.distance, intersection.hitPoint,
                                 intersection.hitNormal,   intersection.u,        intersection.v};
    return temp;
#endif  // BARYCENTRIC
    return IntersectionInformation(intersectedTriangleIndex, nullptr, intersection.distance, intersection.hitPoint,
                                   intersection.hitNormal);
  }
  return {};
}

size_t Scene::binarySearch(const size_t leftIncluded, const size_t rightExcluded, const size_t triangleIndex) const {
  if (leftIncluded >= this->objects.size()) {
    return std::numeric_limits<size_t>::infinity();
  }
  if (leftIncluded > rightExcluded) {
    return std::numeric_limits<size_t>::infinity();
  }
  if (leftIncluded == rightExcluded) {
    return leftIncluded;
  }
  size_t middle = leftIncluded + (rightExcluded - leftIncluded) / 2;
  if (triangleIndex < this->objects[middle].beginIncluded) {
    return binarySearch(leftIncluded, middle, triangleIndex);
  }
  if (triangleIndex >= this->objects[middle].endExcluded) {
    return binarySearch(this->objects[middle].endExcluded, rightExcluded, triangleIndex);
  }
  return middle;
}

size_t Scene::getObject(const size_t triangleIndex) {
  // triangles are sequential and grouped by objects
  // when we iterate over them, it's likely that the desired object is next to the last one
  // size_t lastObjectIndexLocal = this->lastObjectIndex.load();
  // if (triangleIndex < this->objects[lastObjectIndexLocal].beginIncluded) {
  //   if (lastObjectIndexLocal > 0 && this->objects[lastObjectIndexLocal - 1].containsTriangle(triangleIndex)) {
  //     this->lastObjectIndex--;
  //   } else {
  //     this->lastObjectIndex = binarySearch(0, this->objects.size(), triangleIndex);
  //   }
  // } else if (this->objects[lastObjectIndexLocal].endExcluded <= triangleIndex) {
  //   if (lastObjectIndexLocal < this->objects.size() - 1 &&
  //       this->objects[lastObjectIndexLocal + 1].containsTriangle(triangleIndex)) {
  //     this->lastObjectIndex++;
  //   } else {
  //     this->lastObjectIndex = binarySearch(0, this->objects.size(), triangleIndex);
  //   }
  // }
  // return lastObjectIndexLocal;
  // return binarySearch(0, this->objects.size(), triangleIndex);
  for (auto objectIndex = 0; objectIndex < this->objects.size(); objectIndex++) {
    if (this->objects[objectIndex].containsTriangle(triangleIndex)) {
      return objectIndex;
    }
  }
}
#pragma once
#include <cstddef>
#include <optional>
#include <vector>

#include "tracer/Camera.h"
#include "tracer/Material.h"
#include "tracer/Ray.h"
#include "tracer/Triangle.h"

struct Image {
  unsigned int width;
  unsigned int height;
};

struct SceneSettings {
  Color sceneBackgroundColor;
  Image image;
  unsigned int bucketSize;
};

struct Light {
  Vector position;
  unsigned int intentsity;
};

class Mesh {
 public:
  const Material &material;
  std::vector<Vertex> vertices;
  size_t beginIncluded;
  size_t endExcluded;

 public:
  Mesh(const Material &material, const std::vector<Vertex> &vertices, const size_t beginIterator,
       const size_t endIterator);
  Mesh(const Mesh &other);
  Mesh(Mesh &&other) noexcept;

  Mesh &operator=(const Mesh &other) = delete;
  Mesh &operator=(Mesh &&other) noexcept = delete;
  bool containsTriangle(const size_t triangleIndex) const;
};

struct IntersectionInformation {
  size_t triangleIndex;
  float distance;
  Vector hitPoint;
  Vector hitNormal;
#if (defined(BARYCENTRIC) && BARYCENTRIC) || (defined(USE_TEXTURES) && USE_TEXTURES)
  float u;
  float v;
#endif  // BARYCENTRIC
};

struct Scene {
  SceneSettings sceneSettings;
  Camera camera;
#if (defined USE_TEXTURES) && USE_TEXTURES
  std::vector<Texture *> textures;
#endif  // USE_TEXTUTES
  std::vector<Material> materials;
  std::vector<Light> lights;
  std::vector<Mesh> objects;
  std::vector<Triangle> triangles;
  size_t lastObjectIndex;

 private:
  size_t binarySearch(const size_t leftIncluded, const size_t rightExcluded, const size_t triangleIndex) const;

 public:
  Scene();

  Scene(Scene &&other) noexcept;
  Scene &operator=(Scene &&other) noexcept;

  Scene(const Scene &other) = delete;
  Scene &operator=(const Scene &other) = delete;

  ~Scene();

  const Mesh &getObject(const size_t triangleIndex);
  std::optional<IntersectionInformation> trace(const Ray &ray, const std::vector<size_t> &triangleIndexes) const;
};
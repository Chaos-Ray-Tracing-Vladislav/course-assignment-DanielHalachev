#pragma once
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
  unsigned int bucketSize = 1;
};

struct Light {
  Vector position;
  unsigned int intentsity;
};

class Mesh {
 public:
  const Material &material;
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indexes;
  std::vector<Triangle> triangles;

 public:
  Mesh(const Material &material, const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indexes);
  Mesh(const Mesh &other);
  Mesh(Mesh &&other) noexcept;

  Mesh &operator=(const Mesh &other) = delete;
  Mesh &operator=(Mesh &&other) = delete;
};

struct IntersectionInformation {
  const Mesh *object;
  const Triangle *triangle;
  Intersection intersection;
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

  Scene();

  Scene(Scene &&other) noexcept;
  Scene &operator=(Scene &&other) noexcept;

  Scene(const Scene &other) = delete;
  Scene &operator=(const Scene &other) = delete;

  ~Scene();
};
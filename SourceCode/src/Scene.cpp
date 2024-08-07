#include "tracer/Scene.h"

Mesh::Mesh(const Material &material, const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indexes)
    : material(material), vertices{vertices}, indexes{indexes} {
  triangles.reserve(indexes.size() / 3);
  // calculate vertex normals
  // and add triangles
  for (auto i = 0; i < indexes.size(); i += 3) {
    unsigned int index0 = this->indexes[i];
    unsigned int index1 = this->indexes[i + 1];
    unsigned int index2 = this->indexes[i + 2];

    Vertex &v0 = this->vertices[index0];
    Vertex &v1 = this->vertices[index1];
    Vertex &v2 = this->vertices[index2];

    Triangle tr(v0, v1, v2);
    triangles.push_back(tr);
    Vector faceNormal = tr.getTriangleNormal();

    this->vertices[index0].normal += faceNormal;
    this->vertices[index1].normal += faceNormal;
    this->vertices[index2].normal += faceNormal;
  }
  for (auto &vertex : this->vertices) {
    vertex.normal.normalize();
  }
}

Mesh::Mesh(Mesh &&other) noexcept {
  this->material = other.material;
  this->vertices = std::move(other.vertices);
  this->indexes = std::move(other.indexes);
  this->triangles = std::move(other.triangles);
}

Scene::Scene() = default;

Scene::Scene(Scene &&other) noexcept {
  this->sceneSettings = other.sceneSettings;
  this->camera = other.camera;
#if (defined USE_TEXTURES) && USE_TEXTURES
  this->textures = std::move(other.textures);
#endif  // USE_TEXTURES
  this->materials = std::move(other.materials);
  this->lights = std::move(other.lights);
  this->objects = std::move(other.objects);
}

Scene &Scene::operator=(Scene &&other) noexcept {
  if (this != &other) {
    this->sceneSettings = other.sceneSettings;
    this->camera = other.camera;
#if (defined USE_TEXTURES) && USE_TEXTURES
    this->textures = std::move(other.textures);
#endif  // USE_TEXTURES    this->materials = std::move(other.materials);
    this->lights = std::move(other.lights);
    this->objects = std::move(other.objects);
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
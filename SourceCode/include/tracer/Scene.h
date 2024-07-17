#pragma once
#include <utility>
#include <vector>

#include "tracer/Camera.h"
#include "tracer/Triangle.h"
#include "tracer/Utils.h"
#include "tracer/Vector.h"

struct Triangle;
struct Light;

struct Image {
  unsigned int width;
  unsigned int height;
};

struct SceneSettings {
  Color sceneBackgroundColor;
  Image image;
};

struct Light {
  Vector position;
  unsigned int intentsity;
};

struct Mesh {
  Material material;
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indexes;
  std::vector<Triangle> triangles;
  Mesh(const Material &material, const std::vector<Vertex> &vertices, const std::vector<unsigned int> &indexes)
      : material(material), vertices(vertices), indexes(indexes) {
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
      Vector faceNormal = tr.calculateNormal();

      this->vertices[index0].normal += faceNormal;
      this->vertices[index1].normal += faceNormal;
      this->vertices[index2].normal += faceNormal;
    }
    for (auto &vertex : this->vertices) {
      vertex.normal.normalize();
    }
  }

  Mesh(const Mesh &other) = delete;

  Mesh(Mesh &&other) noexcept {
    this->material = other.material;
    this->vertices = std::move(other.vertices);
    this->indexes = std::move(other.indexes);
    this->triangles = std::move(other.triangles);
  }

  Mesh &operator=(const Mesh &other) = delete;
  Mesh &operator=(Mesh &&other) = delete;
};

struct Scene {
  SceneSettings sceneSettings;
  Camera camera;
  std::vector<Material> materials;
  std::vector<Light> lights;
  std::vector<Mesh> objects;
};
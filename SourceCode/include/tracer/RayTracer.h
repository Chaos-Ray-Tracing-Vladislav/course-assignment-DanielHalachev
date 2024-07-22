#pragma once
#include <random>
#include <string>
#include <vector>

#include "tracer/Ray.h"
#include "tracer/Scene.h"
#include "tracer/Triangle.h"
#include "tracer/Vector.h"

typedef Vector ColorVector;
#if defined(GLOBAL_ILLUMINATION) && GLOBAL_ILLUMINATION
const int MAX_DEPTH = 3;
const unsigned int DEFAULT_SAMPLE_SIZE = 4;
#else
const int MAX_DEPTH = 5;
#endif  // GLOBAL_ILLUMINATION
const float SHADOW_BIAS = 1e-4;
const float REFLECTION_BIAS = 1e-4;
const float REFRACTION_BIAS = 1e-4;
const float MONTE_CARLO_BIAS = 1e-4;

class RayTracer {
 private:
  static thread_local std::default_random_engine engine;
  static thread_local std::uniform_real_distribution<float> distribution;
  struct IntersectionInformation {
    const Mesh *const object;
    const Triangle *const triangle;
    Vector intersectionPoint;
    Vector hitNormal;
#if (defined(BARYCENTRIC) && BARYCENTRIC) || (defined(USE_TEXTURES) && USE_TEXTURES)
    float u;
    float v;
#endif  // BARYCENTRIC
  };

  // data members kept here for debugging purposes
  // they will be moved later
  bool rayUpdateRequired;
  bool renderRequired;
  Scene scene;
  std::vector<std::vector<Ray>> pixelRays;
  std::vector<std::vector<Color>> colorBuffer;
  void updateRays();
  Color shootRay(const Ray &ray, const unsigned int depth = 0) const;
  Color shade(const Ray &ray) const;
  std::optional<RayTracer::IntersectionInformation> trace(const Ray &ray) const;
  bool hasIntersection(const Ray &ray, const float distanceToLight) const;

 public:
  explicit RayTracer(const std::string &pathToScene, const std::string &basePath);
  const Camera &getCamera() const;
  Camera &setCamera();
  void render();
  void writePPM(const std::string &pathToImage);
};
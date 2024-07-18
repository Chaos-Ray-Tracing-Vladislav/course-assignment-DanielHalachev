#pragma once
#include <string>
#include <vector>

#include "Ray.h"
#include "Scene.h"
#include "tracer/Camera.h"
#include "tracer/Vector.h"

typedef Vector ColorVector;
const int MAX_DEPTH = 5;
const float SHADOW_BIAS = 0.0001f;
const float REFLECTION_BIAS = 1e-4;
const float REFRACTION_BIAS = 1e-4;

class RayTracer {
 private:
  struct IntersectionInformation {
    const Mesh *const object;
    Vector intersectionPoint;
    Vector hitNormal;
#if defined(BARYCENTRIC) && BARYCENTRIC
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
  Color shootRay(const Ray &ray, const unsigned int depth = 0, const float IOR = 0) const;
  Color shade(const Ray &ray) const;
  std::optional<RayTracer::IntersectionInformation> trace(const Ray &ray) const;
  bool hasIntersection(const Ray &ray) const;

 public:
  explicit RayTracer(const std::string &pathToScene);
  const Camera &getCamera() const;
  Camera &setCamera();
  void render();
  void writePPM(const std::string &pathToImage);
};
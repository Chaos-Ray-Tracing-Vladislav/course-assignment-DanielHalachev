#pragma once
#include <string>
#include <vector>

#include "Ray.h"
#include "Scene.h"
#include "tracer/Camera.h"
#include "tracer/Vector.h"

typedef Vector ColorVector;
#define MAX_DEPTH 5

class RayTracer {
 private:
  struct IntersectionInformation {
    const Mesh *const object;
    Vector intersectionPoint;
    Vector hitNormal;
  };

  bool rayUpdateRequired;
  bool renderRequired;
  Scene scene;
  std::vector<std::vector<Ray<Primary>>> pixelRays;
  std::vector<std::vector<Color>> colorBuffer;
  void updateRays();
  Color shootRay(const Ray<Primary> &ray, const unsigned int depth = 0) const;
  template <RayType T>
  Color shade(const Ray<T> &ray) const;
  template <RayType T>
  std::optional<RayTracer::IntersectionInformation> trace(const Ray<T> &ray) const;
  bool hasIntersection(const Ray<Shadow> &ray) const;

 public:
  explicit RayTracer(const std::string &pathToScene);
  const Camera &getCamera() const;
  Camera &setCamera();
  void render();
  void writePPM(const std::string &pathToImage);
};
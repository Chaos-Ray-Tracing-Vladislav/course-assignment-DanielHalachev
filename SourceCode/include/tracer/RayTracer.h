#pragma once
#include <atomic>
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

enum RenderOptimization {
  NoOptimization,
  Regions,
  BucketsThreadPool,
  BucketsQueue,
  AABB,
  BucketsThreadPoolAABB,
  BucketsQueueAABB
};

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

  struct BoundingBox {
    Vector minPoint;
    Vector maxPoint;
    bool hasIntersection(const Ray &ray) {
      Vector tMax;  // tXmin, tYmin, tZmin
      Vector tMin;  // tXmax, tYmax, tZmax
      for (auto i = 0; i < 3; i++) {
        if (ray.direction[i] == 0) {
          if (ray.origin[i] < minPoint[i] || ray.origin[i] > maxPoint[i]) {
            return false;
          }
          tMax[i] = std::numeric_limits<float>::infinity();
          tMin[i] = -std::numeric_limits<float>::infinity();
        } else {
          tMax[i] = (maxPoint[i] - ray.origin[i]) / ray.direction[i];
          tMin[i] = (minPoint[i] - ray.origin[i]) / ray.direction[i];
        }
      }

      for (auto i = 0; i < 3; i++) {
        if (tMin[i] > 0) {
          Vector intersectionPoint = ray.origin + tMin[i] * ray.direction;
          if (intersectionPoint[0] >= this->minPoint[0] && intersectionPoint[0] <= this->maxPoint[0] &&
              intersectionPoint[1] >= this->minPoint[1] && intersectionPoint[1] <= this->maxPoint[1] &&
              intersectionPoint[2] >= this->minPoint[2] && intersectionPoint[2] <= this->maxPoint[2]) {
            return true;
          }
        } else if (tMax[i] > 0) {
          Vector intersectionPoint = ray.origin + tMax[i] * ray.direction;
          if (intersectionPoint[0] >= this->minPoint[0] && intersectionPoint[0] <= this->maxPoint[0] &&
              intersectionPoint[1] >= this->minPoint[1] && intersectionPoint[1] <= this->maxPoint[1] &&
              intersectionPoint[2] >= this->minPoint[2] && intersectionPoint[2] <= this->maxPoint[2]) {
            return true;
          }
        }
      }
      return false;
    }
  } boundingBox;

  Scene scene;
  std::vector<std::vector<Color>> colorBuffer;
  unsigned short rectangleCount = 1;
  std::atomic_ushort rectanglesDone = 0;

  void printProgress(double percentage);
  Ray getRay(unsigned int pixelRow, unsigned int pixelCol) const;
  Color shootRay(const Ray &ray, const unsigned int depth = 0) const;
  Color shade(const Ray &ray) const;
  std::optional<RayTracer::IntersectionInformation> trace(const Ray &ray) const;
  bool hasIntersection(const Ray &ray, const float distanceToLight) const;
  void renderRectangle(unsigned int rowIndex, unsigned int colIndex, unsigned int width, unsigned int height);
  void renderRectangleAABB(unsigned int rowIndex, unsigned int colIndex, unsigned int width, unsigned int height);
  void renderRegions(bool useBoundingBox);
  void renderBucketsThreadpool(bool useBoundingBox);
  void renderBucketsQueue(bool useBoundingBox);

 public:
  explicit RayTracer(Scene &scene);
  const Camera &getCamera() const;
  Camera &setCamera();
  std::vector<std::vector<Color>> render(const std::string &pathToImage, RenderOptimization optimization = AABB);
  void exportPPM(const std::string &pathToImage, const std::vector<std::vector<Color>> &colorBuffer);
};
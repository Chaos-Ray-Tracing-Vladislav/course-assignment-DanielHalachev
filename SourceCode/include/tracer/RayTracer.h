#pragma once
#include <atomic>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "tracer/Ray.h"
#include "tracer/Scene.h"
#include "tree/AccelerationStructure.h"

enum RenderOptimization {
  NoOptimization,
  Regions,
  BucketsThreadPool,
  BucketsQueue,
  AABB,
  BucketsThreadPoolAABB,
  BucketsQueueAABB,
  BVH,
  BVHBucketsThreadPool,
  BVHBucketsQueue
};

struct RenderOptions {
  RenderOptimization optimization = BVHBucketsThreadPool;
  bool USE_GI = false;
  unsigned int MAX_DEPTH = 5;
  unsigned int GI_SAMPLE_SIZE = 2;
  unsigned int RAYS_PER_PIXEL = 1;
  float SHADOW_BIAS = 1e-4;
  float REFLECTION_BIAS = 1e-4;
  float REFRACTION_BIAS = 1e-4;
  float MONTE_CARLO_BIAS = 1e-4;

  explicit RenderOptions(const RenderOptimization optimization = BVHBucketsThreadPool, const unsigned int maxDepth = 5,
                         const bool useGI = false, const unsigned int sampleSize = 2,
                         const unsigned int raysPerPixel = 1, const float shadowBias = 1e-4,
                         const float reflectionBias = 1e-4, const float refractionBias = 1e-4,
                         const float monteCarloBias = 1e-4)
      : optimization{optimization},
        USE_GI{useGI},
        MAX_DEPTH{maxDepth},
        GI_SAMPLE_SIZE{sampleSize},
        RAYS_PER_PIXEL{raysPerPixel},
        SHADOW_BIAS{shadowBias},
        REFLECTION_BIAS{reflectionBias},
        REFRACTION_BIAS{refractionBias},
        MONTE_CARLO_BIAS{monteCarloBias} {};
};

enum BoundingBoxType { SingleBoundingBox, Tree };

struct Region {
  unsigned int rowIndex;
  unsigned int colIndex;
};

class RayTracer {
 private:
  static thread_local std::default_random_engine engine;
  static thread_local std::uniform_real_distribution<float> distribution;

  const BoundingBox boundingBox;
  const AccelerationStructure accelerationStructure;

  const Scene &scene;
  Camera camera;
  std::vector<std::vector<Color>> colorBuffer;

  RenderOptions renderOptions = RenderOptions();
  bool useBounding = true;
  enum BoundingBoxType boundingType = SingleBoundingBox;
  unsigned short threadCount = std::thread::hardware_concurrency();
  unsigned short rectangleCount = 1;
  std::atomic_ushort rectanglesDone = 0;
  std::mutex printMutex;

  void printProgress(double percentage);
  Ray getRay(unsigned int pixelRow, unsigned int pixelCol, const bool random) const;
  Color calculateDiffusion(const Ray &ray, const unsigned int depth,
                           const IntersectionInformation &intersectionInformation);
  Color calculateReflection(const Ray &ray, const unsigned int depth,
                            const IntersectionInformation &intersectionInformation);
  Color calculateRefraction(const Ray &ray, const unsigned int depth,
                            const IntersectionInformation &intersectionInformation);
  Color shootRay(Ray &ray, const unsigned int depth = 0);
  Color shade(const Ray &ray) const;
  std::optional<IntersectionInformation> trace(const Ray &ray) const;
  bool hasIntersection(const Ray &ray, const float distanceToLight) const;
  void renderRectangle(unsigned int rowIndex, unsigned int colIndex, unsigned int width, unsigned int height);
  void renderRegions();
  void renderBucketsThreadpool();
  void renderBucketsQueue();

 public:
  explicit RayTracer(Scene &scene);
  const Camera &getCamera() const;
  Camera &setCamera();
  std::vector<std::vector<Color>> render(const std::string &pathToImage, RenderOptions renderOptions = RenderOptions());
  void exportPPM(const std::string &pathToImage, const std::vector<std::vector<Color>> &colorBuffer);
};

#include "tracer/RayTracer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <numeric>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "threadpool/ThreadManager.h"
#include "tracer/Camera.h"
#include "tracer/Ray.h"
#include "tracer/Scene.h"
#include "tracer/Vector.h"

const float PI = M_PI;
thread_local std::default_random_engine RayTracer::engine(clock() ^
                                                          std::hash<std::thread::id>{}(std::this_thread::get_id()));
thread_local std::uniform_real_distribution<float> RayTracer::distribution(0.0f, 1.0f);

void RayTracer::printProgress(double percentage) {
  // const int progressBarWidth = 60;
  // const std::string str(progressBarWidth, '|');
  // const char *progressBarString = str.c_str();
  // int val = static_cast<int>(percentage * 100);
  // int leftPadding = static_cast<int>(percentage * progressBarWidth);
  // int rightPadding = progressBarWidth - leftPadding;

  // printf("\r%3d%% [%.*s%*s] [%i / %i]", val, leftPadding, progressBarString, rightPadding, "",
  //        this->rectanglesDone.load(), this->rectangleCount);
  // // this could be interrupted by another thread
  // // but this is a sacrifice I'm willing to make :-D
  // fflush(stdout);
}

RayTracer::RayTracer(Scene &scene)
    : boundingBox(scene), accelerationStructure(scene), scene(std::move(scene)), camera(scene.camera) {
  this->colorBuffer.resize(this->scene.sceneSettings.image.height);
  for (auto &row : colorBuffer) {
    row.resize(this->scene.sceneSettings.image.width);
  }
};

const Camera &RayTracer::getCamera() const {
  return this->camera;
}

Camera &RayTracer::setCamera() {
  return this->camera;
}

Ray RayTracer::getRay(unsigned int pixelRow, unsigned int pixelCol, const bool random) const {
  float offsetX = random ? distribution(engine) : 0.5f;
  float offsetY = random ? distribution(engine) : 0.5f;
  float x = static_cast<float>(pixelCol) + offsetX;
  float y = static_cast<float>(pixelRow) + offsetY;

  x = x / static_cast<float>(this->scene.sceneSettings.image.width);
  y = y / static_cast<float>(this->scene.sceneSettings.image.height);

  x = (2.0f * x) - 1.0f;
  y = 1.0f - (2.0f * y);

  x = x * (static_cast<float>(this->scene.sceneSettings.image.width) /
           static_cast<float>(this->scene.sceneSettings.image.height));

  Vector direction(x, y, -1.0);
  direction = direction * this->camera.getRotationMatrix();
  direction.normalize();
  return Ray(this->camera.getPosition(), direction, PrimaryRay);
}

void RayTracer::renderRectangle(unsigned int rowIndex, unsigned int columnIndex, unsigned int width,
                                unsigned int height) {
  unsigned int rowLimit = std::min(this->scene.sceneSettings.image.height, rowIndex + height);
  unsigned int columnLimit = std::min(this->scene.sceneSettings.image.width, columnIndex + width);
  for (unsigned int pixelRow = rowIndex; pixelRow < rowLimit; pixelRow++) {
    for (unsigned int pixelCol = columnIndex; pixelCol < columnLimit; pixelCol++) {
      Ray ray = this->getRay(pixelRow, pixelCol, false);
      const Color color = this->shootRay(ray);
      if (this->renderOptions.USE_GI) {
        std::vector<Color> colorVector;
        colorVector.reserve(this->renderOptions.RAYS_PER_PIXEL);
        colorVector.push_back(color);
        for (unsigned int rayNum = 1; rayNum < this->renderOptions.RAYS_PER_PIXEL; rayNum++) {
          Ray loopRay = this->getRay(pixelRow, pixelCol, true);
          const Color loopColor = this->shootRay(loopRay);
          colorVector.push_back(loopColor);
          if (loopColor == this->scene.sceneSettings.sceneBackgroundColor) {
            break;
          }
        }
        this->colorBuffer[pixelRow][pixelCol] =
            std::accumulate(colorVector.begin(), colorVector.end(), Color(0, 0, 0)) *
            (1.0f / static_cast<float>(colorVector.size()));
      } else {
        this->colorBuffer[pixelRow][pixelCol] = color;
      }
    }
  }
  this->rectanglesDone++;
  printProgress(static_cast<float>(this->rectanglesDone) / static_cast<float>(this->rectangleCount));
}

void RayTracer::renderRegions() {
  unsigned int threadNumY = static_cast<unsigned int>(std::sqrt(this->rectangleCount));
  if (threadNumY == 0) {
    threadNumY = 1;
  }

  unsigned int threadNumX = this->rectangleCount / threadNumY;
  unsigned int regionWidth = this->scene.sceneSettings.image.width / threadNumX;
  unsigned int regionHeight = this->scene.sceneSettings.image.height / threadNumY;
  if (this->threadCount == 1) {
    renderRectangle(0, 0, regionWidth, regionHeight);
    return;
  }
  std::vector<std::thread> threads;
  threads.reserve(this->threadCount);
  for (auto i = 0; i < this->threadCount; i++) {
    unsigned column = (i * regionWidth) % this->scene.sceneSettings.image.width;
    unsigned row = (i / threadNumX) * regionHeight;

    threads.push_back(std::thread(&RayTracer::renderRectangle, this, row, column, regionWidth, regionHeight));
  }
  for (auto &thread : threads) {
    thread.join();
  }
}

void RayTracer::renderBucketsThreadpool() {
  ThreadManager manager(this->threadCount);
  unsigned int threadNumY = static_cast<unsigned int>(std::sqrt(rectangleCount));
  if (threadNumY == 0) {
    threadNumY = 1;
  }
  unsigned int threadNumX = this->rectangleCount / threadNumY;
  unsigned int regionWidth = this->scene.sceneSettings.image.width / threadNumX;
  unsigned int regionHeight = this->scene.sceneSettings.image.height / threadNumY;
  for (auto i = 0; i < rectangleCount; i++) {
    unsigned column = (i * regionWidth) % this->scene.sceneSettings.image.width;
    unsigned row = (i / threadNumX) * regionHeight;
    manager.doJob([this, row, column, regionWidth, regionHeight]() {
      RayTracer::renderRectangle(row, column, regionWidth, regionHeight);
    });
  }
  manager.waitForAll();
}

void RayTracer::renderBucketsQueue() {
  struct Region {
    unsigned int rowIndex;
    unsigned int colIndex;
  };
  std::queue<Region> queue;
  std::mutex mutex;
  unsigned int threadNumY = static_cast<unsigned int>(std::sqrt(rectangleCount));
  if (threadNumY == 0) {
    threadNumY = 1;
  }
  unsigned int threadNumX = this->rectangleCount / threadNumY;
  unsigned int regionWidth = this->scene.sceneSettings.image.width / threadNumX;
  unsigned int regionHeight = this->scene.sceneSettings.image.height / threadNumY;
  for (auto i = 0; i < rectangleCount; i++) {
    unsigned column = (i * regionWidth) % this->scene.sceneSettings.image.width;
    unsigned row = (i / threadNumX) * regionHeight;
    queue.push(Region{row, column});
  }
  std::vector<std::thread> threads;
  threads.reserve(this->threadCount);
  for (auto i = 0; i < this->threadCount; i++) {
    threads.push_back(std::thread([this, regionWidth, regionHeight, &queue, &mutex]() {
      while (true) {
        std::unique_lock<std::mutex> lock(mutex);
        if (queue.empty()) {
          lock.unlock();
          break;
        }
        Region region = queue.front();
        queue.pop();
        lock.unlock();

        RayTracer::renderRectangle(region.rowIndex, region.colIndex, regionWidth, regionHeight);
      }
    }));
  }
  for (auto &thread : threads) {
    thread.join();
  }
}

std::vector<std::vector<Color>> RayTracer::render(const std::string &pathToImage, RenderOptions renderOptions) {
  this->renderOptions = renderOptions;
  auto startTime = std::chrono::high_resolution_clock::now();
  printProgress(0);
  switch (this->renderOptions.optimization) {
    case NoOptimization: {
      this->useBounding = false;
      this->threadCount = 1;
      this->rectangleCount = 1;
      renderRegions();
      break;
    }
    case Regions: {
      this->useBounding = false;
      this->threadCount = std::thread::hardware_concurrency();
      this->rectangleCount = std::thread::hardware_concurrency();
      renderRegions();
      break;
    }
    case BucketsThreadPool: {
      this->useBounding = false;
      this->threadCount = std::thread::hardware_concurrency();
      this->rectangleCount = this->scene.sceneSettings.bucketSize;
      renderBucketsThreadpool();
      break;
    }
    case BucketsQueue: {
      this->useBounding = false;
      this->threadCount = std::thread::hardware_concurrency();
      this->rectangleCount = this->scene.sceneSettings.bucketSize;
      renderBucketsQueue();
      break;
    }
    case AABB: {
      this->useBounding = true;
      this->boundingType = SingleBoundingBox;
      this->threadCount = 1;
      this->rectangleCount = 1;
      this->renderRegions();
      break;
    }
    case BucketsThreadPoolAABB: {
      this->useBounding = true;
      this->boundingType = SingleBoundingBox;
      this->threadCount = std::thread::hardware_concurrency();
      this->rectangleCount = this->scene.sceneSettings.bucketSize;
      this->renderBucketsThreadpool();
      break;
    }
    case BucketsQueueAABB: {
      this->useBounding = true;
      this->boundingType = SingleBoundingBox;
      this->threadCount = std::thread::hardware_concurrency();
      this->rectangleCount = this->scene.sceneSettings.bucketSize;
      this->renderBucketsQueue();
      break;
    }
    case BHV: {
      this->useBounding = true;
      this->boundingType = Tree;
      this->threadCount = 1;
      this->rectangleCount = 1;
      this->renderRegions();
      break;
    }
    case BHVBucketsThreadPool: {
      this->useBounding = true;
      this->boundingType = Tree;
      this->threadCount = std::thread::hardware_concurrency();
      this->rectangleCount = this->scene.sceneSettings.bucketSize;
      this->renderBucketsThreadpool();
      break;
    }
  }
  std::cout << "\n";
  std::cout.flush();
#if defined(MEASURE_TIME) && MEASURE_TIME
  auto endTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = endTime - startTime;
  std::cout << duration.count() << "s\n";
#endif  // MEASURE_TIME
  if (pathToImage != "") {
    this->exportPPM(pathToImage, colorBuffer);
  }
  return colorBuffer;
}

Color RayTracer::calculateDiffusion(const unsigned int depth, const IntersectionInformation &intersectionInformation) {
  Vector finalColor(0, 0, 0);

  const Vector &intersectionPoint = intersectionInformation.intersection.hitPoint;
  const Vector &hitNormal = intersectionInformation.intersection.hitNormal;
  const Mesh &mesh = *intersectionInformation.object;

  for (auto &light : this->scene.lights) {
    Vector lightDirection = light.position - intersectionPoint;
    float distanceToLight = lightDirection.length();
    float sphereRadius = lightDirection.length();
    float sphereArea = 4 * sphereRadius * sphereRadius * PI;
    lightDirection.normalize();
    float angle = std::max(0.0f, lightDirection.dot(hitNormal));

    Ray shadowRay(intersectionPoint + hitNormal * this->renderOptions.SHADOW_BIAS, lightDirection, RayType::ShadowRay);
    bool shadowRayIntersection = hasIntersection(shadowRay, distanceToLight);

    if (!shadowRayIntersection) {
      float directLightContribution = (static_cast<float>(light.intentsity) / sphereArea * angle);
#if (defined USE_TEXTURES) && USE_TEXTURES
      finalColor += directLightContribution * mesh.material.texture.getColor(
                                                  *intersectionInformation.triangle,
                                                  Vector(intersectionInformation.u, intersectionInformation.v,
                                                         1.0f - intersectionInformation.u - intersectionInformation.v));
#else
      finalColor += directLightContribution * mesh.material.albedo;
#endif  // USE_TEXTURE
    }

    if (this->renderOptions.USE_GI) {
      Color indirectLightContribution(0, 0, 0);
      unsigned int sampleSize = this->renderOptions.GI_SAMPLE_SIZE;
      Vector Ny = hitNormal;
      Vector Nx;
      Vector Nz;
      if (std::fabs(Ny[0]) > std::fabs(Ny[1])) {
        Nx = Vector(Ny[2], 0, -Ny[0]) * (1 / std::sqrtf(Ny[0] * Ny[0] + Ny[2] * Ny[2]));
      } else {
        Nx = Vector(0, -Ny[2], Ny[1]) * (1 / sqrtf(Ny[1] * Ny[1] + Ny[2] * Ny[2]));
      }
      Nz = Ny * Nx;
      float probabilityDistributionFunction = 1.0f / (2.0f * PI);
      for (auto i = 0; i < sampleSize; i++) {
        float angle1 = RayTracer::distribution(RayTracer::engine);
        float angle2 = RayTracer::distribution(RayTracer::engine);
        Vector sample = Vector::getVectorSampleOnHemisphere(angle1, angle2);
        Vector sampleInWorldCoordinates(  //
            sample[0] * Nz[0] + sample[1] * Ny[0] + sample[2] * Nx[0],
            sample[0] * Nz[1] + sample[1] * Ny[1] + sample[2] * Nx[1],
            sample[0] * Nz[2] + sample[1] * Ny[2] + sample[2] * Nx[2]);
        Ray sampleRay = Ray(intersectionPoint + sampleInWorldCoordinates * this->renderOptions.MONTE_CARLO_BIAS,
                            sampleInWorldCoordinates, DiffuseRay);
        indirectLightContribution += (angle1 / probabilityDistributionFunction) * shootRay(sampleRay, depth + 1);
      }
      indirectLightContribution = indirectLightContribution * (1.0f / static_cast<float>(sampleSize));
      finalColor += indirectLightContribution * mesh.material.albedo;
    }
  }
  return finalColor;
}
Color RayTracer::calculateReflection(const Ray &ray, const unsigned int depth,
                                     const IntersectionInformation &intersectionInformation) {
  Vector finalColor(0, 0, 0);

  const Vector &intersectionPoint = intersectionInformation.intersection.hitPoint;
  const Vector &hitNormal = intersectionInformation.intersection.hitNormal;
  const Mesh &mesh = *intersectionInformation.object;

  Ray reflectionRay(intersectionPoint + hitNormal * this->renderOptions.REFLECTION_BIAS,
                    ray.direction.reflect(hitNormal).getNormalized(), ReflectionRay);
  Color reflectionColor = shootRay(reflectionRay, depth + 1);
  finalColor += Color(mesh.material.albedo[0] * reflectionColor[0],  // red
                      mesh.material.albedo[1] * reflectionColor[1],  // green
                      mesh.material.albedo[2] * reflectionColor[2]   // blue
  );
  return finalColor;
}
Color RayTracer::calculateRefraction(const Ray &ray, const unsigned int depth,
                                     const IntersectionInformation &intersectionInformation) {
  const Vector &intersectionPoint = intersectionInformation.intersection.hitPoint;
  const Vector &hitNormal = intersectionInformation.intersection.hitNormal;
  const Mesh &mesh = *intersectionInformation.object;

  float eta1 = 1.0f;
  float eta2 = mesh.material.ior;
  Vector normal = hitNormal;
  float incidentDotNormal = ray.direction.dot(normal);

  if (incidentDotNormal > 0) {
    std::swap(eta1, eta2);
    normal = -1 * normal;
    incidentDotNormal = -incidentDotNormal;
  }

  Color reflectionColor(0, 0, 0);
  Color refractionColor(0, 0, 0);

  float cosineAlpha = -incidentDotNormal;
  float sineAlpha = std::sqrtf(std::max(0.0f, 1 - cosineAlpha * cosineAlpha));

  Ray reflectionRay(intersectionPoint + normal * this->renderOptions.REFLECTION_BIAS,
                    ray.direction.reflect(normal).getNormalized(), ReflectionRay);
  reflectionColor = shootRay(reflectionRay, depth + 1);

  float etaRatio = eta1 / eta2;
  float sineBeta = etaRatio * sineAlpha;

  if (sineBeta < 1.0f) {
    float R0 = std::powf((eta1 - eta2) / (eta1 + eta2), 2);
    float fresnelCoefficient = R0 + (1 - R0) * std::powf(1.0f - cosineAlpha, 5);

    float cosineBeta = std::sqrtf(std::max(0.0f, 1 - sineBeta * sineBeta));
    Vector refractionDirection = etaRatio * (ray.direction + cosineAlpha * normal) - cosineBeta * normal;
    Ray refractionRay(intersectionPoint - normal * this->renderOptions.REFRACTION_BIAS,
                      refractionDirection.getNormalized(), RefractionRay);
    refractionColor = shootRay(refractionRay, depth + 1);
    return fresnelCoefficient * reflectionColor + (1 - fresnelCoefficient) * refractionColor;
  }
  return reflectionColor;
}

Color RayTracer::shootRay(Ray &ray, const unsigned int depth) {
  ray.direction.normalize();
  if (this->useBounding && this->boundingType == SingleBoundingBox) {
    if (!this->boundingBox.hasIntersection(ray)) {
      return this->scene.sceneSettings.sceneBackgroundColor;
    }
  }

  if (depth > this->renderOptions.MAX_DEPTH) {
    return this->scene.sceneSettings.sceneBackgroundColor;
  }
  std::optional<IntersectionInformation> intersectionInformationOptional = trace(ray);
  if (intersectionInformationOptional.has_value()) {
    IntersectionInformation intersectionInformation = intersectionInformationOptional.value();
    switch (intersectionInformation.object->material.type) {
      case Diffuse: {
        return this->calculateDiffusion(depth, intersectionInformation);
      }
      case Reflective: {
        return this->calculateReflection(ray, depth, intersectionInformation);
      }
      case Refractive: {
        return this->calculateRefraction(ray, depth, intersectionInformation);
      }
      default: {
        // neither refractive, reflective, nor diffusive
        return this->scene.sceneSettings.sceneBackgroundColor;
      }
    }
  }
  // if there is no intersection
  return this->scene.sceneSettings.sceneBackgroundColor;
}

std::optional<IntersectionInformation> RayTracer::trace(const Ray &ray) const {
  if (this->useBounding) {
    if (this->boundingType == Tree) {
      return this->accelerationStructure.intersect(ray);
    }
  }
  float minDistance = std::numeric_limits<float>::max();
  const Mesh *intersectedObject = nullptr;
  const Triangle *intersectedTriangle = nullptr;
  Intersection intersection;

  for (const auto &object : this->scene.objects) {
    for (const auto &triangle : object.triangles) {
      std::optional<Intersection> tempIntersectionOptional = ray.intersectWithTriangle(triangle);
      if (tempIntersectionOptional.has_value()) {
        const Intersection tempIntersection = tempIntersectionOptional.value();
        float distance = tempIntersection.distance;  // t*ray.direction
        if (distance < minDistance) {
          minDistance = distance;
          intersectedObject = &object;
          intersectedTriangle = &triangle;
          intersection = tempIntersection;
        }
      }
    }
  }
  if (intersectedObject != nullptr) {
    bool calculateUV = intersectedObject->material.smoothShading;
    float u = 0;
    float v = 0;
#if (defined(BARYCENTRIC) && BARYCENTRIC) || (defined(USE_TEXTURES) && USE_TEXTURES)
    calculateUV = true;
#endif  // BARYCENTRIC || USE_TEXTURES
    if (calculateUV == true) {
      std::pair<float, float> UV = intersectedTriangle->getBarycentricCoordinates(intersection.hitPoint);
      u = UV.first;
      v = UV.second;
      if (intersectedObject->material.smoothShading) {
        intersection.hitNormal = ((*intersectedTriangle)[1].normal * u + (*intersectedTriangle)[2].normal * v +
                                  (*intersectedTriangle)[0].normal * (1 - u - v));
        intersection.hitNormal.normalize();
      }
    }

#if (defined(BARYCENTRIC) && BARYCENTRIC) || (defined(USE_TEXTURES) && USE_TEXTURES)
    IntersectionInformation temp{intersectedObject, intersectedTriangle, intersection, u, v};
    return temp;
#else
    return IntersectionInformation{intersectedObject, intersectedTriangle, intersection};
#endif  // BARYCENTRIC
  }
  return {};
}

bool RayTracer::hasIntersection(const Ray &ray, const float distanceToLight) const {
  if (this->useBounding) {
    switch (this->boundingType) {
      case SingleBoundingBox:
        if (this->boundingBox.hasIntersection(ray) == false) {
          return false;
        }
        break;
      case Tree: {
        return this->accelerationStructure.checkForIntersection(ray, distanceToLight, this->renderOptions.USE_GI);
        break;
      }
    }
  }
  for (const auto &object : this->scene.objects) {
    if (this->renderOptions.USE_GI == false) {
      if (ray.rayType == ShadowRay && object.material.type == Refractive) {
        continue;
      }
    }
    for (const auto &triangle : object.triangles) {
      std::optional<Intersection> intersectionOptional = ray.intersectWithTriangle(triangle);
      if (intersectionOptional.has_value()) {
        Intersection intersection = intersectionOptional.value();
        if ((intersection.hitPoint - ray.origin).length() <= distanceToLight) {
          return true;
        }
      }
    }
  }
  return false;
}

void RayTracer::exportPPM(const std::string &pathToImage, const std::vector<std::vector<Color>> &colorBuffer) {
  std::ofstream outputStream(pathToImage);
  outputStream << "P3"
               << "\n"
               << this->scene.sceneSettings.image.width << " " << this->scene.sceneSettings.image.height << "\n"
               << 255 << "\n";
  for (unsigned int pixelRow = 0; pixelRow < this->scene.sceneSettings.image.height; pixelRow++) {
    for (unsigned int pixelCol = 0; pixelCol < this->scene.sceneSettings.image.width; pixelCol++) {
      outputStream << PPMColor(colorBuffer[pixelRow][pixelCol]) << "\t";
    }
    outputStream << "\n";
  }
}

#include "tracer/RayTracer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "threadpool/ThreadManager.h"
#include "tracer/Scene.h"

const float PI = (22.0f / 7.0f);
thread_local std::default_random_engine RayTracer::engine;
thread_local std::uniform_real_distribution<float> RayTracer::distribution(0.0f, 1.0f);

void RayTracer::printProgress(double percentage) {
  const int progressBarWidth = 60;
  const std::string str(progressBarWidth, '|');
  const char *progressBarString = str.c_str();
  int val = static_cast<int>(percentage * 100);
  int leftPadding = static_cast<int>(percentage * progressBarWidth);
  int rightPadding = progressBarWidth - leftPadding;

  printf("\r%3d%% [%.*s%*s] [%i / %i]", val, leftPadding, progressBarString, rightPadding, "",
         this->rectanglesDone.load(), this->rectangleCount);
  // this could be interrupted by another thread
  // but this is a sacrifice I'm willing to make :-D
  fflush(stdout);
}

RayTracer::RayTracer(Scene &scene) {
  this->scene = std::move(scene);
  this->colorBuffer.resize(this->scene.sceneSettings.image.height);
  for (auto &row : colorBuffer) {
    row.resize(this->scene.sceneSettings.image.width);
  }
  for (auto &object : this->scene.objects) {
    for (auto &triangle : object.triangles) {
      for (auto *vertex : triangle.getVertices()) {
        for (auto i = 0; i < 3; i++) {
          this->boundingBox.minPoint[i] = std::min(this->boundingBox.minPoint[i], vertex->position[i]);
          this->boundingBox.maxPoint[i] = std::max(this->boundingBox.maxPoint[i], vertex->position[i]);
        }
      }
    }
  }
};

const Camera &RayTracer::getCamera() const {
  return this->scene.camera;
}

Camera &RayTracer::setCamera() {
  return this->scene.camera;
}

Ray RayTracer::getRay(unsigned int pixelRow, unsigned int pixelCol) const {
  float x = static_cast<float>(pixelCol) + 0.5f;
  float y = static_cast<float>(pixelRow) + 0.5f;

  x = x / static_cast<float>(this->scene.sceneSettings.image.width);
  y = y / static_cast<float>(this->scene.sceneSettings.image.height);

  x = (2.0f * x) - 1.0f;
  y = 1.0f - (2.0f * y);

  x = x * (static_cast<float>(this->scene.sceneSettings.image.width) /
           static_cast<float>(this->scene.sceneSettings.image.height));

  Vector direction(x, y, -1.0);
  direction = direction * this->scene.camera.getRotationMatrix();
  direction.normalize();
  return Ray(this->scene.camera.getPosition(), direction, PrimaryRay);
}

void RayTracer::renderRectangle(unsigned int rowIndex, unsigned int columnIndex, unsigned int width,
                                unsigned int height) {
  unsigned int rowLimit = std::min(this->scene.sceneSettings.image.height, rowIndex + height);
  unsigned int columnLimit = std::min(this->scene.sceneSettings.image.width, columnIndex + width);
  for (unsigned int pixelRow = rowIndex; pixelRow < rowLimit; pixelRow++) {
    for (unsigned int pixelCol = columnIndex; pixelCol < columnLimit; pixelCol++) {
      this->colorBuffer[pixelRow][pixelCol] = this->shootRay(this->getRay(pixelRow, pixelCol));
    }
  }
  this->rectanglesDone++;
  printProgress(static_cast<float>(this->rectanglesDone) / static_cast<float>(this->rectangleCount));
}

void RayTracer::renderRectangleAABB(unsigned int rowIndex, unsigned int columnIndex, unsigned int width,
                                    unsigned int height) {
  unsigned int rowLimit = std::min(this->scene.sceneSettings.image.height, rowIndex + height);
  unsigned int columnLimit = std::min(this->scene.sceneSettings.image.width, columnIndex + width);
  for (unsigned int pixelRow = rowIndex; pixelRow < rowLimit; pixelRow++) {
    for (unsigned int pixelCol = columnIndex; pixelCol < columnLimit; pixelCol++) {
      Ray ray = this->getRay(pixelRow, pixelCol);
      this->colorBuffer[pixelRow][pixelCol] = (this->boundingBox.hasIntersection(ray))
                                                  ? (this->shootRay(ray))
                                                  : (this->scene.sceneSettings.sceneBackgroundColor);
    }
  }
  this->rectanglesDone++;
  printProgress(static_cast<float>(this->rectanglesDone) / static_cast<float>(this->rectangleCount));
}

void RayTracer::renderRegions(bool useBoundingBox) {
  unsigned int threadNumY = static_cast<unsigned int>(std::sqrt(this->rectangleCount));
  if (threadNumY < 0) {
    threadNumY = 1;
  }
  unsigned int threadNumX = this->rectangleCount / threadNumY;
  unsigned int regionWidth = this->scene.sceneSettings.image.width / threadNumX;
  unsigned int regionHeight = this->scene.sceneSettings.image.height / threadNumY;
  std::vector<std::thread> threads;
  threads.reserve(rectangleCount);
  for (auto i = 0; i < rectangleCount; i++) {
    unsigned column = (i * regionWidth) % this->scene.sceneSettings.image.width;
    unsigned row = (i / threadNumX) * regionHeight;
    if (useBoundingBox) {
      threads.push_back(std::thread(&RayTracer::renderRectangleAABB, this, row, column, regionWidth, regionHeight));
    } else {
      threads.push_back(std::thread(&RayTracer::renderRectangle, this, row, column, regionWidth, regionHeight));
    }
  }
  for (auto &thread : threads) {
    thread.join();
  }
}

void RayTracer::renderBucketsThreadpool(bool useBoundingBox) {
  ThreadManager manager(this->rectangleCount);
  unsigned int threadNumY = static_cast<unsigned int>(std::sqrt(rectangleCount));
  if (threadNumY < 0) {
    threadNumY = 1;
  }
  unsigned int threadNumX = this->rectangleCount / threadNumY;
  unsigned int regionWidth = this->scene.sceneSettings.image.width / threadNumX;
  unsigned int regionHeight = this->scene.sceneSettings.image.height / threadNumY;
  for (auto i = 0; i < rectangleCount; i++) {
    unsigned column = (i * regionWidth) % this->scene.sceneSettings.image.width;
    unsigned row = (i / threadNumX) * regionHeight;
    if (useBoundingBox) {
      manager.doJob([this, row, column, regionWidth, regionHeight]() {
        RayTracer::renderRectangleAABB(row, column, regionWidth, regionHeight);
      });
    } else {
      manager.doJob([this, row, column, regionWidth, regionHeight]() {
        RayTracer::renderRectangle(row, column, regionWidth, regionHeight);
      });
    }
  }
  manager.waitForAll();
}

void RayTracer::renderBucketsQueue(bool useBoundingBox) {
  struct Region {
    unsigned int rowIndex;
    unsigned int colIndex;
  };
  std::queue<Region> queue;
  std::mutex mutex;
  unsigned int threadNumY = static_cast<unsigned int>(std::sqrt(rectangleCount));
  if (threadNumY < 0) {
    threadNumY = 1;
  }
  unsigned int threadNumX = this->rectangleCount / threadNumY;
  unsigned int regionWidth = this->scene.sceneSettings.image.width / threadNumX;
  unsigned int regionHeight = this->scene.sceneSettings.image.height / threadNumY;
  std::vector<std::thread> threads;
  for (auto i = 0; i < rectangleCount; i++) {
    unsigned column = (i * regionWidth) % this->scene.sceneSettings.image.width;
    unsigned row = (i / threadNumX) * regionHeight;
    queue.push(Region{row, column});
  }
  threads.reserve(rectangleCount);
  for (auto i = 0; i < rectangleCount; i++) {
    threads.push_back(std::thread([this, useBoundingBox, regionWidth, regionHeight, &queue, &mutex]() {
      while (true) {
        std::unique_lock<std::mutex> lock(mutex);
        if (queue.empty()) {
          lock.unlock();
          break;
        }
        Region region = queue.front();
        queue.pop();
        lock.unlock();
        if (useBoundingBox) {
          RayTracer::renderRectangleAABB(region.rowIndex, region.colIndex, regionWidth, regionHeight);
        } else {
          RayTracer::renderRectangle(region.rowIndex, region.colIndex, regionWidth, regionHeight);
        }
      }
    }));
  }
  for (auto &thread : threads) {
    thread.join();
  }
}

std::vector<std::vector<Color>> RayTracer::render(const std::string &pathToImage, RenderOptimization optimization) {
  auto startTime = std::chrono::high_resolution_clock::now();
  printProgress(0);
  switch (optimization) {
    case NoOptimization: {
      this->rectangleCount = 1;
      renderRegions(false);
      break;
    }
    case Regions: {
      this->rectangleCount = std::thread::hardware_concurrency();
      renderRegions(false);
      break;
    }
    case BucketsThreadPool: {
      this->rectangleCount = this->scene.sceneSettings.bucketSize;
      renderBucketsThreadpool(false);
      break;
    }
    case BucketsQueue: {
      this->rectangleCount = this->scene.sceneSettings.bucketSize;
      renderBucketsQueue(false);
      break;
    }
    case AABB: {
      this->rectangleCount = 1;
      this->renderRegions(true);
      break;
    }
    case BucketsThreadPoolAABB: {
      this->rectangleCount = this->scene.sceneSettings.bucketSize;
      this->renderBucketsThreadpool(true);
      break;
    }
    case BucketsQueueAABB: {
      this->rectangleCount = this->scene.sceneSettings.bucketSize;
      this->renderBucketsQueue(true);
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

Color RayTracer::shootRay(const Ray &ray, const unsigned int depth) const {
  if (depth > MAX_DEPTH) {
    return this->scene.sceneSettings.sceneBackgroundColor;
  }
  std::optional<IntersectionInformation> intersectionInformation = trace(ray);
  if (intersectionInformation.has_value()) {
#if defined(BARYCENTRIC) && BARYCENTRIC
    return Color(intersectionInformation->u, intersectionInformation->v, 0);
#endif  // BARYCENTRIC
    const Vector &intersectionPoint = intersectionInformation->intersectionPoint;
    const Vector &hitNormal = intersectionInformation->hitNormal;
    const Mesh &mesh = *intersectionInformation->object;

    Vector finalColor(0, 0, 0);

    switch (mesh.material.type) {
      case Diffuse: {
        for (auto &light : this->scene.lights) {
          Vector lightDirection = light.position - intersectionPoint;
          float distanceToLight = lightDirection.length();
          float sphereRadius = lightDirection.length();
          float sphereArea = 4 * sphereRadius * sphereRadius * PI;
          lightDirection.normalize();
          float angle = std::max(0.0f, lightDirection.dot(hitNormal));

          Ray shadowRay(intersectionPoint + hitNormal * SHADOW_BIAS, lightDirection, RayType::ShadowRay);
          bool shadowRayIntersection = hasIntersection(shadowRay, distanceToLight);

          if (!shadowRayIntersection) {
            float directLightContribution = (static_cast<float>(light.intentsity) / sphereArea * angle);
#if (defined USE_TEXTURES) && USE_TEXTURES
            finalColor +=
                directLightContribution *
                mesh.material.texture.getColor(*intersectionInformation->triangle,
                                               Vector(intersectionInformation->u, intersectionInformation->v,
                                                      1.0f - intersectionInformation->u - intersectionInformation->v));
#else
            finalColor += directLightContribution * mesh.material.albedo;
#endif  // USE_TEXTURE
          }

#if defined(GLOBAL_ILLUMINATION) && GLOBAL_ILLUMINATION
          // let P be the intersection point
          // we draw a half-sphere
          // we create a local coordinate system, the y axis of which is aligned with the hit normal
          // there are a total of N = sampleSize such rays
          // we trace the ray to determine the light contribution
          // we determine the light contribution and divide by the sample size according to the Monte Carlo technique
          Color indirectLightContribution(0, 0, 0);
          unsigned int sampleSize = DEFAULT_SAMPLE_SIZE;
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
            indirectLightContribution += angle1 / probabilityDistributionFunction *
                                         shootRay(Ray(intersectionPoint + sampleInWorldCoordinates * MONTE_CARLO_BIAS,
                                                      sampleInWorldCoordinates, DiffuseRay),
                                                  depth + 1);
            indirectLightContribution = indirectLightContribution * (1.0f / static_cast<float>(sampleSize));
          }
          finalColor += indirectLightContribution * mesh.material.albedo;
#endif  // GLOBAL_ILLUMINATION
        }
        return finalColor;
      }
      case Reflective: {
        Ray reflectionRay(intersectionPoint + hitNormal * REFLECTION_BIAS,
                          ray.direction.reflect(hitNormal).getNormalized(), ReflectionRay);
        Color reflectionColor = shootRay(reflectionRay, depth + 1);
        finalColor += Color(mesh.material.albedo[0] * reflectionColor[0],  // red
                            mesh.material.albedo[1] * reflectionColor[1],  // green
                            mesh.material.albedo[2] * reflectionColor[2]   // blue
        );
        return finalColor;
      }
      case Refractive: {
        // float eta1 = IOR;
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
        cosineAlpha = std::clamp(cosineAlpha, -1.0f, 1.0f);
        float sineAlpha = std::sqrt(1 - cosineAlpha * cosineAlpha);

        Ray reflectionRay(intersectionPoint + normal * REFLECTION_BIAS, ray.direction.reflect(normal).getNormalized(),
                          ReflectionRay);
        reflectionColor = shootRay(reflectionRay, depth + 1);

        if (sineAlpha < eta1 / eta2) {
          float R0 = std::powf((eta1 - eta2) / (eta1 + eta2), 2);
          float fresnelCoefficient = R0 + (1 - R0) * std::powf(1.0f + incidentDotNormal, 5);
          // float fresnelCoefficient = 0.5f * std::powf(1.0f + incidentDotNormal, 5);

          float sineBeta = (sineAlpha * eta1) / eta2;
          sineBeta = std::clamp(sineBeta, -1.0f, 1.0f);
          float cosineBeta = std::sqrt(1 - sineBeta * sineBeta);
          Vector refractionDirection = -1 * cosineBeta * normal                                  // A
                                       + (ray.direction + cosineAlpha * normal).getNormalized()  // C // B
                                             * sineBeta;                                         //   // B
          Ray refractionRay(intersectionPoint - normal * REFRACTION_BIAS, refractionDirection, RefractionRay);
          refractionColor = shootRay(refractionRay, depth + 1);
          return fresnelCoefficient * reflectionColor + (1 - fresnelCoefficient) * refractionColor;
        }

        return reflectionColor;
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

std::optional<RayTracer::IntersectionInformation> RayTracer::trace(const Ray &ray) const {
  float minDistance = std::numeric_limits<float>::infinity();
  const Mesh *intersectedObject = nullptr;
  const Triangle *intersectedTriangle = nullptr;
  Intersection intersection;

  for (auto &object : this->scene.objects) {
    for (auto &triangle : object.triangles) {
      std::optional<Intersection> tempIntersection =
          ray.intersectWithTriangle(triangle, object.material.type, object.material.smoothShading);
      if (tempIntersection.has_value()) {
        float distance = (tempIntersection.value().hitPoint - ray.origin).length();
        if (distance < minDistance) {
          minDistance = distance;
          intersectedObject = &object;
          intersectedTriangle = &triangle;
          intersection = tempIntersection.value();
        }
      }
    }
  }
  if (intersectedObject != nullptr) {
    //
#if (defined(BARYCENTRIC) && BARYCENTRIC) || (defined(USE_TEXTURES) && USE_TEXTURES)
    IntersectionInformation temp{intersectedObject,      intersectedTriangle, intersection.hitPoint,
                                 intersection.hitNormal, intersection.u,      intersection.v};
    return temp;
#endif  // BARYCENTRIC
    return IntersectionInformation{intersectedObject, intersectedTriangle, intersection.hitPoint,
                                   intersection.hitNormal};
  }
  return {};
}

bool RayTracer::hasIntersection(const Ray &ray, const float distanceToLight) const {
  for (auto &object : this->scene.objects) {
#if (!defined GLOBAL_ILLUMINATION) || ((defined GLOBAL_ILLUMINATION) && !GLOBAL_ILLUMINATION)
    if (ray.rayType == ShadowRay && object.material.type == Refractive) {
      continue;
    }
#endif
    for (auto &triangle : object.triangles) {
      std::optional<Intersection> intersection =
          ray.intersectWithTriangle(triangle, object.material.type, object.material.smoothShading);
      if (intersection.has_value() && (intersection->hitPoint - ray.origin).length() <= distanceToLight) {
        return true;
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

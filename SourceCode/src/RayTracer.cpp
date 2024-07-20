
#include "tracer/RayTracer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>

#include "tracer/Camera.h"
#include "tracer/Ray.h"
#include "tracer/Scene.h"
#include "tracer/SceneParser.h"
#include "tracer/Triangle.h"
#include "tracer/Utils.h"
#include "tracer/Vector.h"

const float PI = (22.0f / 7.0f);

void printProgress(double percentage) {
  const char *progressBarString = "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||";
  const int progressBarWidth = 60;
  int val = static_cast<int>(percentage * 100);
  int leftPadding = static_cast<int>(percentage * progressBarWidth);
  int rightPadding = progressBarWidth - leftPadding;
  printf("\r%3d%% [%.*s%*s]", val, leftPadding, progressBarString, rightPadding, "");
  fflush(stdout);
}

RayTracer::RayTracer(const std::string &pathToScene) : rayUpdateRequired(true), renderRequired(true) {
  this->scene = std::move(SceneParser::parseScene(pathToScene));
};

const Camera &RayTracer::getCamera() const {
  return this->scene.camera;
}

Camera &RayTracer::setCamera() {
  this->rayUpdateRequired = true;
  this->renderRequired = true;
  return this->scene.camera;
}

void RayTracer::updateRays() {
  this->pixelRays.resize(this->scene.sceneSettings.image.height);
  for (auto &row : this->pixelRays) {
    row.resize(this->scene.sceneSettings.image.width);
  }
  this->colorBuffer.resize(this->scene.sceneSettings.image.height);
  for (auto &row : this->colorBuffer) {
    row.resize(this->scene.sceneSettings.image.width);
  }

  for (unsigned int pixelRow = 0; pixelRow < this->scene.sceneSettings.image.height; pixelRow++) {
    for (unsigned int pixelCol = 0; pixelCol < this->scene.sceneSettings.image.width; pixelCol++) {
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
      this->pixelRays[pixelRow][pixelCol] = Ray(this->scene.camera.getPosition(), direction, Primary);
    }
  }
  this->rayUpdateRequired = false;
}

void RayTracer::render() {
  auto startTime = std::chrono::high_resolution_clock::now();
  if (this->rayUpdateRequired == true) {
    this->updateRays();
  }
  for (unsigned int pixelRow = 0; pixelRow < this->scene.sceneSettings.image.height; pixelRow++) {
    for (unsigned int pixelCol = 0; pixelCol < this->scene.sceneSettings.image.width; pixelCol++) {
      this->colorBuffer[pixelRow][pixelCol] = this->shootRay(this->pixelRays[pixelRow][pixelCol]);
    }
    printProgress(static_cast<float>(pixelRow) / static_cast<float>(this->scene.sceneSettings.image.height));
  }
  std::cout << "\n";
  std::cout.flush();
  this->renderRequired = false;

#if defined(MEASURE_TIME) && MEASURE_TIME
  auto endTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = endTime - startTime;
  std::cout << duration.count() << "s\n";
#endif  // MEASURE_TIME
}

Color RayTracer::shootRay(const Ray &ray, const unsigned int depth, const float IOR) const {
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

          Ray shadowRay(intersectionPoint + hitNormal * SHADOW_BIAS, lightDirection, RayType::Shadow);
          bool shadowRayIntersection = hasIntersection(shadowRay, distanceToLight);

          if (!shadowRayIntersection) {
            float lightContribution = (static_cast<float>(light.intentsity) / sphereArea * angle);
            finalColor += lightContribution * mesh.material.albedo;
          }
        }
        return finalColor;
      }
      case Reflective: {
        Ray reflectionRay(intersectionPoint + hitNormal * REFLECTION_BIAS,
                          ray.direction.reflect(hitNormal).getNormalized(), Reflection);
        Color reflectionColor = shootRay(reflectionRay, depth + 1, mesh.material.ior);
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
                          Reflection);
        reflectionColor = shootRay(reflectionRay, depth + 1, mesh.material.ior);

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
          Ray refractionRay(intersectionPoint - normal * REFRACTION_BIAS, refractionDirection, Refraction);
          refractionColor = shootRay(refractionRay, depth + 1, mesh.material.ior);
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
          intersection = tempIntersection.value();
        }
      }
    }
  }
  if (intersectedObject != nullptr) {
    //
#if defined(BARYCENTRIC) && BARYCENTRIC
    IntersectionInformation temp{intersectedObject, intersection.hitPoint, intersection.hitNormal, intersection.u,
                                 intersection.v};
    return temp;
#endif  // BARYCENTRIC
    return IntersectionInformation{intersectedObject, intersection.hitPoint, intersection.hitNormal};
  }
  return {};
}

bool RayTracer::hasIntersection(const Ray &ray, const float distanceToLight) const {
  for (auto &object : this->scene.objects) {
    if (ray.rayType == Shadow && object.material.type == Refractive) {
      continue;
    }
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

void RayTracer::writePPM(const std::string &pathToImage) {
  if (this->renderRequired == true) {
    this->render();
  }
  std::ofstream outputStream(pathToImage);
  outputStream << "P3"
               << "\n"
               << this->scene.sceneSettings.image.width << " " << this->scene.sceneSettings.image.height << "\n"
               << 255 << "\n";
  for (unsigned int pixelRow = 0; pixelRow < this->scene.sceneSettings.image.height; pixelRow++) {
    for (unsigned int pixelCol = 0; pixelCol < this->scene.sceneSettings.image.width; pixelCol++) {
      outputStream << PPMColor(this->colorBuffer[pixelRow][pixelCol]) << "\t";
    }
    outputStream << "\n";
  }
}

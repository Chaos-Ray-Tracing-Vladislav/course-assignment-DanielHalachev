#include <cmath>
#include <iostream>
#include <ostream>
#include <string>

#include "tracer/Matrix.h"
#include "tracer/RayTracer.h"
#include "tracer/SceneParser.h"

int main() {
  SceneParser parser;
  for (short i = 2; i < 3; i++) {
    std::cout << i << '\n';
    Scene scene = parser.parseScene("scene" + std::to_string(i) + ".crtscene", "/home/daniel");
    RayTracer tracer(scene);
    // RenderOptions options{BHVBucketsThreadPool, 5, true, 1, 32};
    RenderOptions options{BHV, 5, false};

    const short FPS = 24;
    const short SECONDS = 10;
    const float DEG_CHANGE = 360.0f / (FPS * SECONDS);

    const float radius = 5.12f;  // Radius of the circle around the scene

    float degrees = 0;

    for (float t = 0; t <= FPS * SECONDS; ++t) {
      float radians = degrees * (M_PIf / 180.0f);

      float x = std::sinf(radians) * radius;
      float z = std::cosf(radians) * radius - 3;
      tracer.setCamera().setPosition() = Vector(x, 0, z);
      float deltaX = x - 0;  // x - centerX
      float deltaZ = z + 3;  // z - CenterZ
      tracer.setCamera().setRotationMatrix() = Matrix<3>::IDENTITY_MATRIX;
      float lookAtAngle = std::atan2(deltaX, deltaZ) * (180.0f / M_PIf);
      tracer.setCamera().pan(lookAtAngle);  // Adjust based on how your pan() method is defined

      tracer.render("/home/daniel/Result/result" + std::to_string(t) + ".ppm", options);

      degrees += DEG_CHANGE;
    }

    std::cout << '\n';
  }
  return 0;
}
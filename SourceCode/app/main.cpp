#include <iostream>
#include <ostream>
#include <string>

#include "tracer/RayTracer.h"
#include "tracer/SceneParser.h"

int main() {
  SceneParser parser;
  for (short i = 0; i < 1; i++) {
    std::cout << i << '\n';
    Scene scene = parser.parseScene("scene" + std::to_string(i) + ".crtscene", "/home/daniel");
    RayTracer tracer(scene);
    tracer.render("/home/daniel/result" + std::to_string(i) + ".ppm", RenderOptimization::BHVBucketsThreadPool);
    std::cout << '\n';
  }
  return 0;
}
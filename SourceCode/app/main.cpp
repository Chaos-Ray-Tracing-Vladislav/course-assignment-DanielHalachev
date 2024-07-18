#include <iostream>
#include <ostream>
#include <string>

#include "tracer/RayTracer.h"

int main() {
  for (short i = 0; i < 9; i++) {
    std::cout << i << '\n';
    RayTracer tracer("/home/daniel/scene" + std::to_string(i) + ".crtscene");
    tracer.render();
    tracer.writePPM("/home/daniel/result" + std::to_string(i) + ".ppm");
    std::cout << '\n';
  }
  return 0;
}
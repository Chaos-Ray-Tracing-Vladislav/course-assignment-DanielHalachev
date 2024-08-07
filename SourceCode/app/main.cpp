#include <iostream>
#include <ostream>
#include <string>

#include "tracer/RayTracer.h"

int main() {
  for (short i = 0; i < 5; i++) {
    std::cout << i << '\n';
    RayTracer tracer("scene" + std::to_string(i) + ".crtscene", "/home/daniel/TexturesHomework");
    tracer.render();
    tracer.writePPM("/home/daniel/result" + std::to_string(i) + ".ppm");
    std::cout << '\n';
  }
  return 0;
}
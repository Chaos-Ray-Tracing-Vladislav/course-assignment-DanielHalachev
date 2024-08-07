#include "tracer/Vertex.h"

#if (defined USE_TEXTURES) && USE_TEXTURES
Vertex::Vertex(const Vector &position, const Vector &normal, const Vector &UV)
    : position{position}, normal{normal}, UV{UV} {};
#else
Vertex::Vertex(const Vector &position, const Vector &normal) : position{position}, normal{normal} {};
#endif  // USE_TEXTURES

float &Vertex::operator[](const unsigned short index) {
  return this->position[index];
}

const float &Vertex::operator[](const unsigned short index) const {
  return this->position[index];
}
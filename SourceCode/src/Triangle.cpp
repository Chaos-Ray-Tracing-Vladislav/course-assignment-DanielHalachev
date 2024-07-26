#include <tracer/Triangle.h>

#include <array>
#include <limits>

#include "tracer/Vector.h"

// Triangle::Triangle() = default;
// Triangle::Triangle(const std::array<Vertex, 3> &vertices) : vertices(vertices) {
//   this->normal = calculateNormal();
// }

Triangle::Triangle(Vertex &v1, Vertex &v2, Vertex &v3) : vertices{&v1, &v2, &v3} {
  this->normal = this->calculateNormal();
  this->normal.normalize();
}

const Vertex &Triangle::operator[](unsigned short i) const {
  return *this->vertices[i];
}

Vector Triangle::calculateNormal() const {
  Vector v1 = this->vertices[1]->position - this->vertices[0]->position;
  Vector v2 = this->vertices[2]->position - this->vertices[0]->position;
  Vector result = (v1 * v2);
  return result;
}

const Vector &Triangle::getTriangleNormal() const {
  return this->normal;
}

const std::array<const Vertex *const, TRIANGLE_NUM_VERTICES> &Triangle::getVertices() const {
  return this->vertices;
}

bool Triangle::pointIsInTriangle(const Vector &point) const {
  Vector e0 = this->vertices[1]->position - this->vertices[0]->position;
  Vector c0 = point - this->vertices[0]->position;
  if (this->normal.dot(e0 * c0) < -std::numeric_limits<float>::epsilon()) {
    return false;
  }

  Vector e1 = this->vertices[2]->position - this->vertices[1]->position;
  Vector c1 = point - this->vertices[1]->position;
  if (this->normal.dot(e1 * c1) < -std::numeric_limits<float>::epsilon()) {
    return false;
  }

  Vector e2 = this->vertices[0]->position - this->vertices[2]->position;
  Vector c2 = point - this->vertices[2]->position;
  if (this->normal.dot(e2 * c2) < -std::numeric_limits<float>::epsilon()) {
    return false;
  }

  return true;
}

float Triangle::area() const {
  return this->normal.length() / 2;
}

std::pair<float, float> Triangle::getBarycentricCoordinates(const Vector &intersectionPoint) const {
  float u = 0;
  float v = 0;
  Vector v0p = intersectionPoint - this->vertices[0]->position;
  Vector v0v1 = this->vertices[1]->position - this->vertices[0]->position;
  Vector v0v2 = this->vertices[2]->position - this->vertices[0]->position;
  float area = (v0v1 * v0v2).length();
  u = (v0p * v0v2).length() / area;
  v = (v0v1 * v0p).length() / area;
  return {u, v};
}

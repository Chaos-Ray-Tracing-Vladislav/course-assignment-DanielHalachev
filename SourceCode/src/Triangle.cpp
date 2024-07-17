#include <tracer/Triangle.h>

#include <array>

#include "tracer/Vector.h"

// Triangle::Triangle() = default;
// Triangle::Triangle(const std::array<Vertex, 3> &vertices) : vertices(vertices) {
//   this->normal = calculateNormal();
// }

Triangle::Triangle(Vertex &v1, Vertex &v2, Vertex &v3) : vertices{&v1, &v2, &v3} {
  this->normal = this->calculateNormal();
  this->normal.normalize();
}

Vertex &Triangle::operator[](unsigned short i) {
  return *this->vertices[i];
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

bool Triangle::pointIsInTriangle(const Vector &point) const {
  Vector e0 = this->vertices[1]->position - this->vertices[0]->position;
  Vector c0 = point - this->vertices[0]->position;
  if (this->normal.dot(e0 * c0) < 0) {
    return false;
  }

  Vector e1 = this->vertices[2]->position - this->vertices[1]->position;
  Vector c1 = point - this->vertices[1]->position;
  if (this->normal.dot(e1 * c1) < 0) {
    return false;
  }

  Vector e2 = this->vertices[0]->position - this->vertices[2]->position;
  Vector c2 = point - this->vertices[2]->position;
  if (this->normal.dot(e2 * c2) < 0) {
    return false;
  }
  return true;
}

float Triangle::area() const {
  return this->calculateNormal().length() / 2;
}
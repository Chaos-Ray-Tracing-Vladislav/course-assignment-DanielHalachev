#pragma once
#include <array>

#include "Vertex.h"
#include "tracer/Vector.h"

#define TRIANGLE_NUM_VERTICES 3

class Triangle {
 private:
  std::array<Vertex *, TRIANGLE_NUM_VERTICES> vertices;
  Vector normal;

 public:
  // Triangle();
  // explicit Triangle(const std::array<Vertex, 3> &vertices);
  Triangle(Vertex &v1, Vertex &v2, Vertex &v3);
  Vertex &operator[](unsigned short i);
  const Vertex &operator[](unsigned short i) const;
  const std::array<Vertex *, TRIANGLE_NUM_VERTICES> &getVertices() const;
  const Vector &getTriangleNormal() const;
  Vector calculateNormal() const;
  bool pointIsInTriangle(const Vector &point) const;
  float area() const;
  std::pair<float, float> getBarycentricCoordinates(const Vector &hitPoint) const;
};
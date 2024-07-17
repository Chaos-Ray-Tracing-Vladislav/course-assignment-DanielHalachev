#pragma once
#include <array>

#include "Vector.h"

#define TRIANGLE_NUM_VERTICES 3
#define SHADOW_BIAS 0.0001f

struct Light;

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
  const Vector &getTriangleNormal() const;
  Vector calculateNormal() const;
  bool pointIsInTriangle(const Vector &point) const;
  float area() const;
};
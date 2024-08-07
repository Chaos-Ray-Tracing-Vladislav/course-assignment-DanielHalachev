#pragma once
#include <array>
#include <ostream>
#include <vector>
class Vector {
 protected:
  std::array<float, 3> values;

 public:
  Vector();
  Vector(float x, float y, float z);
  explicit Vector(const std::vector<float> &values);
  const float &operator[](const unsigned short index) const;
  float &operator[](const unsigned short index);
  Vector operator-(const Vector &other) const;
  Vector operator+(const Vector &other) const;
  Vector &operator+=(const Vector &other);
  float dot(const Vector &other) const;
  Vector operator*(const Vector &other) const;
  Vector operator*(float scalar) const;
  friend Vector operator*(float lhs, const Vector &rhs);
  friend std::ostream &operator<<(std::ostream &os, const Vector &vector);
  friend std::istream &operator>>(std::istream &is, Vector &vector);
  void normalize();
  Vector getNormalized() const;
  float length() const;
  Vector reflect(const Vector &normal) const;
  static Vector generateRandom();
};

class Vertex {
 private:
 public:
  Vector position;
  Vector normal;

 public:
  explicit Vertex(const Vector &position, const Vector &normal = Vector()) : position(position), normal(normal){};

  float &operator[](const unsigned short index) {
    return this->position[index];
  }

  const float &operator[](const unsigned short index) const {
    return this->position[index];
  }
};
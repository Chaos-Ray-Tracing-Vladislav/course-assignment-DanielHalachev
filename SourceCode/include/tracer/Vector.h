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
  bool operator==(const Vector &other) const;
  friend std::ostream &operator<<(std::ostream &os, const Vector &vector);
  friend std::istream &operator>>(std::istream &is, Vector &vector);
  void normalize();
  Vector getNormalized() const;
  float length() const;
  Vector reflect(const Vector &normal) const;
  static Vector generateRandom();
  static Vector getVectorSampleOnHemisphere(const float angle1, const float angle2);
};
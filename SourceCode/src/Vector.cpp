#include <tracer/Vector.h>

#include <cmath>
#include <istream>
#include <random>
#include <stdexcept>
#include <vector>

Vector::Vector() : values{{0.0f, 0.0f, 0.0f}} {}

Vector::Vector(float x, float y, float z) : values{{x, y, z}} {}

Vector::Vector(const std::vector<float>& vectorValues) {
  if (vectorValues.size() != 3) {
    throw std::invalid_argument("Vector must have exactly 3 elements.");
  }
  this->values = {vectorValues[0], vectorValues[1], vectorValues[2]};
}

const float& Vector::operator[](const unsigned short index) const {
  if (index >= 3) {
    throw std::out_of_range("Invalid range");
  }
  return this->values[index];
}

float& Vector::operator[](const unsigned short index) {
  if (index >= 3) {
    throw std::out_of_range("Invalid range");
  }
  return this->values[index];
}

Vector Vector::operator-(const Vector& other) const {
  return {
      this->values[0] - other.values[0],  // x
      this->values[1] - other.values[1],  // y
      this->values[2] - other.values[2]   // z
  };
}

Vector Vector::operator+(const Vector& other) const {
  return {
      this->values[0] + other.values[0],  // x
      this->values[1] + other.values[1],  // y
      this->values[2] + other.values[2]   // z
  };
}

Vector& Vector::operator+=(const Vector& other) {
  this->values[0] += other.values[0];
  this->values[1] += other.values[1];
  this->values[2] += other.values[2];
  return *this;
}

float Vector::dot(const Vector& other) const {
  return this->values[0] * other.values[0] + this->values[1] * other.values[1] + this->values[2] * other.values[2];
}

Vector Vector::operator*(const Vector& other) const {
  return {this->values[1] * other.values[2] - this->values[2] * other.values[1],   // x
          this->values[2] * other.values[0] - this->values[0] * other.values[2],   // y
          this->values[0] * other.values[1] - this->values[1] * other.values[0]};  // z
}

Vector Vector::operator*(float scalar) const {
  return {this->values[0] * scalar, this->values[1] * scalar, this->values[2] * scalar};
}

Vector operator*(float lhs, const Vector& rhs) {
  return {lhs * rhs.values[0], lhs * rhs.values[1], lhs * rhs.values[2]};
}

std::ostream& operator<<(std::ostream& os, const Vector& vector) {
  os << "[" << vector.values[0] << "," << vector.values[1] << "," << vector.values[2] << "]";
  return os;
}

std::istream& operator>>(std::istream& is, Vector& vector) {
  char leftBracket = '[';
  char rightBracket = ']';
  char delim = ',';
  is >> leftBracket >> vector.values[0] >> delim >> vector.values[1] >> delim >> vector.values[2] >> rightBracket;
  return is;
}

void Vector::normalize() {
  float length = this->length();
  if (length == 0) {
    return;
  }
  length = 1.0f / length;
  values[0] *= length;
  values[1] *= length;
  values[2] *= length;
}

Vector Vector::getNormalized() const {
  Vector temp(*this);
  temp.normalize();
  return temp;
}

float Vector::length() const {
  return std::sqrt(this->values[0] * this->values[0] + this->values[1] * this->values[1] +
                   this->values[2] * this->values[2]);
}

Vector Vector::reflect(const Vector& normal) const {
  Vector result = *this - (2 * this->dot(normal)) * normal;
  return result;
}

Vector Vector::generateRandom() {
  std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
  std::mt19937 rng(std::random_device{}());
  return Vector(distribution(rng), distribution(rng), distribution(rng));
}

Vector Vector::getVectorSampleOnHemisphere(const float angle1, const float angle2) {
  // angle1 = cos(theta) = y coordinate
  float sinTheta = std::sqrtf(1 - angle1 * angle1);
  float phi = 2.0f * static_cast<float>(M_PI) * angle2;
  return Vector(sinTheta * std::cosf(phi), angle1, sinTheta * std::sinf(phi));
}
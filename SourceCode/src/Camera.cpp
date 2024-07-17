#include "tracer/Camera.h"

#include <math.h>

#include <cmath>

#include "tracer/Matrix.h"
#include "tracer/Vector.h"

float degreesToRadians(const float degrees) {
  return degrees * (22 / (7 * 180.0f));
}

Camera::Camera() : position{0, 0, 0}, rotationMatrix{Matrix<3>::IDENTITY_MATRIX} {};
Camera::Camera(const Vector& position) : position{position}, rotationMatrix{Matrix<3>::IDENTITY_MATRIX} {};

const Vector& Camera::getPosition() const {
  return this->position;
}

Vector& Camera::setPosition() {
  return this->position;
}

const Matrix<>& Camera::getRotationMatrix() const {
  return this->rotationMatrix;
}

Matrix<>& Camera::setRotationMatrix() {
  return this->rotationMatrix;
}

Camera& Camera::truck(const Vector& direction) {
  const Vector moveDirection = direction * this->rotationMatrix;
  this->position += moveDirection;
  return *this;
}

Camera& Camera::pan(const float degrees) {
  const float radians = degreesToRadians(degrees);
  const Matrix rotateAroundYAxis{
      cosf(radians), 0.0f, -sinf(radians),  //
      0.0f,          1.0f, 0.0f,            //
      sinf(radians), 0.0f, cosf(radians)    //
  };
  this->rotationMatrix *= rotateAroundYAxis;
  return *this;
}

Camera& Camera::roll(const float degrees) {
  const float radians = degreesToRadians(degrees);
  const Matrix rotateAroundYAxis{
      cosf(radians), -sinf(radians), 0.0f,  //
      sinf(radians), cosf(radians),  0.0f,  //
      0.0f,          0.0f,           1.0f   //
  };
  this->rotationMatrix *= rotateAroundYAxis;
  return *this;
}

Camera& Camera::tilt(const float degrees) {
  const float radians = degreesToRadians(degrees);
  const Matrix rotateAroundYAxis{
      1.0f, 0.0f,          0.0f,            //
      0.0f, cosf(radians), -sinf(radians),  //
      0.0f, sinf(radians), cosf(radians)    //
  };
  this->rotationMatrix *= rotateAroundYAxis;
  return *this;
}

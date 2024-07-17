#pragma once

#include "tracer/Matrix.h"
#include "tracer/Vector.h"
struct Camera {
 private:
  Vector position;
  Matrix<> rotationMatrix;

 public:
  Camera();
  explicit Camera(const Vector& position);
  const Vector& getPosition() const;
  Vector& setPosition();
  const Matrix<3>& getRotationMatrix() const;
  Matrix<3>& setRotationMatrix();
  Camera& truck(const Vector& direction);
  Camera& pan(const float degress);
  Camera& tilt(const float degrees);
  Camera& roll(const float degrees);
};
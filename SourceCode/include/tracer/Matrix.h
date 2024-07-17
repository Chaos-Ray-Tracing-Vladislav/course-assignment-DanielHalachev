#pragma once
#include <array>
#include <vector>

#include "tracer/Vector.h"
template <size_t Dimension = 3>
class Matrix {
 private:
  float table[Dimension][Dimension] = {};
  static Matrix<Dimension> generateIndentityMatrix();

 public:
  const static Matrix<Dimension> IDENTITY_MATRIX;

 public:
  Matrix();
  explicit Matrix(const std::array<float, Dimension * Dimension>& values);
  explicit Matrix(const std::vector<float>& values);
  Matrix(const std::initializer_list<float>& values);
  float* operator[](unsigned short index);
  const float* operator[](unsigned short index) const;
  Matrix<Dimension> operator+(const Matrix<Dimension>& right) const;
  Matrix<Dimension>& operator+=(const Matrix<Dimension>& right);
  template <size_t Dim>
  friend Matrix<Dim> operator*(const Matrix<Dim>& left, const Matrix<Dim>& right);
  friend Vector operator*(const Vector& left, const Matrix<3>& right);
  template <size_t Dim>
  friend Matrix<Dim>& operator*=(Matrix<Dim>& left, const Matrix<Dim>& right);
  friend Vector& operator*=(Vector& left, const Matrix<3>& right);
};

template <size_t Dimension>
const Matrix<Dimension> Matrix<Dimension>::IDENTITY_MATRIX = Matrix<Dimension>::generateIndentityMatrix();

template <size_t Dimension>
Matrix<Dimension> Matrix<Dimension>::generateIndentityMatrix() {
  Matrix<Dimension> result;
  for (auto i = 0; i < Dimension; i++) {
    result.table[i][i] = 1;
  }
  return result;
}

template <size_t Dimension>
Matrix<Dimension>::Matrix() = default;

template <size_t Dimension>
Matrix<Dimension>::Matrix(const std::array<float, Dimension * Dimension>& values) {
  auto current = values.begin();
  for (auto i = 0; i < Dimension; i++) {
    for (auto j = 0; j < Dimension; j++) {
      if (current != values.end()) {
        this->table[i][j] = *current;
        current++;
      } else {
        this->table[i][j] = 0;
      }
    }
  }
}

template <size_t Dimension>
Matrix<Dimension>::Matrix(const std::vector<float>& values) {
  auto current = values.begin();
  for (auto i = 0; i < Dimension; i++) {
    for (auto j = 0; j < Dimension; j++) {
      if (current != values.end()) {
        this->table[i][j] = *current;
        current++;
      } else {
        this->table[i][j] = 0;
      }
    }
  }
}

template <size_t Dimension>
Matrix<Dimension>::Matrix(const std::initializer_list<float>& values) {
  const auto* current = values.begin();
  for (auto i = 0; i < Dimension; i++) {
    for (auto j = 0; j < Dimension; j++) {
      if (current != values.end()) {
        this->table[i][j] = *current;
        current++;
      } else {
        this->table[i][j] = 0;
      }
    }
  }
}

template <size_t Dimension>
float* Matrix<Dimension>::operator[](unsigned short index) {
  return this->table[index];
}

template <size_t Dimension>
const float* Matrix<Dimension>::operator[](unsigned short index) const {
  return this->table[index];
}

template <size_t Dimension>
Matrix<Dimension> Matrix<Dimension>::operator+(const Matrix& right) const {
  Matrix result = *this;
  for (auto i = 0; i < Dimension; i++) {
    for (auto j = 0; j < Dimension; j++) {
      result.table[i][j] += right.table[i][j];
    }
  }
  return result;
}

template <size_t Dimension>
Matrix<Dimension>& Matrix<Dimension>::operator+=(const Matrix& right) {
  for (auto i = 0; i < Dimension; i++) {
    for (auto j = 0; j < Dimension; j++) {
      this->table[i][j] += right.table[i][j];
    }
  }
  return *this;
}

template <size_t Dimension>
Matrix<Dimension> operator*(const Matrix<Dimension>& left, const Matrix<Dimension>& right) {
  Matrix result;
  for (auto i = 0; i < Dimension; i++) {
    for (auto j = 0; j < Dimension; j++) {
      result.table[i][j] = 0;
      for (auto k = 0; k < Dimension; k++) {
        result.table[i][j] += left.table[i][k] * right.table[k][j];
      }
    }
  }
  return result;
}

inline Vector operator*(const Vector& left, const Matrix<3>& right) {
  Vector result;
  return Vector(left[0] * right[0][0] + left[1] * right[1][0] + left[2] * right[2][0],   // x
                left[0] * right[0][1] + left[1] * right[1][1] + left[2] * right[2][1],   // y
                left[0] * right[0][2] + left[1] * right[1][2] + left[2] * right[2][2]);  // z
}

template <size_t Dimension>
Matrix<Dimension>& operator*=(Matrix<Dimension>& left, const Matrix<Dimension>& right) {
  Matrix result;
  for (auto i = 0; i < Dimension; i++) {
    for (auto j = 0; j < Dimension; j++) {
      result.table[i][j] = 0;
      for (auto k = 0; k < Dimension; k++) {
        result.table[i][j] += left.table[i][k] * right.table[k][j];
      }
    }
  }
  left = result;
  return left;
}

inline Vector& operator*=(Vector& left, const Matrix<3>& right) {
  left = Vector(left[0] * right[0][0] + left[1] * right[1][0] + left[2] * right[2][0],   // x
                left[0] * right[0][1] + left[1] * right[1][1] + left[2] * right[2][1],   // y
                left[0] * right[0][2] + left[1] * right[1][2] + left[2] * right[2][2]);  // z
  return left;
}
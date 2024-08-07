#pragma once

#include <string>

#include "tracer/Color.h"
#include "tracer/Triangle.h"

class Texture {
 public:
  const std::string name;

 public:
  Texture() = default;
  explicit Texture(const std::string &name);
  virtual const Color getColor(const Triangle &triangle, const Vector &barycentricCoordinates) const = 0;
  virtual ~Texture() = default;
};

class AlbedoTexture : public Texture {
 private:
  Albedo albedo;

 public:
  AlbedoTexture(const std::string &name, const Albedo &albedo);
  virtual const Color getColor(const Triangle & /*triangle*/, const Vector & /*barycentricCoordinates*/) const;
};

class EdgeTexture : public Texture {
 private:
  Color innerColor;
  Color edgeColor;
  float width;

 public:
  EdgeTexture(const std::string &name, const Color &innerColor, const Color &edgeColor, const float width);
  virtual const Color getColor(const Triangle & /*triangle*/, const Vector &barycentricCoordinates) const;
};

class CheckerTexture : public Texture {
 private:
  Color colorA;
  Color colorB;
  float squareSize = 0;

 public:
  CheckerTexture(const std::string &name, const Color &colorA, const Color &colorB, const float squareSize);
  virtual const Color getColor(const Triangle &triangle, const Vector &barycentricCoordinates) const;
};

class BitmapTexture : public Texture {
 private:
  int width;
  int height;
  int channels;
  std::vector<Color> buffer;

 public:
  BitmapTexture(const std::string &name, const std::string &filePath);
  virtual const Color getColor(const Triangle &triangle, const Vector &barycentricCoordinates) const;
};
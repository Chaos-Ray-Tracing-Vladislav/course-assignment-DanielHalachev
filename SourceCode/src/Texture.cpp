#include "tracer/Texture.h"

#include <algorithm>
#include <iostream>

#include "stb_image.h"

#if (defined USE_TEXTURES) && USE_TEXTURES

Texture::Texture(const std::string &name) : name(name){};

AlbedoTexture::AlbedoTexture(const std::string &name, const Albedo &albedo) : Texture(name), albedo(albedo){};

const Color AlbedoTexture::getColor(const Triangle & /*triangle*/, const Vector & /*barycentricCoordinates*/) const {
  return this->albedo;
}

EdgeTexture::EdgeTexture(const std::string &name, const Color &innerColor, const Color &edgeColor, const float width)
    : Texture(name), innerColor(innerColor), edgeColor(edgeColor), width(width){};

const Color EdgeTexture::getColor(const Triangle & /*triangle*/, const Vector &barycentricCoordinates) const {
  if (barycentricCoordinates[0] < this->width || barycentricCoordinates[1] < this->width ||
      barycentricCoordinates[2] < this->width) {
    return this->edgeColor;
  }
  return this->innerColor;
}

CheckerTexture::CheckerTexture(const std::string &name, const Color &colorA, const Color &colorB,
                               const float squareSize)
    : Texture(name), colorA(colorA), colorB(colorB), squareSize{squareSize} {};

const Color CheckerTexture::getColor(const Triangle &triangle, const Vector &barycentricCoordinates) const {
  Vector interpolatedCoordinates = barycentricCoordinates[0] * triangle[1].UV +
                                   barycentricCoordinates[1] * triangle[2].UV +
                                   (barycentricCoordinates[2] * triangle[0].UV);

  unsigned int x = static_cast<unsigned int>(interpolatedCoordinates[0] / this->squareSize);
  unsigned int y = static_cast<unsigned int>(interpolatedCoordinates[1] / this->squareSize);
  if (x % 2 == y % 2) {
    return this->colorA;
  }
  return this->colorB;
}

BitmapTexture::BitmapTexture(const std::string &name, const std::string &filePath) : Texture(name) {
  unsigned char *image = stbi_load(filePath.c_str(), &width, &height, &channels, 0);
  if (image == nullptr) {
    std::cerr << stbi_failure_reason() << '\n';
  }
  size_t newSize = static_cast<size_t>(this->width) * this->height;
  this->buffer.resize(newSize);
  float coefficient = 1.0f / 255.0f;
  for (auto i = 0; i < newSize; i++) {
    this->buffer[i] = Color(static_cast<float>(image[this->channels * i + 0]) * coefficient,
                            static_cast<float>(image[this->channels * i + 1]) * coefficient,
                            static_cast<float>(image[this->channels * i + 2]) * coefficient);
  }
  stbi_image_free(image);
}

const Color BitmapTexture::getColor(const Triangle &triangle, const Vector &barycentricCoordinates) const {
  Vector interpolatedCoordinates = barycentricCoordinates[0] * triangle[1].UV +
                                   barycentricCoordinates[1] * triangle[2].UV +
                                   (barycentricCoordinates[2] * triangle[0].UV);

  int x =
      std::clamp(static_cast<int>(interpolatedCoordinates[0] * static_cast<float>(this->width)), 0, this->width - 1);
  int y = std::clamp(static_cast<int>((1.0f - interpolatedCoordinates[1]) * static_cast<float>(this->height)), 0,
                     this->height - 1);
  return this->buffer[y * width + x];
}
#endif  // USE_TEXTURES
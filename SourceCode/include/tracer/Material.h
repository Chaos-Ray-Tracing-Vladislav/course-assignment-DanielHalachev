#pragma once
#include "tracer/Color.h"
#if (defined USE_TEXTURES) && USE_TEXTURES
#include "tracer/Texture.h"
#endif  // USE_TEXTURES

enum MaterialType { Diffuse, Reflective, Constant, Refractive };

class Material {
#if (defined USE_TEXTURES) && USE_TEXTURES
 public:
  const Texture &texture;

 public:
  explicit Material(const Texture &texture, const Albedo &albedo, const MaterialType &type = Diffuse,
                    const bool smoothShading = false, const float ior = 1.0f);
  Material(const Material &other);
  Material(Material &&other) noexcept;
  Material &operator=(const Material &other) = delete;
  Material &operator=(Material &&other) noexcept = delete;
#else
 public:
  explicit Material(const Albedo &albedo, const MaterialType &type = Diffuse, const bool smoothShading = false,
                    const float ior = 1.0f);
#endif  // USE_TEXTURES
 public:
  Albedo albedo;
  MaterialType type;
  bool smoothShading;
  float ior;
};
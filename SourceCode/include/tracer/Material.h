#pragma once
#include "tracer/Color.h"
#if (defined USE_TEXTURES) && USE_TEXTURES
#include "tracer/Texture.h"
#endif  // USE_TEXTURES

enum MaterialType { Diffuse, Reflective, Constant, Refractive };

class Material {
#if (defined USE_TEXTURES) && USE_TEXTURES
 public:
  const Texture *texture = nullptr;

 public:
  explicit Material(const Texture &texture, const Albedo &albedo, const MaterialType &type = Diffuse,
                    const bool smoothShading = false, const float ior = 1.0f);
  Material(const Material &other);
  Material &operator=(const Material &other);
  Material &operator=(Material &&other) noexcept;
  Material(Material &&other) noexcept;
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

 public:
 public:
  Material();
  const Albedo &getAlbedo() const;
  const MaterialType getType() const;
  const bool hasSmoothShading() const;
  const float getIOR() const;
};
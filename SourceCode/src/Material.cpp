#include "tracer/Material.h"
const Albedo &Material::getAlbedo() const {
  return this->albedo;
}

const MaterialType Material::getType() const {
  return this->type;
}

const bool Material::hasSmoothShading() const {
  return this->smoothShading;
}

const float Material::getIOR() const {
  return this->ior;
}

#if (defined USE_TEXTURES) && USE_TEXTURES
Material::Material() : texture{nullptr}, albedo{Albedo()}, type{Diffuse}, smoothShading{false}, ior{1.0f} {};
Material::Material(const Texture &texture, const Albedo &albedo, const MaterialType &type, const bool smoothShading,
                   const float ior)
    : albedo(albedo), type(type), smoothShading(smoothShading), ior{ior} {
  this->texture = &texture;
};

Material::Material(const Material &other) {
  this->ior = other.ior;
  this->smoothShading = other.smoothShading;
  this->type = other.type;
  this->texture = other.texture;
}

Material &Material::operator=(const Material &other) {
  if (this != &other) {
    this->ior = other.ior;
    this->smoothShading = other.smoothShading;
    this->type = other.type;
    this->texture = other.texture;
  }
  return *this;
}

Material &Material::operator=(Material &&other) noexcept {
  if (this != &other) {
    this->ior = other.ior;
    this->smoothShading = other.smoothShading;
    this->type = other.type;
    this->texture = other.texture;
    other.texture = nullptr;
  }
  return *this;
}

Material::Material(Material &&other) noexcept {
  this->texture = other.texture;
  this->ior = other.ior;
  this->smoothShading = other.smoothShading;
  this->type = other.type;
  other.texture = nullptr;
}

#else
Material::Material() : albedo{Albedo()}, type{Diffuse}, smoothShading{false}, ior{1.0f} {};
Material::Material(const Albedo &albedo, const MaterialType &type, const bool smoothShading, const float ior)
    : albedo{albedo}, type(type), smoothShading(smoothShading), ior{ior} {};
#endif  // USE_TEXTURES
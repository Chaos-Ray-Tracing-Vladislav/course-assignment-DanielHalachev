#include "tracer/Material.h"

#if (defined USE_TEXTURES) && USE_TEXTURES
Material::Material(const Texture &texture, const Albedo &albedo, const MaterialType &type, const bool smoothShading,
                   const float ior)
    : texture{texture}, albedo{albedo}, type{type}, smoothShading{smoothShading}, ior{ior} {};

Material::Material(const Material &other)
    : texture{other.texture},
      albedo{other.albedo},
      type{other.type},
      smoothShading{other.smoothShading},
      ior{other.ior} {}

Material::Material(Material &&other) noexcept
    : texture{other.texture},
      albedo{other.albedo},
      type{other.type},
      smoothShading{other.smoothShading},
      ior{other.ior} {}

// Material &Material::operator=(const Material &other) {
//   if (this != &other) {
//     this->ior = other.ior;
//     this->smoothShading = other.smoothShading;
//     this->type = other.type;
//     this->texture = other.texture;
//   }
//   return *this;
// }

// Material &Material::operator=(Material &&other) noexcept {
//   if (this != &other) {
//     this->ior = other.ior;
//     this->smoothShading = other.smoothShading;
//     this->type = other.type;
//     this->texture = other.texture;
//     other.texture = nullptr;
//   }
//   return *this;
// }
#else
// Material::Material() : albedo{Albedo()}, type{Diffuse}, smoothShading{false}, ior{1.0f} {};
Material::Material(const Albedo &albedo, const MaterialType &type, const bool smoothShading, const float ior)
    : albedo{albedo}, type{type}, smoothShading{smoothShading}, ior{ior} {};
#endif  // USE_TEXTURES
#include "tracer/Color.h"

#include <algorithm>
#include <iostream>

#if (defined USE_TEXTURES) && USE_TEXTURES
#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG  // generate user friendly error messages
#include "stb_image.h"
#endif  // USE_TEXTURES

PPMColor::PPMColor(const Color &color) {
  this->red = static_cast<unsigned short>(std::clamp(color[0], 0.0f, 1.0f) * 255);
  this->green = static_cast<unsigned short>(std::clamp(color[1], 0.0f, 1.0f) * 255);
  this->blue = static_cast<unsigned short>(std::clamp(color[2], 0.0f, 1.0f) * 255);
}

std::ostream &operator<<(std::ostream &os, const struct PPMColor &color) {
  os << color.red << " " << color.green << " " << color.blue;
  return os;
}
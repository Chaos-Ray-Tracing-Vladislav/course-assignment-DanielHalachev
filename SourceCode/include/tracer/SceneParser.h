#pragma once
#include <string>
#include <vector>

#include "rapidjson/document.h"
#include "tracer/Camera.h"
#include "tracer/Scene.h"
#include "tracer/Texture.h"

class SceneParser {
 private:
  static const char* SETTINGS;
  static const char* BG_COLOR;
  static const char* IMAGE_SETTINGS;
  static const char* CAMERA;
  static const char* CAMERA_MATRIX;
  static const char* TEXTURES;
  static const char* MATERIALS;
  static const char* MATERIAL_INDEX;
  static const char* TYPE;
  static const char* ALBEDO;
  static const char* MATERIAL_SHADING;
  static const char* MATERIAL_IOR;
  static const char* LIGHTS;
  static const char* LIGHT_INTENSITY;
  static const char* POSITION;
  static const char* SCENE_OBJECTS;
  static const char* VERTICES;
  static const char* UVS;
  static const char* TRIANGLES;

  static SceneSettings parseSceneSettings(const rapidjson::Document& document);
  static Camera parseCameraSettings(const rapidjson::Document& document);
  static std::vector<Light> parseLightSettings(const rapidjson::Document& document);
  static std::vector<Texture*> parseTextures(const rapidjson::Document& document, const std::string& basePath);
  static std::vector<Material> parseMaterials(const rapidjson::Document& document,
                                              const std::vector<Texture*>& textures = {});
  static std::vector<Mesh> parseSceneObjects(const rapidjson::Document& document,
                                             const std::vector<Material>& materials);

 public:
  explicit SceneParser();
  static Scene parseScene(const std::string& pathToScene, const std::string& basePath = "");
};
#include "tracer/SceneParser.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <utility>
#include <vector>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "tracer/Scene.h"
#include "tracer/Utils.h"
#include "tracer/Vector.h"

const char* SceneParser::SETTINGS = "settings";
const char* SceneParser::BG_COLOR = "background_color";
const char* SceneParser::IMAGE_SETTINGS = "image_settings";
const char* SceneParser::CAMERA = "camera";
const char* SceneParser::CAMERA_MATRIX = "matrix";
const char* SceneParser::MATERIALS = "materials";
const char* SceneParser::MATERIAL_INDEX = "material_index";
const char* SceneParser::MATERIAL_TYPE = "type";
const char* SceneParser::MATERIAL_ALBEDO = "albedo";
const char* SceneParser::MATERIAL_SHADING = "smooth_shading";
const char* SceneParser::LIGHTS = "lights";
const char* SceneParser::LIGHT_INTENSITY = "intensity";
const char* SceneParser::POSITION = "position";
const char* SceneParser::SCENE_OBJECTS = "objects";
const char* SceneParser::VERTICES = "vertices";
const char* SceneParser::TRIANGLES = "triangles";

SceneParser::SceneParser() = default;

Scene SceneParser::parseScene(const std::string& pathToScene) {
  std::ifstream ifs(pathToScene);
  assert(ifs.is_open());

  rapidjson::IStreamWrapper isWrapper(ifs);
  rapidjson::Document document;
  document.ParseStream(isWrapper);

  SceneSettings sceneSettings = parseSceneSettings(document);
  Camera camera = parseCameraSettings(document);
  std::vector<Material> materials = parseMaterials(document);
  std::vector<Light> lights = parseLightSettings(document);
  std::vector<Mesh> objects = parseSceneObjects(document, materials);
  Scene result;
  result.sceneSettings = sceneSettings;
  result.camera = camera;
  result.materials = std::move(materials);
  result.lights = std::move(lights);
  result.objects = std::move(objects);
  return result;
  // return Scene{sceneSettings, camera, materials, lights, objects};
}

template <typename T>
std::vector<T> loadIntSTLVector(const rapidjson::Value::ConstArray& array, size_t expectedSize) {
  if (expectedSize != 0) {
    assert(array.Size() == expectedSize);
  }
  std::vector<T> result(expectedSize);
  std::transform(array.begin(), array.end(), result.begin(),
                 [](const auto value) { return static_cast<T>(value.GetUint64()); });
  return result;
}

std::vector<float> loadFloatSTLVector(const rapidjson::Value::ConstArray& array, size_t expectedSize) {
  if (expectedSize != 0) {
    assert(array.Size() == expectedSize);
  }
  std::vector<float> result(expectedSize);
  std::transform(array.begin(), array.end(), result.begin(), [](const auto& value) { return value.GetFloat(); });
  return result;
}

SceneSettings SceneParser::parseSceneSettings(const rapidjson::Document& document) {
  SceneSettings sceneSettings;
  const rapidjson::Value& settingsValue = document.FindMember(SceneParser::SETTINGS)->value;
  if (!settingsValue.IsNull() && settingsValue.IsObject()) {
    const rapidjson::Value& backgroundColorValue = settingsValue.FindMember(SceneParser::BG_COLOR)->value;
    assert(!backgroundColorValue.IsNull() && backgroundColorValue.IsArray());
    auto tempArray = backgroundColorValue.GetArray();
    sceneSettings.sceneBackgroundColor = Vector(loadFloatSTLVector(backgroundColorValue.GetArray(), 3));

    const rapidjson::Value& imageSettingsValue = settingsValue.FindMember(SceneParser::IMAGE_SETTINGS)->value;
    if (!imageSettingsValue.IsNull() && settingsValue.IsObject()) {
      const rapidjson::Value& imageWidthValue = imageSettingsValue.FindMember("width")->value;
      const rapidjson::Value& imageHeightValue = imageSettingsValue.FindMember("height")->value;
      assert(!imageWidthValue.IsNull() && imageWidthValue.IsInt());
      assert(!imageHeightValue.IsNull() && imageHeightValue.IsInt());
      sceneSettings.image = {imageWidthValue.GetUint(), imageHeightValue.GetUint()};
    }
  }
  return sceneSettings;
}

Camera SceneParser::parseCameraSettings(const rapidjson::Document& document) {
  Camera camera;

  const rapidjson::Value& cameraValue = document.FindMember(SceneParser::CAMERA)->value;
  if (!cameraValue.IsNull() && cameraValue.IsObject()) {
    const rapidjson::Value& cameraPositionValue = cameraValue.FindMember(SceneParser::POSITION)->value;
    assert(!cameraPositionValue.IsNull() && cameraPositionValue.IsArray());
    camera.setPosition() = Vector(loadFloatSTLVector(cameraPositionValue.GetArray(), 3));

    const rapidjson::Value& cameraMatrixValue = cameraValue.FindMember(SceneParser::CAMERA_MATRIX)->value;
    assert(!cameraMatrixValue.IsNull() && cameraMatrixValue.IsArray());
    camera.setRotationMatrix() = Matrix<3>(loadFloatSTLVector(cameraMatrixValue.GetArray(), 9));
  }
  return camera;
}

std::vector<Light> SceneParser::parseLightSettings(const rapidjson::Document& document) {
  std::vector<Light> lights;
  const rapidjson::Value& lightsValue = document.FindMember(SceneParser::LIGHTS)->value;
  if (!lightsValue.IsNull() && lightsValue.IsArray()) {
    lights.reserve(lightsValue.GetArray().Size());
    for (auto& light : lightsValue.GetArray()) {
      const rapidjson::Value& lightIntensityValue = light.FindMember(SceneParser::LIGHT_INTENSITY)->value;
      assert(!lightIntensityValue.IsNull() && lightIntensityValue.IsInt());
      const rapidjson::Value& lightPositionValue = light.FindMember(SceneParser::POSITION)->value;
      assert(!lightPositionValue.IsNull() && lightPositionValue.IsArray());
      Light temp = {Vector(loadFloatSTLVector(lightPositionValue.GetArray(), 3)), lightIntensityValue.GetUint()};
      lights.push_back(temp);
    }
  }
  return lights;
}

std::vector<Material> SceneParser::parseMaterials(const rapidjson::Document& document) {
  std::vector<Material> materials;
  const rapidjson::Value& materialsValue = document.FindMember(SceneParser::MATERIALS)->value;
  if (!materialsValue.IsNull() && materialsValue.IsArray()) {
    materials.reserve(materialsValue.GetArray().Size());
    for (auto& material : materialsValue.GetArray()) {
      const rapidjson::Value& materialTypeValue = material.FindMember(SceneParser::MATERIAL_TYPE)->value;
      assert(!materialTypeValue.IsNull() && materialTypeValue.IsString());
      const char* materialTypeString = materialTypeValue.GetString();
      MaterialType materialType;
      if (strcmp(materialTypeString, "diffuse") == 0) {
        materialType = Diffuse;
      } else if (strcmp(materialTypeString, "reflective") == 0) {
        materialType = Reflect;
      } else {
        throw "Invalid material";
      }
      const rapidjson::Value& materialAlbedoValue = material.FindMember(SceneParser::MATERIAL_ALBEDO)->value;
      assert(!materialAlbedoValue.IsNull() && materialAlbedoValue.IsArray());
      const rapidjson::Value& materialShadingValue = material.FindMember(SceneParser::MATERIAL_SHADING)->value;
      assert(!materialShadingValue.IsNull() && materialShadingValue.IsBool());
      Material temp(Albedo(loadFloatSTLVector(materialAlbedoValue.GetArray(), 3)), materialType,
                    materialShadingValue.GetBool());
      materials.push_back(temp);
    }
  }
  return materials;
}

std::vector<Mesh> SceneParser::parseSceneObjects(const rapidjson::Document& document,
                                                 std::vector<Material>& materials) {
  std::vector<Mesh> meshes;

  const rapidjson::Value& objectsValue = document.FindMember(SceneParser::SCENE_OBJECTS)->value;
  if (!objectsValue.IsNull() && objectsValue.IsArray()) {
    meshes.reserve(objectsValue.GetArray().Size());
    for (auto& object : objectsValue.GetArray()) {
      std::vector<Vertex> vertices;
      std::vector<unsigned int> triangleTriples;
      Material material;

      const rapidjson::Value& materialIndexValue = object.FindMember(SceneParser::MATERIAL_INDEX)->value;
      assert(!materialIndexValue.IsNull() && materialIndexValue.IsUint());
      material = materials[materialIndexValue.GetUint()];

      const rapidjson::Value& verticesValue = object.FindMember(SceneParser::VERTICES)->value;
      assert(!verticesValue.IsNull() && verticesValue.IsArray());
      auto verticesValueArray = verticesValue.GetArray();
      assert(verticesValueArray.Size() % 3 == 0);
      vertices.reserve(verticesValueArray.Size() / 3);
      for (size_t i = 0; i < verticesValueArray.Size(); i += 3) {
        vertices.push_back(Vertex(Vector(verticesValueArray[i].GetFloat(), verticesValueArray[i + 1].GetFloat(),
                                         verticesValueArray[i + 2].GetFloat())));
      }

      const rapidjson::Value& triangleTriplesValue = object.FindMember(SceneParser::TRIANGLES)->value;
      assert(!triangleTriplesValue.IsNull() && triangleTriplesValue.IsArray());
      auto triangleTempArray = triangleTriplesValue.GetArray();
      assert(triangleTempArray.Size() % 3 == 0);
      triangleTriples.reserve(triangleTempArray.Size());
      for (size_t i = 0; i < triangleTempArray.Size(); i++) {
        triangleTriples.push_back(triangleTempArray[i].GetUint());
      }
      meshes.push_back(Mesh{material, vertices, triangleTriples});
    }
  }
  return meshes;
}
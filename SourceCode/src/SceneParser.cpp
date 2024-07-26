#include "tracer/SceneParser.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <thread>
#include <utility>
#include <vector>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "tracer/Scene.h"
#include "tracer/Vector.h"

const char* SceneParser::SETTINGS = "settings";
const char* SceneParser::BG_COLOR = "background_color";
const char* SceneParser::IMAGE_SETTINGS = "image_settings";
const char* SceneParser::CAMERA = "camera";
const char* SceneParser::CAMERA_MATRIX = "matrix";
const char* SceneParser::TEXTURES = "textures";
const char* SceneParser::MATERIALS = "materials";
const char* SceneParser::MATERIAL_INDEX = "material_index";
const char* SceneParser::TYPE = "type";
const char* SceneParser::ALBEDO = "albedo";
const char* SceneParser::MATERIAL_SHADING = "smooth_shading";
const char* SceneParser::MATERIAL_IOR = "ior";
const char* SceneParser::LIGHTS = "lights";
const char* SceneParser::LIGHT_INTENSITY = "intensity";
const char* SceneParser::POSITION = "position";
const char* SceneParser::SCENE_OBJECTS = "objects";
const char* SceneParser::VERTICES = "vertices";
const char* SceneParser::UVS = "uvs";
const char* SceneParser::TRIANGLES = "triangles";

SceneParser::SceneParser() = default;

Scene SceneParser::parseScene(const std::string& pathToScene, const std::string& sceneFolder) {
  std::ifstream ifs((sceneFolder == "" ? "" : sceneFolder + "/") + pathToScene);
  assert(ifs.is_open());

  rapidjson::IStreamWrapper isWrapper(ifs);
  rapidjson::Document document;
  document.ParseStream(isWrapper);
  Scene result;

  SceneSettings sceneSettings = parseSceneSettings(document);
  result.sceneSettings = sceneSettings;
  Camera camera = parseCameraSettings(document);
  result.camera = camera;
#if (defined USE_TEXTURES) && USE_TEXTURES
  std::vector<Texture*> textures = parseTextures(document, sceneFolder);
  result.textures = std::move(textures);
  std::vector<Material> materials = parseMaterials(document, result.textures);
#else
  std::vector<Material> materials = parseMaterials(document);
#endif  // USE_TEXTURES
  result.materials = std::move(materials);
  std::vector<Light> lights = parseLightSettings(document);
  result.lights = std::move(lights);
  std::vector<Mesh> objects = parseSceneObjects(document, result.materials);
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
      const rapidjson::Value& bucketSizeValue = imageSettingsValue.FindMember("bucket_size")->value;
      assert(!imageWidthValue.IsNull() && imageWidthValue.IsInt());
      assert(!imageHeightValue.IsNull() && imageHeightValue.IsInt());
      unsigned int bucketSize =
          (std::thread::hardware_concurrency() == 1) ? (1) : (std::thread::hardware_concurrency() * 6);
      if (!bucketSizeValue.IsNull() && bucketSizeValue.IsInt()) {
        bucketSize = bucketSizeValue.GetInt();
      }
      sceneSettings.bucketSize = bucketSize;
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

#if (defined USE_TEXTURES) && USE_TEXTURES
std::vector<Texture*> SceneParser::parseTextures(const rapidjson::Document& document, const std::string& sceneFolder) {
  std::vector<Texture*> textures;
  const rapidjson::Value& texturesValue = document.FindMember(SceneParser::TEXTURES)->value;
  if (!texturesValue.IsNull() && texturesValue.IsArray()) {
    textures.reserve(texturesValue.GetArray().Size());
    for (auto& texture : texturesValue.GetArray()) {
      const rapidjson::Value& textureTypeValue = texture.FindMember(SceneParser::TYPE)->value;
      assert(!textureTypeValue.IsNull() && textureTypeValue.IsString());
      const char* textureTypeString = textureTypeValue.GetString();
      const rapidjson::Value& textureNameValue = texture.FindMember("name")->value;
      assert(!textureNameValue.IsNull() && textureNameValue.IsString());
      const std::string textureName = textureNameValue.GetString();

      if (strcmp(textureTypeString, "albedo") == 0) {
        const rapidjson::Value& textureAlbedoValue = texture.FindMember(SceneParser::ALBEDO)->value;
        assert(!textureAlbedoValue.IsNull() && textureAlbedoValue.IsArray());
        Albedo albedo = Albedo(loadFloatSTLVector(textureAlbedoValue.GetArray(), 3));
        textures.push_back(new AlbedoTexture(textureName, albedo));
      } else if (strcmp(textureTypeString, "edges") == 0) {
        const rapidjson::Value& innerColorValue = texture.FindMember("inner_color")->value;
        assert(!innerColorValue.IsNull() && innerColorValue.IsArray());
        Color innerColor = Color(loadFloatSTLVector(innerColorValue.GetArray(), 3));

        const rapidjson::Value& edgeColorValue = texture.FindMember("edge_color")->value;
        assert(!edgeColorValue.IsNull() && edgeColorValue.IsArray());
        Color edgeColor = Color(loadFloatSTLVector(edgeColorValue.GetArray(), 3));

        const rapidjson::Value& edgeWidthValue = texture.FindMember("edge_width")->value;
        assert(!edgeWidthValue.IsNull() && edgeWidthValue.IsFloat());
        float edgeWidth = edgeWidthValue.GetFloat();

        textures.push_back(new EdgeTexture(textureName, innerColor, edgeColor, edgeWidth));
      } else if (strcmp(textureTypeString, "checker") == 0) {
        const rapidjson::Value& colorValueA = texture.FindMember("color_A")->value;
        assert(!colorValueA.IsNull() && colorValueA.IsArray());
        Color colorA = Color(loadFloatSTLVector(colorValueA.GetArray(), 3));

        const rapidjson::Value& colorValueB = texture.FindMember("color_B")->value;
        assert(!colorValueB.IsNull() && colorValueB.IsArray());
        Color colorB = Color(loadFloatSTLVector(colorValueB.GetArray(), 3));

        const rapidjson::Value& squareSizeValue = texture.FindMember("square_size")->value;
        assert(!squareSizeValue.IsNull() && squareSizeValue.IsFloat());
        float squareSize = squareSizeValue.GetFloat();

        textures.push_back(new CheckerTexture(textureName, colorA, colorB, squareSize));
      } else if (strcmp(textureTypeString, "bitmap") == 0) {
        const rapidjson::Value& pathValue = texture.FindMember("file_path")->value;
        assert(!pathValue.IsNull() && pathValue.IsString());
        std::string path = pathValue.GetString();

        textures.push_back(new BitmapTexture(textureName, sceneFolder + path));
      } else {
        throw "Invalid material";
      }
    }
  }
  return textures;
}
#endif  // USE_TEXTURES

std::vector<Material> SceneParser::parseMaterials(const rapidjson::Document& document,
                                                  const std::vector<Texture*>& textures) {
  std::vector<Material> materials;
  const rapidjson::Value& materialsValue = document.FindMember(SceneParser::MATERIALS)->value;
  if (!materialsValue.IsNull() && materialsValue.IsArray()) {
    materials.reserve(materialsValue.GetArray().Size());
    for (auto& material : materialsValue.GetArray()) {
      const rapidjson::Value& materialTypeValue = material.FindMember(SceneParser::TYPE)->value;
      assert(!materialTypeValue.IsNull() && materialTypeValue.IsString());
      const char* materialTypeString = materialTypeValue.GetString();
      MaterialType materialType;
      float IOR = 0;
      Albedo albedo(0, 0, 0);

      if (strcmp(materialTypeString, "diffuse") == 0) {
        materialType = Diffuse;
      } else if (strcmp(materialTypeString, "reflective") == 0) {
        materialType = Reflective;
      } else if (strcmp(materialTypeString, "refractive") == 0) {
        materialType = Refractive;
        const rapidjson::Value& materialIORValue = material.FindMember(SceneParser::MATERIAL_IOR)->value;
        assert(!materialIORValue.IsNull() && materialIORValue.IsFloat());
        IOR = materialIORValue.GetFloat();
      } else if (strcmp(materialTypeString, "constant") == 0) {
        materialType = Constant;
      } else {
        throw "Invalid material";
      }
      const rapidjson::Value& materialShadingValue = material.FindMember(SceneParser::MATERIAL_SHADING)->value;
      assert(!materialShadingValue.IsNull() && materialShadingValue.IsBool());
#if (defined USE_TEXTURES) && USE_TEXTURES
      const rapidjson::Value& textureNameValue = material.FindMember("albedo")->value;
      assert(!textureNameValue.IsNull() && textureNameValue.IsString());
      const std::string textureName = textureNameValue.GetString();
      Texture* texture = nullptr;
      for (Texture* t : textures) {
        if (t->name == textureName) {
          texture = t;
          break;
        }
      }
      if (materialType != Diffuse) {
        const rapidjson::Value& materialAlbedoValue = material.FindMember(SceneParser::ALBEDO)->value;
        assert(!materialAlbedoValue.IsNull() && materialAlbedoValue.IsArray());
        albedo = Albedo(loadFloatSTLVector(materialAlbedoValue.GetArray(), 3));
      }
      Material temp(*texture, albedo, materialType, materialShadingValue.GetBool(), IOR);
      materials.push_back(temp);
#else
      if (materialType != Refractive) {
        const rapidjson::Value& materialAlbedoValue = material.FindMember(SceneParser::ALBEDO)->value;
        assert(!materialAlbedoValue.IsNull() && materialAlbedoValue.IsArray());
        albedo = Albedo(loadFloatSTLVector(materialAlbedoValue.GetArray(), 3));
      }
      Material temp(albedo, materialType, materialShadingValue.GetBool(), IOR);
      materials.push_back(temp);
#endif  // USE_TEXTURES
    }
  }
  return materials;
}

std::vector<Mesh> SceneParser::parseSceneObjects(const rapidjson::Document& document,
                                                 const std::vector<Material>& materials) {
  std::vector<Mesh> meshes;

  const rapidjson::Value& objectsValue = document.FindMember(SceneParser::SCENE_OBJECTS)->value;
  if (!objectsValue.IsNull() && objectsValue.IsArray()) {
    meshes.reserve(objectsValue.GetArray().Size());
    for (auto& object : objectsValue.GetArray()) {
      std::vector<Vertex> vertices;
      std::vector<unsigned int> triangleTriples;

      const rapidjson::Value& materialIndexValue = object.FindMember(SceneParser::MATERIAL_INDEX)->value;
      assert(!materialIndexValue.IsNull() && materialIndexValue.IsUint());
      const Material& material = materials[materialIndexValue.GetUint()];

      const rapidjson::Value& verticesValue = object.FindMember(SceneParser::VERTICES)->value;
      assert(!verticesValue.IsNull() && verticesValue.IsArray());
      auto verticesValueArray = verticesValue.GetArray();
      assert(verticesValueArray.Size() % 3 == 0);
      vertices.reserve(verticesValueArray.Size() / 3);
      for (size_t i = 0; i < verticesValueArray.Size(); i += 3) {
        vertices.push_back(Vertex(Vector(verticesValueArray[i].GetFloat(), verticesValueArray[i + 1].GetFloat(),
                                         verticesValueArray[i + 2].GetFloat())));
      }

#if (defined USE_TEXTURES) && USE_TEXTURES
      const rapidjson::Value& UVsValue = object.FindMember(SceneParser::UVS)->value;
      assert(!UVsValue.IsNull() && UVsValue.IsArray());
      auto uvValuesArray = UVsValue.GetArray();
      assert(uvValuesArray.Size() % 3 == 0);
      for (size_t i = 0; i < uvValuesArray.Size(); i += 3) {
        Vector temp =
            Vector(uvValuesArray[i].GetFloat(), uvValuesArray[i + 1].GetFloat(), uvValuesArray[i + 2].GetFloat());
        vertices[i / 3].UV = temp;
      }
#endif  // USE_TEXTURES

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
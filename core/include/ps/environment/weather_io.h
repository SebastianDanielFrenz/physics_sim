// ps/environment/weather_io.h - loaders for physics_sim.weather_preset/1
// (docs/data_formats/weather_preset.md) and physics_sim.weather_classes/1
// (docs/data_formats/weather_classes.md), weather plan W3. Both resolve
// `extends` (json_extends.h) and reject unknown keys. Library
// ps_environment_io (nlohmann PRIVATE).

#pragma once

#include "ps/environment/weather_classes.h"
#include "ps/environment/weather_desc.h"

#include <nlohmann/json_fwd.hpp>

#include <filesystem>
#include <string>

namespace ps::io {

environment::WeatherPresetDesc load_weather_preset_json(const std::filesystem::path& path);
environment::WeatherPresetDesc parse_weather_preset_json(const nlohmann::json& root, const std::string& origin);

environment::WeatherClassesDesc load_weather_classes_json(const std::filesystem::path& path);
environment::WeatherClassesDesc parse_weather_classes_json(const nlohmann::json& root, const std::string& origin);

} // namespace ps::io

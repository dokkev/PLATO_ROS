#include "plato_hardware_interface/utils/plato_hand_config_loader.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "plato_hardware_interface/utils/actuator_config_loader.hpp"
#include "plato_hardware_interface/utils/actuator_offset_loader.hpp"
#include "plato_hardware_interface/utils/linkage_config_loader.hpp"

namespace plato_hand
{

namespace
{

std::string default_config_dir()
{
  return ament_index_cpp::get_package_share_directory("plato_hardware_interface") + "/config";
}

}  // namespace

PlatoHandConfig load_default_plato_hand_config()
{
  return load_plato_hand_config(default_config_dir());
}

PlatoHandConfig load_plato_hand_config(const std::string & config_dir)
{
  const std::filesystem::path config_path(config_dir);
  auto actuator_configs =
    plato_actuator::load_plato_actuator_configs((config_path / "plato_actuators.yaml").string());
  const auto actuator_offsets =
    plato_actuator::load_plato_actuator_position_offsets(
    (config_path / "plato_actuator_offsets.yaml").string());

  if (actuator_configs.size() != actuator_offsets.size()) {
    throw std::runtime_error("Actuator config and actuator offset YAML size mismatch");
  }

  for (size_t i = 0; i < actuator_configs.size(); ++i) {
    actuator_configs[i].core.position_offset = actuator_offsets[i];
  }

  return PlatoHandConfig{
    std::move(actuator_configs),
    FiveBarLinkage::load_plato_linkage_config((config_path / "plato_linkage.yaml").string()),
    (config_path / "plato_actuator_offsets.yaml").string(),
  };
}

}  // namespace plato_hand

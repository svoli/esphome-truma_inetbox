#include "TrumaRoomClimate.h"
#include "esphome/components/truma_inetbox/helpers.h"

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.room_climate";

// Setup: Callback registrieren, Status vom Heizgerät abholen
void TrumaRoomClimate::setup() {
  if (!this->parent_) return;

  this->parent_->get_heater()->add_on_message_callback([this](const StatusFrameHeater *status_heater) {
    // Temperaturen umrechnen
    this->target_temperature = temp_code_to_decimal(status_heater->target_temp_room);
    this->current_temperature = temp_code_to_decimal(status_heater->current_temp_room);

    // Modus bestimmen
    this->mode = std::isnan(this->target_temperature) ? climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;

    // Fan-Mode basierend auf Heizmodus
    switch (status_heater->heating_mode) {
      case HeatingMode::HEATING_MODE_ECO:
        this->fan_mode = climate::CLIMATE_FAN_LOW;
        break;
      case HeatingMode::HEATING_MODE_HIGH:
        this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
        break;
      case HeatingMode::HEATING_MODE_BOOST:
        this->fan_mode = climate::CLIMATE_FAN_HIGH;
        break;
      default:
        this->fan_mode = climate::CLIMATE_FAN_OFF;
        break;
    }

    // Status an ESPHome veröffentlichen
    this->publish_state();
  });
}

// Dump config
void TrumaRoomClimate::dump_config() {
  LOG_CLIMATE(TAG, "Truma Room Climate", this);
}

// Steuerung aus ESPHome
void TrumaRoomClimate::control(const climate::ClimateCall &call) {
  if (!this->parent_) return;
  auto heater = this->parent_->get_heater();
  auto status_heater = heater->get_status();

  // Temperatur setzen, falls kein Fan-Modus angegeben
  if (call.get_target_temperature().has_value() && !call.get_fan_mode().has_value()) {
    float temp = *call.get_target_temperature();
    heater->action_heater_room(static_cast<uint8_t>(temp));
  }

  // Modus setzen
  if (call.get_mode().has_value()) {
    climate::ClimateMode mode = *call.get_mode();
    switch (mode) {
      case climate::CLIMATE_MODE_HEAT:
        if (status_heater->target_temp_room == TargetTemp::TARGET_TEMP_OFF) {
          heater->action_heater_room(5);
        }
        break;
      default:
        heater->action_heater_room(0);
        break;
    }
  }

  // Fan-Modus setzen
  if (call.get_fan_mode().has_value()) {
    climate::ClimateFanMode fan_mode = *call.get_fan_mode();
    float temp = temp_code_to_decimal(status_heater->target_temp_room, 0);
    if (call.get_target_temperature().has_value()) {
      temp = *call.get_target_temperature();
    }

    if (fan_mode == climate::CLIMATE_FAN_LOW || fan_mode == climate::CLIMATE_FAN_MEDIUM || fan_mode == climate::CLIMATE_FAN_HIGH) {
      if (temp < 5) temp = 5;
    }

    switch (fan_mode) {
      case climate::CLIMATE_FAN_LOW:
        heater->action_heater_room(static_cast<uint8_t>(temp), HeatingMode::HEATING_MODE_ECO);
        break;
      case climate::CLIMATE_FAN_MEDIUM:
        heater->action_heater_room(static_cast<uint8_t>(temp), HeatingMode::HEATING_MODE_HIGH);
        break;
      case climate::CLIMATE_FAN_HIGH:
        heater->action_heater_room(static_cast<uint8_t>(temp), HeatingMode::HEATING_MODE_BOOST);
        break;
      default:
        heater->action_heater_room(0);
        break;
    }
  }

  // Preset-Unterstützung auskommentiert, kann später aktiviert werden
  /*
  if (call.get_preset().has_value()) {
    climate::ClimatePreset pres = *call.get_preset();
    float temp = temp_code_to_decimal(status_heater->target_temp_room);
    if (call.get_target_temperature().has_value()) {
      temp = *call.get_target_temperature();
    }

    switch (pres) {
      case climate::CLIMATE_PRESET_ECO:
        heater->action_heater_room(temp, HeatingMode::HEATING_MODE_ECO);
        break;
      case climate::CLIMATE_PRESET_COMFORT:
        heater->action_heater_room(temp, HeatingMode::HEATING_MODE_HIGH);
        break;
      case climate::CLIMATE_PRESET_BOOST:
        heater->action_heater_room(temp, HeatingMode::HEATING_MODE_BOOST);
        break;
      default:
        heater->action_heater_room(0);
        break;
    }
  }
  */
}

// Traits definieren (ESPHome 2026-kompatibel)
climate::ClimateTraits TrumaRoomClimate::traits() {
  climate::ClimateTraits traits;

  // Aktuelle Temperatur unterstützen
  traits.add_feature_flags(climate::ClimateFeature::SUPPORTS_CURRENT_TEMPERATURE);

  // Unterstützte Modi
  climate::ClimateModeMask mode_mask;
  for (auto mode : supported_modes_) {
    mode_mask.insert(mode);
  }
  traits.set_supported_modes(mode_mask);

  // Unterstützte Fan-Modes
  traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_OFF,
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
  });

  // Visualisierung
  traits.set_visual_min_temperature(visual_min_temperature_);
  traits.set_visual_max_temperature(visual_max_temperature_);
  traits.set_visual_temperature_step(visual_temperature_step_);

  return traits;
}

// Setzen der unterstützten Modi
void TrumaRoomClimate::set_supported_modes(const std::set<climate::ClimateMode> &modes) {
  this->supported_modes_ = modes;
}

}  // namespace truma_inetbox
}  // namespace esphome

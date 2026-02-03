#include "TrumaWaterClimate.h"
#include "esphome/components/truma_inetbox/helpers.h"
#include <set>

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.water_climate";

// Setup: Callback registrieren, Status vom Heizgerät abholen
void TrumaWaterClimate::setup() {
  if (!this->parent_) return;

  this->parent_->get_heater()->add_on_message_callback([this](const StatusFrameHeater *status_heater) {
    // Aktuelle Werte aus Statusframe übernehmen
    this->target_temperature = water_temp_200_fix(temp_code_to_decimal(status_heater->target_temp_water));
    this->current_temperature = temp_code_to_decimal(status_heater->current_temp_water);
    this->mode = (status_heater->target_temp_water == TargetTemp::TARGET_TEMP_OFF) ?
                 climate::CLIMATE_MODE_OFF : climate::CLIMATE_MODE_HEAT;

    this->publish_state();
  });
}

// Dump config
void TrumaWaterClimate::dump_config() {
  LOG_CLIMATE(TAG, "Truma Water Climate", this);
}

// Steuerung aus ESPHome
void TrumaWaterClimate::control(const climate::ClimateCall &call) {
  if (!this->parent_) return;
  auto heater = this->parent_->get_heater();
  auto status_heater = heater->get_status();

  // Temperatur setzen
  if (call.get_target_temperature().has_value()) {
    float temp = *call.get_target_temperature();
    heater->action_heater_water(static_cast<uint8_t>(temp));
  }

  // Modus setzen
  if (call.get_mode().has_value()) {
    climate::ClimateMode mode = *call.get_mode();
    switch (mode) {
      case climate::CLIMATE_MODE_HEAT:
        if (status_heater->target_temp_water == TargetTemp::TARGET_TEMP_OFF) {
          heater->action_heater_water(40);  // Default-Wassertemperatur
        }
        break;
      default:
        heater->action_heater_water(0);  // AUS
        break;
    }
  }
}

// Traits definieren (ESPHome 2026-kompatibel)
climate::ClimateTraits TrumaWaterClimate::traits() {
  climate::ClimateTraits traits;

  // Aktuelle Temperatur unterstützen
  traits.set_supports_current_temperature(true);

  // Unterstützte Modi
  climate::ClimateModeMask mode_mask;
  for (auto mode : supported_modes_) {
    mode_mask.insert(mode);
  }
  traits.set_supported_modes(mode_mask);

  // Visualisierung
  traits.set_visual_min_temperature(40);
  traits.set_visual_max_temperature(80);
  traits.set_visual_temperature_step(20);

  return traits;
}

// Setzen der unterstützten Modi
void TrumaWaterClimate::set_supported_modes(const std::set<climate::ClimateMode> &modes) {
  this->supported_modes_ = modes;
}

}  // namespace truma_inetbox
}  // namespace esphome

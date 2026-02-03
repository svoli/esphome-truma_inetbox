#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/core/component.h"
#include "esphome/components/truma_inetbox/TrumaiNetBoxApp.h"
#include <set>  // Für std::set

namespace esphome {
namespace truma_inetbox {

class TrumaRoomClimate : public Component, public climate::Climate, public Parented<TrumaiNetBoxApp> {
 public:
  // Lifecycle
  void setup() override;
  void loop() override {}
  void dump_config() override;

  // Steuerung aus ESPHome
  void control(const climate::ClimateCall &call) override;

  // Traits definieren
  climate::ClimateTraits traits() override;

  // Visualisierungseinstellungen
  void set_visual_min_temperature(float value) { this->visual_min_temperature_ = value; }
  void set_visual_max_temperature(float value) { this->visual_max_temperature_ = value; }
  void set_visual_temperature_step(float value) { this->visual_temperature_step_ = value; }

  // Unterstützte Modi setzen
  void set_supported_modes(const std::set<climate::ClimateMode> &modes);

 protected:
  // Unterstützte Modi
  std::set<climate::ClimateMode> supported_modes_;

  // Visualisierung von Temperatur
  float visual_min_temperature_{5.0};
  float visual_max_temperature_{30.0};
  float visual_temperature_step_{1.0};

  // Interne Hilfsmethoden können hier noch ergänzt werden, falls benötigt
};

}  // namespace truma_inetbox
}  // namespace esphome

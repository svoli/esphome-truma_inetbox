#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/core/component.h"
#include "esphome/components/truma_inetbox/TrumaiNetBoxApp.h"
#include <set>  // Für std::set

namespace esphome {
namespace truma_inetbox {

class TrumaWaterClimate : public Component, public climate::Climate, public Parented<TrumaiNetBoxApp> {
 public:
  // Lifecycle
  void setup() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;

  // Unterstützte Modi setzen
  void set_supported_modes(const std::set<climate::ClimateMode> &modes);

  // Optional: Visualisierung der Temperaturgrenzen
  void set_visual_min_temperature(float value) { this->visual_min_temperature_ = value; }
  void set_visual_max_temperature(float value) { this->visual_max_temperature_ = value; }
  void set_visual_temperature_step(float value) { this->visual_temperature_step_ = value; }

 protected:
  // Intern: unterstützte Modi
  std::set<esphome::climate::ClimateMode> supported_modes_;

  // Visualisierung der Temperatur in der UI
  float visual_min_temperature_{40.0};
  float visual_max_temperature_{80.0};
  float visual_temperature_step_{20.0};
};

}  // namespace truma_inetbox
}  // namespace esphome

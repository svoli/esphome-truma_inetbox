#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/truma_inetbox/TrumaiNetBoxApp.h"
#include <set>  // Für std::set

namespace esphome {
namespace truma_inetbox {

class TrumaRoomClimate : public climate::Climate, public TrumaiNetBoxApp {
 public:
  // Konstruktor
  TrumaRoomClimate(TrumaiNetBoxApp* app);

  // Lifecycle
  void setup() override;
  void loop() override;

  // ESPHome 2026-kompatible Traits-Funktion
  climate::ClimateTraits traits() override;

  // Methoden zum Setzen der unterstützten Modi
  void set_supported_modes(const std::set<climate::ClimateMode> &modes);

 protected:
  // Interne Hilfsmethoden für Mode-Konvertierung etc.
  void update_state_from_app();
  void send_mode_to_app(climate::ClimateMode mode);

 private:
  // Interne Variablen
  std::set<climate::ClimateMode> supported_modes_;  // nur zur internen Verwaltung
  TrumaiNetBoxApp* app_;
};

}  // namespace truma_inetbox
}  // namespace esphome

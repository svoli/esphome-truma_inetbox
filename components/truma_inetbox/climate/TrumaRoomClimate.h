#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/truma_inetbox/TrumaiNetBoxApp.h"

namespace esphome {
namespace truma_inetbox {

class TrumaRoomClimate : public climate::Climate {
 public:
  void setup() override;
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

 private:
  // Alte std::set supported_modes_ entfällt
};

}  // namespace truma_inetbox
}  // namespace esphome

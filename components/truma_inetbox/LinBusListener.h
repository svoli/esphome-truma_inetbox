#pragma once

#include "LinBusLog.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

#ifdef USE_RP2040
#include <hardware/uart.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#endif

#ifndef TRUMA_MSG_QUEUE_LENGTH
#define TRUMA_MSG_QUEUE_LENGTH 6
#endif

#ifndef TRUMA_LOG_QUEUE_LENGTH
#define TRUMA_LOG_QUEUE_LENGTH 6
#endif

namespace esphome {
namespace truma_inetbox {

enum class LIN_CHECKSUM {
  LIN_CHECKSUM_VERSION_1,
  LIN_CHECKSUM_VERSION_2
};

struct QUEUE_LIN_MSG {
  uint8_t current_PID;
  uint8_t data[8];
  uint8_t len;
};

class LinBusListener : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }

  void setup() override;
  void dump_config() override;
  void loop() override;

  void set_lin_checksum(LIN_CHECKSUM val) { this->lin_checksum_ = val; }
  void set_cs_pin(GPIOPin *pin) { this->cs_pin_ = pin; }
  void set_fault_pin(GPIOPin *pin) { this->fault_pin_ = pin; }
  void set_observer_mode(bool val) { this->observer_mode_ = val; }
  bool get_lin_bus_fault() { return fault_on_lin_bus_reported_ > 3; }

  void process_lin_msg_queue(TickType_t xTicksToWait);
  void process_log_queue(TickType_t xTicksToWait);

 protected:
  // ===== UART Byte Handling =====
  void on_byte_(uint8_t byte);

  // ===== Configuration =====
  LIN_CHECKSUM lin_checksum_ = LIN_CHECKSUM::LIN_CHECKSUM_VERSION_2;
  GPIOPin *cs_pin_ = nullptr;
  GPIOPin *fault_pin_ = nullptr;
  bool observer_mode_ = false;

  // ===== LIN helpers =====
  void write_lin_answer_(const uint8_t *data, uint8_t len);
  bool check_for_lin_fault_();
  virtual bool answer_lin_order_(const uint8_t pid) = 0;
  virtual void lin_message_recieved_(const uint8_t pid,
                                     const uint8_t *message,
                                     uint8_t length) = 0;

 private:
  // ===== Timing =====
  uint32_t time_per_baud_;
  const uint8_t lin_break_length = 13;
  uint32_t time_per_lin_break_;
  const uint8_t frame_length_ = (8 + 1 + 2);
  uint32_t time_per_pid_;
  uint32_t time_per_first_byte_;
  uint32_t time_per_byte_;

  // ===== Fault handling =====
  uint8_t fault_on_lin_bus_reported_ = 0;
  bool can_write_lin_answer_ = false;

  // ===== LIN state machine =====
  enum read_state {
    READ_STATE_BREAK,
    READ_STATE_SYNC,
    READ_STATE_SID,
    READ_STATE_DATA,
    READ_STATE_ACT,
  };

  read_state current_state_ = READ_STATE_BREAK;
  uint8_t current_PID_with_parity_ = 0x00;
  uint8_t current_PID_ = 0x00;
  bool current_PID_order_answered_ = false;
  bool current_data_valid_ = true;
  uint8_t current_data_count_ = 0;
  uint8_t current_data_[9] = {};
  uint32_t last_data_recieved_ = 0;

  void current_state_reset_() {
    current_state_ = READ_STATE_BREAK;
    current_PID_with_parity_ = 0x00;
    current_PID_ = 0x00;
    current_PID_order_answered_ = false;
    current_data_valid_ = true;
    current_data_count_ = 0;
    memset(current_data_, 0, sizeof(current_data_));
  }

  void read_lin_frame_();
  void clear_uart_buffer_();

  // ===== Queues =====
  uint8_t lin_msg_static_queue_storage[TRUMA_MSG_QUEUE_LENGTH * sizeof(QUEUE_LIN_MSG)];
  StaticQueue_t lin_msg_static_queue_;
  QueueHandle_t lin_msg_queue_ =
      xQueueCreateStatic(TRUMA_MSG_QUEUE_LENGTH,
                         sizeof(QUEUE_LIN_MSG),
                         lin_msg_static_queue_storage,
                         &lin_msg_static_queue_);

#if ESPHOME_LOG_LEVEL > ESPHOME_LOG_LEVEL_NONE
  uint8_t log_static_queue_storage[TRUMA_LOG_QUEUE_LENGTH * sizeof(QUEUE_LOG_MSG)];
  StaticQueue_t log_static_queue_;
  QueueHandle_t log_queue_ =
      xQueueCreateStatic(TRUMA_LOG_QUEUE_LENGTH,
                         sizeof(QUEUE_LOG_MSG),
                         log_static_queue_storage,
                         &log_static_queue_);
#endif
};

}  // namespace truma_inetbox
}  // namespace esphome

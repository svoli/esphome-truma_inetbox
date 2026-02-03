#include "LinBusListener.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "helpers.h"

namespace esphome {
namespace truma_inetbox {

static const char *const TAG = "truma_inetbox.LinBusListener";

#define LIN_BREAK 0x00
#define LIN_SYNC 0x55
#define DIAGNOSTIC_FRAME_MASTER 0x3c
#define DIAGNOSTIC_FRAME_SLAVE 0x3d
#define QUEUE_WAIT_DONT_BLOCK (TickType_t) 0

void LinBusListener::dump_config() {
  ESP_LOGCONFIG(TAG, "LinBusListener:");
  LOG_PIN("  CS Pin: ", this->cs_pin_);
  LOG_PIN("  FAULT Pin: ", this->fault_pin_);
  ESP_LOGCONFIG(TAG, "  LIN checksum version: %d",
                this->lin_checksum_ == LIN_CHECKSUM::LIN_CHECKSUM_VERSION_1 ? 1 : 2);
  ESP_LOGCONFIG(TAG, "  Observer mode: %s", YESNO(this->observer_mode_));
  this->check_uart_settings(9600, 2, uart::UART_CONFIG_PARITY_NONE, 8);
}

void LinBusListener::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LIN BUS...");

  this->time_per_baud_ = (1000000.0f / this->parent_->get_baud_rate());
  this->time_per_lin_break_ = this->time_per_baud_ * this->lin_break_length * 1.1f;
  this->time_per_pid_ = this->time_per_baud_ * this->frame_length_ * 1.1f;
  this->time_per_first_byte_ = this->time_per_baud_ * this->frame_length_ * 5.0f;
  this->time_per_byte_ = this->time_per_baud_ * this->frame_length_ * 1.1f;

  if (this->cs_pin_) {
    this->cs_pin_->setup();
    this->cs_pin_->digital_write(!this->observer_mode_);
  }

  if (this->fault_pin_) {
    this->fault_pin_->setup();
  }

#if ESPHOME_LOG_LEVEL > ESPHOME_LOG_LEVEL_NONE
  this->set_interval("logmsg", 50, [this]() {
    this->process_log_queue(QUEUE_WAIT_DONT_BLOCK);
  });
#endif
}

void LinBusListener::loop() {
  while (this->available()) {
    uint8_t byte;
    if (this->read_byte(&byte)) {
      this->on_byte_(byte);
    }
  }
}

void LinBusListener::on_byte_(uint8_t byte) {
  if (this->check_for_lin_fault_()) {
    return;
  }

  this->last_data_recieved_ = micros();
  this->read_lin_frame_();
}

bool LinBusListener::check_for_lin_fault_() {
  if (!this->fault_pin_) {
    return false;
  }

  if (!this->fault_pin_->digital_read()) {
    if (this->fault_on_lin_bus_reported_ < 0xFF) {
      this->fault_on_lin_bus_reported_++;
    }
    if (this->fault_on_lin_bus_reported_ % 3 == 0) {
      QUEUE_LOG_MSG log_msg{};
      log_msg.type = QUEUE_LOG_MSG_TYPE::ERROR_CHECK_FOR_LIN_FAULT_DETECTED;
      TRUMA_LOGE(log_msg);
    }
  } else {
    if (this->get_lin_bus_fault()) {
      QUEUE_LOG_MSG log_msg{};
      log_msg.type = QUEUE_LOG_MSG_TYPE::INFO_CHECK_FOR_LIN_FAULT_FIXED;
      TRUMA_LOGI(log_msg);
    }
    this->fault_on_lin_bus_reported_ = 0;
  }

  if (this->get_lin_bus_fault()) {
    this->current_state_reset_();
    this->clear_uart_buffer_();
    return true;
  }

  return false;
}

/* ================= LIN STATE MACHINE ================= */

void LinBusListener::read_lin_frame_() {
  uint8_t buf;

  switch (this->current_state_) {
    case READ_STATE_BREAK:
      this->current_state_reset_();
      if (!this->read_byte(&buf)) return;

      if (buf == LIN_BREAK) {
        this->current_state_ = READ_STATE_SYNC;
      } else if (buf == LIN_SYNC) {
        this->current_state_ = READ_STATE_SID;
      }
      break;

    case READ_STATE_SYNC:
      if (!this->read_byte(&buf) || buf != LIN_SYNC) {
        this->current_state_ = READ_STATE_BREAK;
      } else {
        this->current_state_ = READ_STATE_SID;
      }
      break;

    case READ_STATE_SID:
      if (!this->read_byte(&this->current_PID_with_parity_)) return;

      this->current_PID_ = this->current_PID_with_parity_ & 0x3F;
      this->current_data_valid_ = true;

      if (this->lin_checksum_ == LIN_CHECKSUM::LIN_CHECKSUM_VERSION_2) {
        if (this->current_PID_with_parity_ !=
            (this->current_PID_ | (addr_parity(this->current_PID_) << 6))) {
          this->current_data_valid_ = false;
        }
      }

      if (this->current_data_valid_) {
        this->can_write_lin_answer_ = true;
        this->answer_lin_order_(this->current_PID_);
        this->can_write_lin_answer_ = false;
      }

      this->current_state_ = READ_STATE_DATA;
      break;

    case READ_STATE_DATA:
      if (!this->read_byte(&buf)) return;

      this->current_data_[this->current_data_count_++] = buf;

      if (this->current_data_count_ >= sizeof(this->current_data_)) {
        this->current_state_ = READ_STATE_ACT;
      }
      break;

    case READ_STATE_ACT:
      if (this->current_data_count_ > 1 && this->current_data_valid_) {
        QUEUE_LIN_MSG lin_msg{};
        lin_msg.current_PID = this->current_PID_;
        lin_msg.len = this->current_data_count_ - 1;
        memcpy(lin_msg.data, this->current_data_, lin_msg.len);
        xQueueSend(this->lin_msg_queue_, &lin_msg, 0);
      }
      this->current_state_ = READ_STATE_BREAK;
      break;
  }
}

void LinBusListener::clear_uart_buffer_() {
  uint8_t dummy;
  while (this->available()) {
    this->read_byte(&dummy);
  }
}

void LinBusListener::process_lin_msg_queue(TickType_t wait) {
  QUEUE_LIN_MSG msg;
  while (xQueueReceive(this->lin_msg_queue_, &msg, wait) == pdPASS) {
    this->lin_message_recieved_(msg.current_PID, msg.data, msg.len);
  }
}

void LinBusListener::process_log_queue(TickType_t wait) {
#if ESPHOME_LOG_LEVEL > ESPHOME_LOG_LEVEL_NONE
  QUEUE_LOG_MSG log_msg;
  while (xQueueReceive(this->log_queue_, &log_msg, wait) == pdPASS) {
    ESP_LOGV(TAG, "LIN log message type %d", (int) log_msg.type);
  }
#endif
}

#undef LIN_BREAK
#undef LIN_SYNC
#undef DIAGNOSTIC_FRAME_MASTER
#undef DIAGNOSTIC_FRAME_SLAVE
#undef QUEUE_WAIT_DONT_BLOCK

}  // namespace truma_inetbox
}  // namespace esphome

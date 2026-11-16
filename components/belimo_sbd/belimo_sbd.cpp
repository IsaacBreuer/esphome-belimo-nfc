#include "belimo_sbd.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#include <cinttypes>
#include <cmath>
#include <span>
#include <string>
#include <type_traits>

namespace esphome::belimo_sbd {

static const char *const TAG = "belimo_sbd";

// PN532 commands
static const uint8_t PN_IN_DATA_EXCHANGE = 0x40;
static const uint8_t PN_IN_LIST_PASSIVE_TARGET = 0x4A;

// NTAG I2C plus commands
static const uint8_t NTAG_READ = 0x30;
static const uint8_t NTAG_FAST_READ = 0x3A;
static const uint8_t NTAG_WRITE = 0xA2;

// NTAG I2C plus memory (sector 0)
static const uint8_t PAGE_SESSION_REGS = 0xEC;  // 0xEC-0xED, 8 bytes
static const uint8_t PAGE_SRAM_START = 0xF0;    // 0xF0-0xFF, 64 bytes
static const uint8_t DUMP_LAST_START = 0xDE;    // last 4-page read covers 0xDE-0xE1 (end of user memory)

// Session register bytes / bits
static const uint8_t REG_NC = 0;             // NC_REG
static const uint8_t REG_MIRROR_BLOCK = 2;   // SRAM_MIRROR_BLOCK
static const uint8_t REG_NS = 6;             // NS_REG
static const uint8_t NC_PTHRU_ON = 0x40;
static const uint8_t NC_PTHRU_NFC_TO_I2C = 0x01;
static const uint8_t NS_RF_LOCKED = 0x20;
static const uint8_t NS_SRAM_RF_READY = 0x08;

// Belimo NFC protocol v2/v3 frame types
static const uint8_t CMD_MP_COMMAND = 0x01;
static const uint8_t CMD_GET_LAST_MP_ANSWER = 0x02;
static const uint8_t ANS_MP_ANSWER = 0x03;
static const uint8_t ANS_MP_ANSWER_DELAYED = 0x04;
static const uint8_t ANS_MP_ANSWER_ERROR = 0x05;
static const uint8_t ANS_PROTOCOL_ERROR = 0x06;

// MP-Bus
static const uint8_t MP_ADDRESS_MODE_PP = 0x01;
static const uint8_t MP_GET_DATA = 0x6F;
static const uint8_t MP_SET_DATA = 0x6E;
static const uint16_t SPSELECT_POOL = 8;  // the ONLY pool this component will ever write
static const uint8_t SPSELECT_MAX = 11;   // enum range 0..11

static const uint32_t SRAM_TIMEOUT_MS = 5000;     // same as Belimo Assistant
static const uint32_t DELAYED_TIMEOUT_MS = 7000;  // same as Belimo Assistant
static const uint32_t REGISTER_POLL_MS = 20;
static const uint8_t TAG_RETRIES = 5;

// Extracts the type of the 2nd parameter of a member function (the PN532 response buffer type).
template<typename C, typename R, typename A1, typename A2> A2 second_arg_of(R (C::*)(A1, A2));

// Gives this component access to the PN532 driver's protected low-level frame functions.
// Only member functions are added, so the object layout is identical to PN532.
// Works with both driver APIs: std::vector (<= 2026.8) and span/PN532Frame (>= 2026.10).
class PN532Access : public pn532::PN532 {
 public:
  bool cmd(const std::vector<uint8_t> &data) { return cmd_impl_(this, data); }
  bool resp(uint8_t command, std::vector<uint8_t> &data) {
    using Frame = std::remove_reference_t<decltype(second_arg_of(&PN532Access::read_response))>;
    Frame buf;
    if (!this->read_response(command, buf))
      return false;
    data.assign(buf.begin(), buf.end());
    return true;
  }
  void abort() { this->send_ack_(); }
  void rf_off() { this->turn_off_rf_(); }

 private:
  template<typename Self> static bool cmd_impl_(Self *self, const std::vector<uint8_t> &data) {
    if constexpr (requires { self->write_command_(std::span<const uint8_t>(data.data(), data.size())); }) {
      return self->write_command_(std::span<const uint8_t>(data.data(), data.size()));
    } else {
      return self->write_command_(data);
    }
  }
};

static PN532Access *pn_access(pn532::PN532 *p) { return static_cast<PN532Access *>(p); }

static std::string hex(const uint8_t *data, size_t len) {
  static const char *const digits = "0123456789ABCDEF";
  std::string s;
  s.reserve(len * 3);
  for (size_t i = 0; i < len; i++) {
    if (i)
      s += ' ';
    s += digits[data[i] >> 4];
    s += digits[data[i] & 0x0F];
  }
  return s;
}
static std::string hex(const std::vector<uint8_t> &v) { return hex(v.data(), v.size()); }

// ---------------------------------------------------------------------------------------------
// Low level
// ---------------------------------------------------------------------------------------------

bool BelimoSbd::pn_cmd_(const std::vector<uint8_t> &cmd, uint8_t pn_cmd_code, std::vector<uint8_t> &resp) {
  auto *pn = pn_access(this->reader_);
  if (!pn->cmd(cmd)) {
    this->handle_reader_failure_();
    return false;
  }
  this->consecutive_reader_failures_ = 0;
  if (!pn->resp(pn_cmd_code, resp)) {
    pn->abort();
    return false;
  }
  return true;
}

bool BelimoSbd::activate_(std::vector<uint8_t> &uid) {
  std::vector<uint8_t> resp;
  if (!this->pn_cmd_({PN_IN_LIST_PASSIVE_TARGET, 0x01, 0x00}, PN_IN_LIST_PASSIVE_TARGET, resp))
    return false;
  // [NbTg][Tg][SENS_RES x2][SEL_RES][UID len][UID...]
  if (resp.size() < 6 || resp[0] != 1)
    return false;
  uint8_t uid_len = resp[5];
  if (resp.size() < 6u + uid_len)
    return false;
  uid.assign(resp.begin() + 6, resp.begin() + 6 + uid_len);
  return true;
}

bool BelimoSbd::tag_xfer_(const std::vector<uint8_t> &tag_cmd, std::vector<uint8_t> &data) {
  std::vector<uint8_t> cmd{PN_IN_DATA_EXCHANGE, 0x01};
  cmd.insert(cmd.end(), tag_cmd.begin(), tag_cmd.end());
  uint8_t last_status = 0xFF;
  // Transient CRC/collision errors (status 0x01/0x02/0x06) are common when the actuator is
  // powered and sharing the chip; retry a few times before giving up on this transaction.
  for (uint8_t attempt = 0; attempt < TAG_RETRIES; attempt++) {
    std::vector<uint8_t> resp;
    if (!this->pn_cmd_(cmd, PN_IN_DATA_EXCHANGE, resp))
      return false;  // reader-level failure (not a tag error) -> already counted
    if (!resp.empty() && (resp[0] & 0x3F) == 0) {
      data.assign(resp.begin() + 1, resp.end());
      return true;
    }
    last_status = resp.empty() ? 0xFF : (resp[0] & 0x3F);
    delay(3);
  }
  ESP_LOGD(TAG, "Tag command %s failed after %u tries, last PN532 status 0x%02X", hex(tag_cmd).c_str(),
           (unsigned) TAG_RETRIES, last_status);
  return false;
}

bool BelimoSbd::read_page4_(uint8_t page, std::vector<uint8_t> &data) {
  if (!this->tag_xfer_({NTAG_READ, page}, data) || data.size() < 16)
    return false;
  data.resize(16);
  return true;
}

bool BelimoSbd::write_page_(uint8_t page, const uint8_t *d) {
  // Safety: this component only ever writes the volatile pass-through SRAM window.
  if (page < PAGE_SRAM_START) {
    ESP_LOGE(TAG, "Refusing to write page 0x%02X (outside SRAM)", page);
    return false;
  }
  std::vector<uint8_t> data;
  return this->tag_xfer_({NTAG_WRITE, page, d[0], d[1], d[2], d[3]}, data);
}

bool BelimoSbd::read_session_regs_(std::array<uint8_t, 8> &regs) {
  std::vector<uint8_t> data;
  // Plain READ (0x30) of 0xEC returns pages 0xEC-0xEF (16 bytes); the session registers are the
  // first 8. This is more robust through the PN532 than FAST_READ, which some stacks mishandle.
  if (this->tag_xfer_({NTAG_READ, PAGE_SESSION_REGS}, data) && data.size() >= 8) {
    std::copy(data.begin(), data.begin() + 8, regs.begin());
    return true;
  }
  // Fallback: FAST_READ (0x3A) of 0xEC-0xED (8 bytes).
  data.clear();
  if (this->tag_xfer_({NTAG_FAST_READ, PAGE_SESSION_REGS, PAGE_SESSION_REGS + 1}, data) && data.size() >= 8) {
    std::copy(data.begin(), data.begin() + 8, regs.begin());
    return true;
  }
  return false;
}

uint16_t BelimoSbd::crc16_arc_(const uint8_t *data, size_t len) {
  uint16_t crc = 0x0000;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

void BelimoSbd::build_frame_(const std::vector<uint8_t> &frame) {
  this->out_.fill(0);
  for (size_t i = 0; i < frame.size() && this->sram_offset_ + i < this->out_.size(); i++)
    this->out_[this->sram_offset_ + i] = frame[i];
}

// ---------------------------------------------------------------------------------------------
// Cycle control
// ---------------------------------------------------------------------------------------------

void BelimoSbd::dump_config() {
  ESP_LOGCONFIG(TAG, "Belimo SBD (NTAG I2C plus MP tunnel):");
  ESP_LOGCONFIG(TAG, "  Protocol version: %s", this->forced_version_ ? "forced" : "auto");
  ESP_LOGCONFIG(TAG, "  Dump memory: %s", YESNO(this->dump_memory_));
  ESP_LOGCONFIG(TAG, "  Stale timeout: %us", this->stale_blank_ms_ / 1000);
  for (auto &p : this->pool_sensors_)
    ESP_LOGCONFIG(TAG, "  Pool %u (scale %g): %s", p.pool_id, p.scale, p.sensor->get_name().c_str());
  LOG_UPDATE_INTERVAL(this);
}

void BelimoSbd::handle_reader_failure_() {
  if (this->consecutive_reader_failures_ < 255)
    this->consecutive_reader_failures_++;
  uint32_t now = millis();
  if (this->consecutive_reader_failures_ >= 3 && now - this->last_recovery_ > 5000) {
    this->last_recovery_ = now;
    ESP_LOGW(TAG, "PN532 not responding, re-initializing reader");
    this->reader_->setup();  // marks the PN532 failed if it is unplugged -> handled in update()
  }
}

void BelimoSbd::update() {
  if (this->reader_ == nullptr)
    return;

  // Reader was absent at boot or after a failed re-init: retry without rebooting.
  if (this->reader_->is_failed()) {
    uint32_t now = millis();
    if (now - this->last_recovery_ < 5000)
      return;
    this->last_recovery_ = now;
    this->reader_->reset_to_construction_state();
    this->reader_->call();  // runs setup()
    if (this->reader_->is_failed())
      return;
    this->reader_->call();  // SETUP -> LOOP
    this->reader_->disable_loop();
    this->reader_->enable_loop();
    this->consecutive_reader_failures_ = 0;
    ESP_LOGW(TAG, "PN532 detected, started without reboot");
    return;
  }

  if (this->state_ != State::IDLE) {
    ESP_LOGD(TAG, "Previous cycle still running");
    return;
  }
  this->state_ = State::ACTIVATE;
}

void BelimoSbd::finish_cycle_(bool ok) {
  if (ok)
    this->last_good_ms_ = millis();
  this->state_ = State::IDLE;
  pn_access(this->reader_)->rf_off();
  if (!ok)
    ESP_LOGD(TAG, "Cycle ended without complete data");
}

// A cycle failed (no tag, tunnel busy, lost mid-handshake, ...). These are routinely transient,
// especially right after a setpoint write while the actuator persists it to EEPROM. Keep the last
// published values and only blank the sensors to "unknown" once there has been no successful read
// for STALE_BLANK_MS, which marks a genuine disconnect rather than a momentary hiccup.
void BelimoSbd::fail_poll_(const char *reason) {
  if (millis() - this->last_good_ms_ > this->stale_blank_ms_) {
    ESP_LOGW(TAG, "%s; no good read for >%us - marking values unknown", reason, this->stale_blank_ms_ / 1000);
    for (auto &p : this->pool_sensors_)
      p.sensor->publish_state(NAN);
  } else {
    ESP_LOGD(TAG, "%s; keeping last values", reason);
  }
  this->finish_cycle_(false);
}

void BelimoSbd::start_job_(Job job) {
  this->job_ = job;
  switch (job) {
    case Job::VERSION_QUERY:
      // Get-BNPV: two zero bytes at SRAM offset 0, rest zero padded
      this->out_.fill(0);
      break;
    case Job::POOL_READ: {
      uint16_t pool = this->pool_sensors_[this->pool_index_].pool_id;
      std::vector<uint8_t> f{0x08, CMD_MP_COMMAND, (2 + 1) << 4 | MP_ADDRESS_MODE_PP, MP_GET_DATA,
                             uint8_t(pool >> 8), uint8_t(pool & 0xFF), 0, 0};
      uint16_t crc = crc16_arc_(f.data(), 6);
      f[6] = crc >> 8;
      f[7] = crc & 0xFF;
      this->build_frame_(f);
      this->delayed_deadline_ = millis() + DELAYED_TIMEOUT_MS;
      ESP_LOGD(TAG, "Pool %u request: %s", pool, hex(f).c_str());
      break;
    }
    case Job::LAST_ANSWER: {
      std::vector<uint8_t> f{0x04, CMD_GET_LAST_MP_ANSWER, 0, 0};
      uint16_t crc = crc16_arc_(f.data(), 2);
      f[2] = crc >> 8;
      f[3] = crc & 0xFF;
      this->build_frame_(f);
      break;
    }
    case Job::POOL_WRITE: {
      // MP_Set_Data to SpSelect: MP header nibble = 1 (opcode) + 3 params (pool hi, lo, value).
      std::vector<uint8_t> f{0x09,
                             CMD_MP_COMMAND,
                             uint8_t((3 + 1) << 4 | MP_ADDRESS_MODE_PP),
                             MP_SET_DATA,
                             uint8_t(SPSELECT_POOL >> 8),
                             uint8_t(SPSELECT_POOL & 0xFF),
                             this->write_value_,
                             0,
                             0};
      uint16_t crc = crc16_arc_(f.data(), 7);
      f[7] = crc >> 8;
      f[8] = crc & 0xFF;
      this->build_frame_(f);
      this->delayed_deadline_ = millis() + DELAYED_TIMEOUT_MS;
      ESP_LOGW(TAG, "WRITE SpSelect = %u request: %s", this->write_value_, hex(f).c_str());
      break;
    }
    case Job::FLUSH:
      break;
  }
  this->state_ = State::WAIT_WRITE_READY;
  this->deadline_ = millis() + SRAM_TIMEOUT_MS;
}

void BelimoSbd::loop() {
  if (this->state_ == State::IDLE || this->reader_ == nullptr || this->reader_->is_failed())
    return;

  uint32_t now = millis();

  switch (this->state_) {
    case State::ACTIVATE: {
      std::vector<uint8_t> uid;
      if (!this->activate_(uid)) {
        this->fail_poll_("No tag in field");
        return;
      }
      if (uid != this->uid_)
        ESP_LOGI(TAG, "Tag found, UID %s (%u bytes)", hex(uid).c_str(), (unsigned) uid.size());
      this->uid_ = uid;
      this->step_ = 0;
      this->state_ = (this->dump_memory_ && this->dumped_uid_ != uid) ? State::DUMP : State::CHECK_TUNNEL;
      return;
    }

    case State::DUMP: {
      uint8_t page = this->step_ * 4 > DUMP_LAST_START - 2 ? DUMP_LAST_START : this->step_ * 4;
      if (page == 0)
        ESP_LOGI(TAG, "---- NTAG memory dump (sector 0, read-only) ----");
      std::vector<uint8_t> data;
      if (!this->read_page4_(page, data)) {
        ESP_LOGW(TAG, "Dump stopped at page 0x%02X", page);
        this->dumped_uid_ = this->uid_;
        this->finish_cycle_(false);
        return;
      }
      ESP_LOGI(TAG, "page %02X: %s", page, hex(data).c_str());
      if (page == DUMP_LAST_START) {
        ESP_LOGI(TAG, "---- end of dump ----");
        this->dumped_uid_ = this->uid_;
        this->state_ = State::CHECK_TUNNEL;
      } else {
        this->step_++;
      }
      return;
    }

    case State::CHECK_TUNNEL: {
      std::array<uint8_t, 8> r{};
      if (!this->read_session_regs_(r)) {
        this->fail_poll_("Could not read session registers");
        return;
      }
      ESP_LOGD(TAG, "Session regs: %s (NC=%02X MIRROR_BLOCK=%02X NS=%02X)", hex(r.data(), 8).c_str(), r[REG_NC],
               r[REG_MIRROR_BLOCK], r[REG_NS]);
      if (!(r[REG_NC] & NC_PTHRU_ON)) {
        this->fail_poll_("Pass-through (MP tunnel) off - actuator unpowered or writing EEPROM");
        return;
      }
      if (this->protocol_version_ == 0 && this->forced_version_ != 0) {
        this->protocol_version_ = this->forced_version_;
        this->sram_offset_ = this->protocol_version_ == 3 ? 48 : 0;
      }
      if (this->protocol_version_ == 0) {
        this->sram_offset_ = 0;
        this->start_job_(Job::VERSION_QUERY);
      } else if (this->write_pending_) {
        this->start_job_(Job::POOL_WRITE);
      } else if (!this->pool_sensors_.empty()) {
        this->pool_index_ = 0;
        this->start_job_(Job::POOL_READ);
      } else {
        this->finish_cycle_(true);
      }
      return;
    }

    case State::WAIT_WRITE_READY:
    case State::WAIT_READ_READY: {
      if (now - this->last_poll_ < REGISTER_POLL_MS)
        return;
      this->last_poll_ = now;
      std::array<uint8_t, 8> r{};
      if (!this->read_session_regs_(r)) {
        ESP_LOGW(TAG, "Lost tag while waiting for SRAM");
        this->finish_cycle_(false);
        return;
      }
      uint8_t nc = r[REG_NC], ns = r[REG_NS];
      bool base = (ns & NS_RF_LOCKED) && (nc & NC_PTHRU_ON);
      bool write_ready = base && (nc & NC_PTHRU_NFC_TO_I2C) && !(ns & NS_SRAM_RF_READY);
      bool read_ready = base && !(nc & NC_PTHRU_NFC_TO_I2C) && (ns & NS_SRAM_RF_READY);

      if (this->state_ == State::WAIT_WRITE_READY) {
        if (read_ready && this->job_ != Job::FLUSH) {
          // A stale answer is waiting in SRAM: read it out first (Belimo does an extra TUNNEL_READ)
          ESP_LOGD(TAG, "Stale answer pending, flushing SRAM");
          this->job_after_flush_ = this->job_;
          this->job_ = Job::FLUSH;
          this->state_ = State::READ_SRAM;
          this->step_ = 0;
          this->in_.clear();
          return;
        }
        if (write_ready) {
          this->state_ = State::WRITE_SRAM;
          this->step_ = 0;
          return;
        }
      } else if (read_ready) {
        this->state_ = State::READ_SRAM;
        this->step_ = 0;
        this->in_.clear();
        return;
      }
      if ((int32_t) (now - this->deadline_) > 0) {
        ESP_LOGW(TAG, "Timeout waiting for %s (regs %s)",
                 this->state_ == State::WAIT_WRITE_READY ? "SRAM write-ready" : "SRAM answer", hex(r.data(), 8).c_str());
        this->finish_cycle_(false);
      }
      return;
    }

    case State::WRITE_SRAM: {
      uint8_t page = PAGE_SRAM_START + this->step_;
      if (!this->write_page_(page, &this->out_[this->step_ * 4])) {
        ESP_LOGW(TAG, "SRAM write failed at page 0x%02X", page);
        this->finish_cycle_(false);
        return;
      }
      if (++this->step_ >= 16) {  // writing page 0xFF hands the SRAM to the actuator
        this->state_ = State::WAIT_READ_READY;
        this->deadline_ = now + SRAM_TIMEOUT_MS;
      }
      return;
    }

    case State::READ_SRAM: {
      std::vector<uint8_t> data;
      uint8_t page = PAGE_SRAM_START + this->step_ * 4;
      if (!this->read_page4_(page, data)) {
        ESP_LOGW(TAG, "SRAM read failed at page 0x%02X", page);
        this->finish_cycle_(false);
        return;
      }
      this->in_.insert(this->in_.end(), data.begin(), data.end());
      if (++this->step_ >= 4)  // reading page 0xFF hands the SRAM back
        this->handle_sram_(this->in_);
      return;
    }

    default:
      return;
  }
}

// ---------------------------------------------------------------------------------------------
// Answer handling
// ---------------------------------------------------------------------------------------------

void BelimoSbd::handle_sram_(const std::vector<uint8_t> &s) {
  if (this->job_ == Job::FLUSH) {
    ESP_LOGD(TAG, "Flushed: %s", hex(s).c_str());
    this->job_ = this->job_after_flush_;
    this->state_ = State::WAIT_WRITE_READY;
    this->deadline_ = millis() + SRAM_TIMEOUT_MS;
    return;
  }

  if (this->job_ == Job::VERSION_QUERY) {
    ESP_LOGI(TAG, "Get-BNPV answer: %s", hex(s).c_str());
    uint8_t v = 2;
    if (s[0] == 3) {
      v = 3;
    } else if (s[0] == 2) {
      v = 2;
    } else if (s[0] >= 4 && s[1] == ANS_PROTOCOL_ERROR && ((s[2] << 8) | s[3]) == 3) {
      ESP_LOGI(TAG, "Length-mismatch protocol error -> device speaks BNPV 2 without Get-BNPV");
      v = 2;
    } else if (s[0] == 1) {
      ESP_LOGW(TAG, "Device reports BNPV 1, which is not supported by this component");
    } else {
      ESP_LOGW(TAG, "Unexpected Get-BNPV answer, assuming version 2");
    }
    this->protocol_version_ = v;
    this->sram_offset_ = v == 3 ? 48 : 0;
    ESP_LOGI(TAG, "Using Belimo NFC protocol version %u (SRAM frame offset %u)", v, this->sram_offset_);
    if (this->pool_sensors_.empty()) {
      this->finish_cycle_(true);
    } else {
      this->pool_index_ = 0;
      this->start_job_(Job::POOL_READ);
    }
    return;
  }

  if (this->job_ == Job::POOL_WRITE) {
    const uint8_t off = this->sram_offset_;
    const uint8_t len = s[off];
    bool ok = false;
    if (len >= 4 && off + len <= 64) {
      uint16_t crc = crc16_arc_(&s[off], len - 2);
      uint16_t got = (s[off + len - 2] << 8) | s[off + len - 1];
      uint8_t type = s[off + 1];
      ok = (crc == got && type == ANS_MP_ANSWER);
      ESP_LOGW(TAG, "WRITE SpSelect=%u ack: type 0x%02X, crc %s, frame %s", this->write_value_, type,
               crc == got ? "ok" : "BAD", hex(&s[off], std::min<size_t>(len ? len : 4, 64 - off)).c_str());
      if (type == ANS_MP_ANSWER_ERROR)
        ESP_LOGW(TAG, "WRITE rejected by actuator (MP answer error)");
      else if (type == ANS_PROTOCOL_ERROR)
        ESP_LOGW(TAG, "WRITE rejected (NFC protocol error)");
    } else {
      ESP_LOGW(TAG, "WRITE SpSelect ack invalid: %s", hex(s).c_str());
    }
    ESP_LOGW(TAG, "WRITE %s - reading values back", ok ? "acknowledged" : "NOT confirmed");
    this->write_pending_ = false;
    if (!this->pool_sensors_.empty()) {
      this->pool_index_ = 0;
      this->start_job_(Job::POOL_READ);
    } else {
      this->finish_cycle_(ok);
    }
    return;
  }

  // POOL_READ or LAST_ANSWER
  auto &ps = this->pool_sensors_[this->pool_index_];
  const uint8_t off = this->sram_offset_;
  const uint8_t len = s[off];
  ESP_LOGD(TAG, "Pool %u answer: %s", ps.pool_id, hex(&s[off], std::min<size_t>(len ? len : 8, 64 - off)).c_str());

  bool next = true;
  if (len < 4 || off + len > 64) {
    ESP_LOGW(TAG, "Pool %u: invalid answer length %u, SRAM: %s", ps.pool_id, len, hex(s).c_str());
  } else {
    uint16_t crc = crc16_arc_(&s[off], len - 2);
    uint16_t got = (s[off + len - 2] << 8) | s[off + len - 1];
    uint8_t type = s[off + 1];
    const uint8_t *payload = &s[off + 2];
    size_t plen = len - 4;
    if (crc != got) {
      ESP_LOGW(TAG, "Pool %u: CRC mismatch (calc %04X, got %04X)", ps.pool_id, crc, got);
    } else if (type == ANS_MP_ANSWER) {
      // Payload is [MP header byte][data...]. The header's high nibble is the data-byte count
      // (0x4D = 4 bytes, 0x1D = 1 byte). The value is those data bytes, big-endian.
      if (plen >= 2) {
        uint8_t mp_hdr = payload[0];
        size_t ndata = plen - 1;  // everything after the MP header byte
        int32_t raw = 0;
        for (size_t i = 0; i < ndata; i++)
          raw = (raw << 8) | payload[1 + i];
        float value = raw * ps.scale;
        ESP_LOGI(TAG, "Pool %u = %" PRId32 " raw -> %.4f  (mp-hdr 0x%02X, payload %s)", ps.pool_id, raw, value,
                 mp_hdr, hex(payload, plen).c_str());
        ps.sensor->publish_state(value);
      } else {
        ESP_LOGW(TAG, "Pool %u: short payload %s", ps.pool_id, hex(payload, plen).c_str());
      }
    } else if (type == ANS_MP_ANSWER_DELAYED) {
      if ((int32_t) (millis() - this->delayed_deadline_) < 0) {
        ESP_LOGD(TAG, "Pool %u: answer delayed, asking for last answer", ps.pool_id);
        this->start_job_(Job::LAST_ANSWER);
        next = false;
      } else {
        ESP_LOGW(TAG, "Pool %u: delayed answer timed out", ps.pool_id);
      }
    } else if (type == ANS_MP_ANSWER_ERROR) {
      ESP_LOGW(TAG, "Pool %u: MP answer error, code %u", ps.pool_id, plen ? payload[0] : 0);
    } else if (type == ANS_PROTOCOL_ERROR) {
      ESP_LOGW(TAG, "Pool %u: NFC protocol error, code %u", ps.pool_id, plen >= 2 ? (payload[0] << 8) | payload[1] : 0);
    } else {
      ESP_LOGW(TAG, "Pool %u: unknown answer type 0x%02X", ps.pool_id, type);
    }
  }

  if (!next)
    return;
  if (++this->pool_index_ < this->pool_sensors_.size()) {
    this->start_job_(Job::POOL_READ);
  } else {
    this->finish_cycle_(true);
  }
}

void BelimoSbd::request_setpoint_select(uint8_t preset) {
  if (preset > SPSELECT_MAX) {
    ESP_LOGW(TAG, "SpSelect %u out of range 0..%u, ignored", preset, SPSELECT_MAX);
    return;
  }
  this->write_value_ = preset;
  this->write_pending_ = true;
  ESP_LOGW(TAG, "Queued SpSelect write = %u", preset);
  // Kick a cycle right now if the reader is idle, so the write applies within ~1-2s instead of
  // waiting for the next scheduled poll. loop() picks up the ACTIVATE state on its next pass.
  if (this->state_ == State::IDLE && this->reader_ != nullptr && !this->reader_->is_failed())
    this->state_ = State::ACTIVATE;
}

}  // namespace esphome::belimo_sbd

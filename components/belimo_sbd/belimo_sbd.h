#pragma once

// Read-only reader for Belimo actuators with an NXP NTAG I2C plus NFC chip (e.g. EWC SBD2).
//
// Protocol (reverse engineered from Belimo Assistant 2, v6.15.1):
//  - Live values travel through the NTAG I2C plus pass-through SRAM (pages 0xF0-0xFF, 64 bytes).
//  - Session registers (pages 0xEC-0xED) tell when the SRAM may be written / is ready to read.
//  - Request frame  : [LEN][CMD][MP-Bus bytes...][CRC16 hi][CRC16 lo]   CMD 0x01 = MP command
//  - Answer frame   : [LEN][TYPE][payload...][CRC16 hi][CRC16 lo]       TYPE 0x03 = MP answer
//  - CRC16          : CRC-16/ARC (poly 0xA001 reflected, init 0), big-endian
//  - MP_Get_Data    : [0x31][0x6F][pool hi][pool lo] -> 4 data bytes, big-endian int32
//  - Frame offset in SRAM: 0 for protocol v2, 48 for protocol v3
//
// This component never writes outside the volatile SRAM window and only sends MP_Get_Data
// (read) commands. It does not touch EEPROM, configuration or lock pages.

#include "esphome/core/component.h"
#include "esphome/components/pn532/pn532.h"
#include "esphome/components/sensor/sensor.h"

#include <array>
#include <vector>

namespace esphome::belimo_sbd {

struct PoolSensor {
  uint16_t pool_id;
  float scale;
  sensor::Sensor *sensor;
};

class BelimoSbd : public PollingComponent {
 public:
  void set_pn532(pn532::PN532 *reader) { this->reader_ = reader; }
  void set_protocol_version(uint8_t v) { this->forced_version_ = v; }
  void set_dump_memory(bool v) { this->dump_memory_ = v; }
  void set_stale_timeout(uint32_t ms) { this->stale_blank_ms_ = ms; }
  void add_pool_sensor(uint16_t pool_id, float scale, sensor::Sensor *s) {
    this->pool_sensors_.push_back({pool_id, scale, s});
  }
  // Queue a write of the setpoint preset selector (SpSelect, pool 8). preset 0..11; see the
  // enum in the profile (6 = 0.50"wc, 8 = 0.60"wc, etc). This is the ONLY writable datapoint,
  // applied on the next poll cycle, after which all pools are read back. Values outside 0..11
  // are ignored. Call from a template button/select in YAML.
  void request_setpoint_select(uint8_t preset);

  void setup() override {}
  void update() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  enum class State : uint8_t {
    IDLE,
    ACTIVATE,
    DUMP,
    CHECK_TUNNEL,
    WAIT_WRITE_READY,
    WRITE_SRAM,
    WAIT_READ_READY,
    READ_SRAM,
  };
  enum class Job : uint8_t { VERSION_QUERY, POOL_READ, LAST_ANSWER, FLUSH, POOL_WRITE };

  // PN532 helpers (one short transaction each)
  bool pn_cmd_(const std::vector<uint8_t> &cmd, uint8_t pn_cmd_code, std::vector<uint8_t> &resp);
  bool activate_(std::vector<uint8_t> &uid);
  bool tag_xfer_(const std::vector<uint8_t> &tag_cmd, std::vector<uint8_t> &data);
  bool read_page4_(uint8_t page, std::vector<uint8_t> &data);
  bool write_page_(uint8_t page, const uint8_t *data);
  bool read_session_regs_(std::array<uint8_t, 8> &regs);

  void start_job_(Job job);
  void finish_cycle_(bool ok);
  void fail_poll_(const char *reason);  // end a failed cycle; blank sensors only once stale
  void handle_sram_(const std::vector<uint8_t> &sram);
  void handle_reader_failure_();

  static uint16_t crc16_arc_(const uint8_t *data, size_t len);
  void build_frame_(const std::vector<uint8_t> &frame);

  pn532::PN532 *reader_{nullptr};
  std::vector<PoolSensor> pool_sensors_;
  uint8_t forced_version_{0};
  bool dump_memory_{true};

  State state_{State::IDLE};
  Job job_{Job::POOL_READ};
  Job job_after_flush_{Job::POOL_READ};
  uint8_t protocol_version_{0};  // 0 = unknown
  uint8_t sram_offset_{0};
  size_t pool_index_{0};
  uint8_t step_{0};
  uint32_t deadline_{0};
  uint32_t delayed_deadline_{0};
  uint32_t last_poll_{0};
  std::array<uint8_t, 64> out_{};
  std::vector<uint8_t> in_;
  std::vector<uint8_t> uid_;
  std::vector<uint8_t> dumped_uid_;
  uint8_t consecutive_reader_failures_{0};
  uint32_t last_recovery_{0};
  bool write_pending_{false};
  uint8_t write_value_{0};
  uint32_t last_good_ms_{0};      // millis() of the last fully successful read; drives staleness
  uint32_t stale_blank_ms_{20000};  // hold last values this long before "unknown" (YAML override)
};

}  // namespace esphome::belimo_sbd

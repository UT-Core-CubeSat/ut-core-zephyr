// ==============================================
// CubeSat CDH - C++-style pseudocode (bare-metal)
// ==============================================
// Modules:
//  - Util (CRC32, majority vote, TLV, timers)
//  - HAL shims (Flash/ExtMem, CAN, Radio/Uplink, Storage, GPIO, WDG)
//  - Data models (Modes, Cmd, Tlm, Health)
//  - Managers: Mode, CommandRouter, TelemetryStore, FirmwareUpdate, Health, Scheduler
//  - Main loop

#include <cstdint>
#include <cstddef>

// ---------- Compile-time config ----------
namespace CFG {
  constexpr uint32_t CDH_NODE_ID          = 0x11;
  constexpr uint32_t CAN_BAUD             = 1000000;
  constexpr uint32_t RADIO_MAX_FRAME      = 220;     // bytes
  constexpr uint32_t TLM_RING_BYTES       = 64 * 1024;
  constexpr uint32_t FW_SLOT_BYTES        = 512 * 1024; // flash per bank
  constexpr uint32_t FW_CHUNK_BYTES       = 512;       // transfer chunk
  constexpr uint32_t FW_TMR_COPIES        = 3;         // triple redundancy copies
  constexpr uint32_t FW_IMAGE_MAX         = FW_SLOT_BYTES;
  constexpr uint32_t SCRUB_PERIOD_MS      = 30'000;    // memory scrub
  constexpr uint32_t HEALTH_PERIOD_MS     = 1'000;
  constexpr uint32_t BEACON_PERIOD_MS     = 10'000;
  constexpr uint32_t WATCHDOG_KICK_MS     = 200;
  constexpr uint32_t COMMAND_TIMEOUT_MS   = 2'000;
  constexpr uint32_t SAFE_RETRY_LIMIT     = 3;
}

// ---------- Utilities ----------
namespace Util {
  uint32_t crc32(const uint8_t* data, size_t len);
  uint16_t crc16(const uint8_t* data, size_t len);

  template<typename T>
  T clamp(T v, T lo, T hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

  // Majority vote across N copies (TMR). Returns {value, majority_ok}
  template<typename T, size_t N>
  struct VoteResult { T value; bool ok; };
  template<typename T, size_t N>
  VoteResult<T, N> majority_vote(const T (&vals)[N]) {
    // Simple pairwise compare. For N=3.
    if (vals[0] == vals[1]) return {vals[0], true};
    if (vals[0] == vals[2]) return {vals[0], true};
    if (vals[1] == vals[2]) return {vals[1], true};
    return {vals[0], false}; // no majority
  }

  // Monotonic ms from a hardware timer
  uint64_t now_ms();

  // TLV helpers (Type-Length-Value for cmds/tlm)
  struct TLV {
    uint16_t type; uint16_t len; const uint8_t* val;
  };
  bool tlv_next(const uint8_t* buf, size_t buflen, size_t& off, TLV& out);

  // Simple backoff helper
  struct Backoff {
    uint32_t base_ms; uint32_t max_ms; uint32_t current_ms;
    void reset() { current_ms = base_ms; }
    uint32_t next() { current_ms = clamp(current_ms * 2, base_ms, max_ms); return current_ms; }
  };
}

// ---------- Hardware Abstraction Shims ----------
namespace HAL {
  // Watchdog
  void wdg_init(); void wdg_kick();

  // LEDs/GPIO (for modes/errors)
  enum class Led { STATUS, ERROR, COMMS };
  void led_set(Led l, bool on);
  void led_pulse(Led l, uint32_t ms);

  // CAN
  struct CanMsg { uint32_t id; uint8_t dlc; uint8_t data[8]; };
  bool can_init(uint32_t baud, uint32_t node_id);
  bool can_send(const CanMsg& m);
  bool can_recv(CanMsg& m, uint32_t timeout_ms);

  // Radio/Uplink/Downlink (half-duplex framing, CRC, ARQ handled inside)
  bool radio_init();
  // Receive ground frame; returns len, fills buf with payload (already CRC-checked)
  int  radio_rx(uint8_t* buf, size_t maxlen, uint32_t timeout_ms);
  bool radio_tx(const uint8_t* buf, size_t len);

  // Non-volatile storage for telemetry/logs (FRAM, MRAM, or NOR via wear-leveling)
  bool store_init();
  bool store_write(uint32_t off, const uint8_t* data, size_t len);
  bool store_read(uint32_t off, uint8_t* data, size_t len);
  uint32_t store_capacity();

  // Program flash / external memories (triple copies)
  enum class Bank { A, B };     // Dual-bank application slots
  enum class Copy { C0, C1, C2 }; // TMR copies per bank
  bool emem_init();
  bool emem_erase(Bank b, Copy c);
  bool emem_write(Bank b, Copy c, uint32_t off, const uint8_t* data, size_t len);
  bool emem_read (Bank b, Copy c, uint32_t off, uint8_t* data, size_t len);

  // Boot control
  void boot_mark_active(Bank b);    // write bootloader marker
  Bank boot_get_active();
  void system_reset();

  // Time base
  void systick_init_1ms();
}

// ---------- IDs, Modes, Commands ----------
namespace IDs {
  // CAN arbitration IDs (11-bit). Example mapping:
  // [10:8]=bus group, [7:5]=dest, [4:2]=src, [1:0]=type
  constexpr uint16_t GROUP_SYS = 0b000;
  constexpr uint16_t NODE_CDH  = 0b001;
  constexpr uint16_t NODE_EPS  = 0b010;
  constexpr uint16_t NODE_ADCS = 0b011;
  constexpr uint16_t NODE_COMM = 0b100;

  constexpr uint16_t TYPE_TLM  = 0b00;
  constexpr uint16_t TYPE_CMD  = 0b01;
  constexpr uint16_t TYPE_SOH  = 0b10;

  constexpr uint16_t CAN_ID(uint16_t dest, uint16_t src, uint16_t type) {
    return (GROUP_SYS << 8) | (dest << 5) | (src << 2) | (type);
  }
}

enum class Mode : uint8_t {
  BOOT, SAFE, IDLE, NOMINAL, SCIENCE, LOW_POWER
};

enum class CmdType : uint16_t {
  NOOP                = 0x0000,
  PING                = 0x0001,
  SET_MODE            = 0x0002,
  REQ_TLM_DUMP        = 0x0003,
  // Reprogramming
  FW_BEGIN            = 0x1000,
  FW_WRITE_CHUNK      = 0x1001,
  FW_END              = 0x1002,
  FW_COMMIT           = 0x1003,
  // Board commands (forwarded to subsystems)
  EPS_SET_STATE       = 0x2000,
  ADCS_SET_TARGET     = 0x2100,
  COMM_SET_RADIO      = 0x2200,
};

struct CmdHeader {
  uint16_t type;    // CmdType
  uint16_t seq;     // sequencing
  uint32_t ts_ms;   // ground send time
};

struct TlmSOH {
  uint32_t ts_ms;
  Mode     mode;
  uint8_t  boot_count;
  uint8_t  safe_retries;
  uint16_t v_batt_mV;
  int16_t  temp_cdeg;
  uint32_t errors; // bitfield
};

struct HealthCounters {
  uint32_t can_rx; uint32_t can_tx;
  uint32_t radio_rx; uint32_t radio_tx;
  uint32_t cmd_ok; uint32_t cmd_err;
  uint32_t tlm_dropped;
};

// ---------- Telemetry Store (ring buffer over NVM) ----------
class TelemetryStore {
public:
  bool init(uint32_t capacity_bytes) {
    cap_ = capacity_bytes; head_ = 0; tail_ = 0; return true;
  }
  bool push(const uint8_t* rec, uint16_t len) {
    if (len + 4 > cap_) return false; // record too big
    uint8_t hdr[4] = { uint8_t(len & 0xFF), uint8_t(len >> 8), 0, 0 };
    uint16_t crc = Util::crc16(rec, len);
    hdr[2] = uint8_t(crc & 0xFF); hdr[3] = uint8_t(crc >> 8);
    if (!write_(hdr, 4)) return false;
    if (!write_(rec, len)) return false;
    return true;
  }
  // Linear dump for downlink
  bool read_range(uint32_t off, uint8_t* buf, size_t len) { return HAL::store_read(off, buf, len); }
  uint32_t head() const { return head_; } // newest
  uint32_t tail() const { return tail_; } // oldest

private:
  bool write_(const uint8_t* data, size_t len) {
    if (len == 0) return true;
    // wrap-aware write
    if (head_ + len <= cap_) {
      if (!HAL::store_write(head_, data, len)) return false;
      head_ += len;
    } else {
      size_t first = cap_ - head_;
      if (!HAL::store_write(head_, data, first)) return false;
      if (!HAL::store_write(0, data + first, len - first)) return false;
      head_ = (len - first);
      tail_ = head_; // simple overwrite policy
    }
    // If ring overlaps tail, advance tail (overwrite)
    // (For brevity: a production impl would move tail by record granularity)
    return true;
  }
  uint32_t cap_{0}, head_{0}, tail_{0};
};

// ---------- Firmware Update (dual-bank + TMR copies) ----------
class FirmwareUpdate {
public:
  struct Session {
    bool active{false};
    HAL::Bank target_bank{HAL::Bank::B};
    uint32_t expected_len{0};
    uint32_t received{0};
    uint32_t golden_crc{0};
  };

  bool init() { return HAL::emem_init(); }

  bool begin(HAL::Bank target_bank, uint32_t total_len, uint32_t crc32) {
    if (sess_.active) return false;
    if (total_len == 0 || total_len > CFG::FW_IMAGE_MAX) return false;
    // Erase all three copies for target bank
    for (int c=0; c<CFG::FW_TMR_COPIES; ++c) {
      if (!HAL::emem_erase(target_bank, (HAL::Copy)c)) return false;
    }
    sess_ = {true, target_bank, total_len, 0, crc32};
    return true;
  }

  bool write_chunk(uint32_t off, const uint8_t* data, size_t len) {
    if (!sess_.active || (off + len) > sess_.expected_len) return false;
    // Write same chunk to three copies
    for (int c=0; c<CFG::FW_TMR_COPIES; ++c) {
      if (!HAL::emem_write(sess_.target_bank, (HAL::Copy)c, off, data, len)) return false;
    }
    sess_.received = (off + len > sess_.received) ? (off + len) : sess_.received;
    return true;
  }

  bool end() {
    if (!sess_.active || sess_.received != sess_.expected_len) return false;
    // Verify majority CRC across three copies
    uint32_t crc[CFG::FW_TMR_COPIES] {};
    for (int c=0; c<CFG::FW_TMR_COPIES; ++c) {
      crc[c] = crc_of_copy_(sess_.target_bank, (HAL::Copy)c, sess_.expected_len);
    }
    auto vote = Util::majority_vote<uint32_t, CFG::FW_TMR_COPIES>(crc);
    if (!vote.ok || vote.value != sess_.golden_crc) return false;
    return true;
  }

  bool commit_and_reboot() {
    if (!sess_.active) return false;
    // Atomic bank swap via boot marker
    HAL::boot_mark_active(sess_.target_bank);
    sess_ = {}; // clear
    HAL::system_reset();
    return true; // not reached
  }

  // Periodic memory scrub: read majority, repair bad copy if mismatch
  void periodic_scrub() {
    uint8_t buf[CFG::FW_CHUNK_BYTES];
    for (uint32_t off=0; off<CFG::FW_IMAGE_MAX; off += sizeof(buf)) {
      size_t len = (off + sizeof(buf) <= CFG::FW_IMAGE_MAX) ? sizeof(buf) : (CFG::FW_IMAGE_MAX - off);
      int good_idx = -1;
      // Read all copies
      for (int c=0; c<CFG::FW_TMR_COPIES; ++c) {
        HAL::emem_read(active_bank(), (HAL::Copy)c, off, tmp_[c], len);
      }
      // Majority vote per byte (bytewise TMR repair)
      for (size_t i=0; i<len; ++i) {
        uint8_t v[3] = { tmp_[0][i], tmp_[1][i], tmp_[2][i] };
        // byte majority
        uint8_t maj = (v[0]==v[1]) ? v[0] : (v[0]==v[2]) ? v[0] : v[1];
        // Repair any dissenters
        for (int c=0; c<CFG::FW_TMR_COPIES; ++c) {
          if (tmp_[c][i] != maj) tmp_[c][i] = maj, dirty_[c] = true;
        }
      }
      // Write back any repaired copies
      for (int c=0; c<CFG::FW_TMR_COPIES; ++c) {
        if (dirty_[c]) {
          HAL::emem_write(active_bank(), (HAL::Copy)c, off, tmp_[c], len);
          dirty_[c] = false;
        }
      }
    }
  }

  HAL::Bank active_bank() const { return HAL::boot_get_active(); }

private:
  uint32_t crc_of_copy_(HAL::Bank b, HAL::Copy c, uint32_t len) {
    uint8_t chunk[CFG::FW_CHUNK_BYTES];
    uint32_t off = 0; uint32_t crc = 0xFFFF'FFFF;
    while (off < len) {
      size_t this_len = (len - off > sizeof(chunk)) ? sizeof(chunk) : (len - off);
      HAL::emem_read(b, c, off, chunk, this_len);
      crc = Util::crc32(chunk, this_len) ^ (crc >> 1); // cheap rolling combo
      off += this_len;
    }
    return crc;
  }

  Session sess_;
  // scratch for scrub
  uint8_t tmp_[CFG::FW_TMR_COPIES][CFG::FW_CHUNK_BYTES];
  bool    dirty_[CFG::FW_TMR_COPIES]{};
};

// ---------- Command Router ----------
class CommandRouter {
public:
  void init(uint16_t node_id) { node_id_ = node_id; }

  // Handle inbound command from radio (ground) or CAN (internal)
  void handle(const uint8_t* buf, size_t len) {
    if (len < sizeof(CmdHeader)) return;
    const CmdHeader* hdr = reinterpret_cast<const CmdHeader*>(buf);
    const uint8_t*   body = buf + sizeof(CmdHeader);
    size_t           body_len = len - sizeof(CmdHeader);

    switch ((CmdType)hdr->type) {
      case CmdType::PING: respond_ping_(hdr->seq); break;
      case CmdType::SET_MODE: set_mode_(body, body_len); break;
      case CmdType::REQ_TLM_DUMP: request_tlm_dump_(); break;

      case CmdType::FW_BEGIN:  fw_begin_(body, body_len); break;
      case CmdType::FW_WRITE_CHUNK: fw_write_chunk_(body, body_len); break;
      case CmdType::FW_END:    fw_end_(); break;
      case CmdType::FW_COMMIT: fw_commit_(); break;

      // Forwarded board commands over CAN
      case CmdType::EPS_SET_STATE: forward_can_(IDs::NODE_EPS, buf, len); break;
      case CmdType::ADCS_SET_TARGET: forward_can_(IDs::NODE_ADCS, buf, len); break;
      case CmdType::COMM_SET_RADIO:  forward_can_(IDs::NODE_COMM, buf, len); break;

      default: /* ignore/telemetry error */ break;
    }
  }

  void bind_fw(FirmwareUpdate* f) { fw_ = f; }
  void bind_mode(class ModeManager* m) { mm_ = m; }
  void bind_tlm(class TelemetryStore* t) { tlm_ = t; }

private:
  void respond_ping_(uint16_t seq) {
    uint8_t resp[8] = { 'P','O','N','G', uint8_t(seq), uint8_t(seq>>8), 0, 0 };
    HAL::radio_tx(resp, sizeof(resp));
  }

  void set_mode_(const uint8_t* body, size_t n) {
    if (n < 1 || !mm_) return;
    mm_->request_mode((Mode)body[0]);
  }

  void request_tlm_dump_() {
    // Stream the ring buffer out the radio in framed chunks
    // (Implementation detail omitted for brevity)
  }

  void fw_begin_(const uint8_t* body, size_t n) {
    if (!fw_ || n < 9) return;
    HAL::Bank bank = (body[0] == 0) ? HAL::Bank::A : HAL::Bank::B;
    uint32_t total = *(uint32_t*)&body[1];
    uint32_t crc   = *(uint32_t*)&body[5];
    fw_->begin(bank, total, crc);
  }

  void fw_write_chunk_(const uint8_t* body, size_t n) {
    if (!fw_ || n < 4) return;
    uint32_t off = *(uint32_t*)body;
    const uint8_t* data = body + 4;
    size_t len = n - 4;
    fw_->write_chunk(off, data, len);
  }

  void fw_end_() { if (fw_) fw_->end(); }

  void fw_commit_() { if (fw_) fw_->commit_and_reboot(); }

  void forward_can_(uint16_t dest_node, const uint8_t* frame, size_t len) {
    // Split into 8B CAN frames with simple sequence numbers
    uint16_t seq = 0;
    for (size_t off=0; off<len; off+=7) {
      HAL::CanMsg m{};
      m.id  = IDs::CAN_ID(dest_node, IDs::NODE_CDH, IDs::TYPE_CMD);
      m.dlc = 8;
      m.data[0] = uint8_t(seq++);
      size_t n = (len - off > 7) ? 7 : (len - off);
      for (size_t i=0; i<n; ++i) m.data[1+i] = frame[off+i];
      for (size_t i=n; i<7; ++i) m.data[1+i] = 0;
      HAL::can_send(m);
    }
  }

  uint16_t node_id_{};
  FirmwareUpdate* fw_{nullptr};
  class ModeManager* mm_{nullptr};
  class TelemetryStore* tlm_{nullptr};
};

// ---------- Mode Manager ----------
class ModeManager {
public:
  void init() {
    mode_ = Mode::BOOT;
    safe_retries_ = 0;
    last_transition_ms_ = Util::now_ms();
  }

  Mode mode() const { return mode_; }

  void request_mode(Mode m) { requested_ = m; }

  void tick() {
    // Handle pending requests, guard with constraints
    if (requested_.has_value()) {
      if (can_enter_(requested_.value())) {
        enter_(requested_.value());
      }
      requested_.reset();
    }
    // Mode-specific periodic actions can be placed here
  }

private:
  bool can_enter_(Mode m) {
    // Example guards: power level, thermal, comm link
    // (Query EPS/ADCS via cached SOH or direct CAN, omitted)
    return true;
  }

  void enter_(Mode m) {
    // Exit actions for current mode (if any)
    // ...
    mode_ = m;
    last_transition_ms_ = Util::now_ms();
    // Entry actions
    switch (mode_) {
      case Mode::SAFE:
        HAL::led_set(HAL::Led::ERROR, true);
        break;
      case Mode::NOMINAL:
      case Mode::SCIENCE:
        HAL::led_set(HAL::Led::ERROR, false);
        break;
      default: break;
    }
  }

  Mode mode_{Mode::BOOT};
  uint8_t safe_retries_{0};
  uint64_t last_transition_ms_{0};
  std::optional<Mode> requested_;
};

// ---------- Health & SOH ----------
class HealthManager {
public:
  void init() { last_health_ms_ = 0; }

  void tick(TelemetryStore& tlm, Mode mode) {
    uint64_t now = Util::now_ms();
    if (now - last_health_ms_ >= CFG::HEALTH_PERIOD_MS) {
      TlmSOH s{};
      s.ts_ms = (uint32_t)now;
      s.mode = mode;
      s.boot_count = boot_count_;
      s.safe_retries = safe_retries_;
      s.v_batt_mV = read_batt_mV_();
      s.temp_cdeg = read_temp_cdeg_();
      s.errors = errors_;
      tlm.push(reinterpret_cast<uint8_t*>(&s), sizeof(s));
      last_health_ms_ = now;
    }
  }

  void bump_error(uint32_t bit) { errors_ |= (1u << bit); }

private:
  uint16_t read_batt_mV_(); // via CAN->EPS
  int16_t  read_temp_cdeg_();

  uint64_t last_health_ms_{0};
  uint8_t  boot_count_{0};
  uint8_t  safe_retries_{0};
  uint32_t errors_{0};
};

// ---------- Scheduler (simple cooperative) ----------
class Scheduler {
public:
  void init() {
    next_beacon_ms_ = Util::now_ms() + CFG::BEACON_PERIOD_MS;
    next_scrub_ms_  = Util::now_ms() + CFG::SCRUB_PERIOD_MS;
  }

  template<typename TlmDumpFn, typename ScrubFn>
  void tick(TlmDumpFn beacon, ScrubFn scrub) {
    uint64_t now = Util::now_ms();
    if (now >= next_beacon_ms_) {
      beacon(); next_beacon_ms_ += CFG::BEACON_PERIOD_MS;
    }
    if (now >= next_scrub_ms_) {
      scrub(); next_scrub_ms_ += CFG::SCRUB_PERIOD_MS;
    }
  }

private:
  uint64_t next_beacon_ms_{0};
  uint64_t next_scrub_ms_{0};
};

// ---------- CDH App ----------
class CDHApp {
public:
  bool init() {
    HAL::systick_init_1ms();
    HAL::wdg_init();
    HAL::led_set(HAL::Led::STATUS, true);

    if (!HAL::can_init(CFG::CAN_BAUD, CFG::CDH_NODE_ID)) return false;
    if (!HAL::radio_init()) return false;
    if (!HAL::store_init()) return false;
    if (!fw_.init()) return false;

    tlm_.init(CFG::TLM_RING_BYTES);
    mode_.init();
    health_.init();
    sched_.init();

    router_.init(IDs::NODE_CDH);
    router_.bind_fw(&fw_);
    router_.bind_mode(&mode_);
    router_.bind_tlm(&tlm_);

    // Enter SAFE if last boot was abnormal (bootloader flag, omitted)
    mode_.request_mode(Mode::SAFE);
    return true;
  }

  void run() {
    uint8_t radio_rx_buf[512];

    while (true) {
      // 1) Kick watchdog
      HAL::wdg_kick();

      // 2) Service mode transitions
      mode_.tick();

      // 3) Poll radio for ground commands (primary source of FW updates)
      if (int n = HAL::radio_rx(radio_rx_buf, sizeof(radio_rx_buf), 0); n > 0) {
        router_.handle(radio_rx_buf, (size_t)n);
      }

      // 4) Poll CAN for inter-board messages (commands to CDH or forwarded)
      HAL::CanMsg cm;
      if (HAL::can_recv(cm, 0)) {
        // Example: route CAN commands addressed to CDH
        if ((cm.id & 0xE0) >> 5 == IDs::NODE_CDH && (cm.id & 0x3) == IDs::TYPE_CMD) {
          // Reassemble multi-frame (sequence in data[0]) - omitted
          // Then router_.handle(fullBuf, fullLen);
        }
        // Collect subsystem SOH frames into telemetry store
        if ((cm.id & 0x3) == IDs::TYPE_SOH) {
          tlm_.push(cm.data, cm.dlc);
        }
      }

      // 5) Periodic health & beacons & scrub
      health_.tick(tlm_, mode_.mode());
      sched_.tick(
        // Beacon
        [&](){
          // Compose a short beacon with SOH counters (example)
          TlmSOH s{}; s.ts_ms = (uint32_t)Util::now_ms(); s.mode = mode_.mode();
          HAL::radio_tx(reinterpret_cast<uint8_t*>(&s), sizeof(s));
        },
        // Scrub firmware bank (repair single-event upsets)
        [&](){ fw_.periodic_scrub(); }
      );

      // 6) Housekeeping: downlink queued telemetry opportunistically
      service_downlink_();

      // 7) Safing guardrails
      safing_checks_();

      // 8) LED heartbeat
      heartbeat_();
    }
  }

private:
  void service_downlink_() {
    // Example: send newest records until radio buffer full (omitted)
  }

  void safing_checks_() {
    // Example criteria: low battery, high temp, repeated command errors
    bool low_power = false; bool over_temp = false;
    if (low_power || over_temp) mode_.request_mode(Mode::SAFE);
  }

  void heartbeat_() {
    static uint64_t last = 0;
    uint64_t now = Util::now_ms();
    if (now - last > 500) {
      HAL::led_pulse(HAL::Led::STATUS, 50);
      last = now;
    }
  }

  FirmwareUpdate  fw_;
  TelemetryStore  tlm_;
  ModeManager     mode_;
  HealthManager   health_;
  Scheduler       sched_;
  CommandRouter   router_;
};

// ---------- Boot ----------
int main() {
  CDHApp app;
  if (!app.init()) {
    // Hard fail -> blink error & wait for watchdog reset
    while (1) { HAL::led_set(HAL::Led::ERROR, true); }
  }
  app.run();
  return 0;
}

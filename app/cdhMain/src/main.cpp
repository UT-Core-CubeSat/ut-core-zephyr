// main.cpp — CDH "logical flow" app skeleton (Zephyr + C++)
// NOTE: This is intentionally hardware-stubbed. Replace hw_*() with real drivers.

extern "C" {
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <string.h>
}

#include <cstdint>
#include <array>

/* =========================
 *  CAN envelope definitions
 * ========================= */
struct CanHeader {
  uint8_t prio;     // 3 bits
  uint8_t class_id; // 5 bits
  uint8_t src;      // 6 bits
  uint8_t dst;      // 6 bits
  uint16_t inst;    // 9 bits
};

struct CanMsg {
  CanHeader h{};
  uint8_t dlc{0};
  std::array<uint8_t, 8> data{};
};

static constexpr uint8_t CDH_NODE_ID = 0x01;
static constexpr uint8_t BROADCAST  = 0x00;

/* Message classes (per doc) */
enum : uint8_t {
  CLASS_HEARTBEAT = 0,
  CLASS_TIMESYNC  = 1,
  CLASS_COMMAND   = 2,
  CLASS_CMDRESP   = 3,
  CLASS_TELEM     = 4,
  CLASS_EVENT     = 5,
  CLASS_PARAMGET  = 6,
  CLASS_PARAMSET  = 7,
  CLASS_PARAMRESP = 8,
  CLASS_FILE      = 9,
  CLASS_BOOT      = 10,
  CLASS_HEALTH    = 11,
};

static inline CanHeader decode_id(uint32_t id29) {
  // id29 bits: [P:3][CLASS:5][SRC:6][DST:6][INST:9] = 29 bits total
  CanHeader h{};
  h.prio     = (id29 >> 26) & 0x7;
  h.class_id = (id29 >> 21) & 0x1F;
  h.src      = (id29 >> 15) & 0x3F;
  h.dst      = (id29 >> 9)  & 0x3F;
  h.inst     = (id29 >> 0)  & 0x1FF;
  return h;
}

static inline uint32_t encode_id(const CanHeader& h) {
  return ((uint32_t(h.prio & 0x7)     << 26) |
          (uint32_t(h.class_id & 0x1F) << 21) |
          (uint32_t(h.src & 0x3F)      << 15) |
          (uint32_t(h.dst & 0x3F)      << 9)  |
          (uint32_t(h.inst & 0x1FF)    << 0));
}

/* =========================
 *  Hardware stubs (replace)
 * ========================= */
static bool hw_can_read_raw(uint32_t& out_id29, uint8_t& out_dlc, uint8_t out_data[8]) {
  // TODO: replace with zephyr CAN driver receive; block or poll depending on your API choice.
  // Return true if a frame was read.
  (void)out_id29; (void)out_dlc; (void)out_data;
  k_sleep(K_MSEC(10));
  return false;
}

static void hw_can_send(const CanMsg& m) {
  // TODO: replace with zephyr CAN send
  (void)m;
}

static void hw_watchdog_kick() {
  // TODO: replace with zephyr watchdog driver kick
}

static uint32_t hw_uptime_ms() {
  return (uint32_t)k_uptime_get_32();
}

/* =========================
 *  System state / mode mgmt
 * ========================= */
enum class Mode : uint8_t { BOOT=0, SAFE=1, STANDBY=2, OP=3, FAULT=4, SHUTDOWN=5 };

struct SystemState {
  Mode mode{Mode::BOOT};
  uint32_t boot_ms{0};
  uint32_t hb_seq{0};
  uint32_t last_timesync_ms{0};

  // “health-ish” placeholders
  int16_t board_temp_c_x100{2500}; // 25.00C
  bool eps_alive{false};
  bool comms_alive{false};
  bool adcs_alive{false};
};

static SystemState g_state;

/* =========================
 *  Queues between services
 * ========================= */
K_MSGQ_DEFINE(can_rx_q, sizeof(CanMsg), 64, 4);

enum class LogType : uint8_t { TELEM, EVENT, ERROR };

struct LogItem {
  LogType type{};
  uint32_t t_ms{};
  uint16_t code{};
  std::array<uint8_t, 16> payload{}; // keep small for now
};

K_MSGQ_DEFINE(log_q, sizeof(LogItem), 128, 4);

/* =========================
 *  Command IDs (INST values)
 *  (examples from doc)
 * ========================= */
enum : uint16_t {
  CMD_PING          = 0,
  CMD_SET_MODE      = 1,
  CMD_GET_STATUS    = 2,
  CMD_REBOOT        = 3,
  // add the rest
};

/* =========================
 *  Helpers: responding
 * ========================= */
static void send_cmd_resp(uint8_t dst, uint16_t cmd_inst, int8_t status, uint8_t seq, const uint8_t* data=nullptr, uint8_t len=0) {
  CanMsg m{};
  m.h.prio     = 2;
  m.h.class_id = CLASS_CMDRESP;
  m.h.src      = CDH_NODE_ID;
  m.h.dst      = dst;
  m.h.inst     = cmd_inst;
  m.dlc        = MIN<uint8_t>(8, (uint8_t)(2 + len));
  m.data[0]    = (uint8_t)status;
  m.data[1]    = seq;
  for (uint8_t i = 0; i < len && (2+i) < 8; i++) m.data[2+i] = data[i];
  hw_can_send(m);
}

static void log_event(uint16_t code) {
  LogItem it{};
  it.type = LogType::EVENT;
  it.t_ms = hw_uptime_ms();
  it.code = code;
  (void)k_msgq_put(&log_q, &it, K_NO_WAIT);
}

/* =========================
 *  Mode manager logic
 * ========================= */
static void request_mode(Mode m) {
  if (g_state.mode == m) return;

  // Example policy: if going to OP, require EPS + COMMS alive; else bounce to SAFE.
  if (m == Mode::OP && !(g_state.eps_alive && g_state.comms_alive)) {
    g_state.mode = Mode::SAFE;
    log_event(/*ModeChange*/ 0x0001);
    return;
  }

  g_state.mode = m;
  log_event(/*ModeChange*/ 0x0001);

  // TODO: tell EPS to power stuff based on mode (stubbed)
  // ex: eps_set_rail("ADCS", m==Mode::OP);
}

/* =========================
 *  Param service (skeleton)
 * ========================= */
static void handle_param_get(const CanMsg& msg) {
  // msg.h.inst could be parameter key. Respond with ParamResp.
  // You might keep a small parameter store in FRAM later.
  CanMsg resp{};
  resp.h.prio     = 2;
  resp.h.class_id = CLASS_PARAMRESP;
  resp.h.src      = CDH_NODE_ID;
  resp.h.dst      = msg.h.src;
  resp.h.inst     = msg.h.inst;
  resp.dlc        = 2;
  resp.data[0]    = 0;     // status=OK
  resp.data[1]    = 0;     // seq placeholder
  hw_can_send(resp);
}

static void handle_param_set(const CanMsg& msg) {
  // Validate + apply; ack with ParamResp.
  (void)msg;
}

/* =========================
 *  File service (skeleton)
 * ========================= */
static void handle_file_msg(const CanMsg& msg) {
  // Implement FILE_BEGIN / FILE_CHUNK / FILE_ACK / FILE_END later.
  (void)msg;
}

/* =========================
 *  Command handlers
 * ========================= */
using CmdHandler = void(*)(const CanMsg&);

static void cmd_ping(const CanMsg& msg) {
  // respond success
  send_cmd_resp(msg.h.src, msg.h.inst, /*status*/0, /*seq*/msg.data[0]);
}

static void cmd_set_mode(const CanMsg& msg) {
  // Example: data[1] = desired mode enum
  uint8_t seq = msg.data[0];
  Mode desired = (Mode)msg.data[1];

  request_mode(desired);
  send_cmd_resp(msg.h.src, msg.h.inst, 0, seq);
}

static void cmd_get_status(const CanMsg& msg) {
  uint8_t seq = msg.data[0];
  uint8_t out[6]{};
  out[0] = (uint8_t)g_state.mode;
  uint32_t up = hw_uptime_ms();
  memcpy(&out[1], &up, 4);
  out[5] = (uint8_t)g_state.hb_seq;
  send_cmd_resp(msg.h.src, msg.h.inst, 0, seq, out, sizeof(out));
}

static void cmd_reboot(const CanMsg& msg) {
  uint8_t seq = msg.data[0];
  send_cmd_resp(msg.h.src, msg.h.inst, 0, seq);
  // TODO: trigger system reboot (sys_reboot) later
}

/* simple handler table */
static CmdHandler g_cmd_table[32] = {
  /*0*/ cmd_ping,
  /*1*/ cmd_set_mode,
  /*2*/ cmd_get_status,
  /*3*/ cmd_reboot,
  // rest null
};

/* =========================
 *  Router
 * ========================= */
static bool is_for_us(const CanMsg& msg) {
  return (msg.h.dst == CDH_NODE_ID) || (msg.h.dst == BROADCAST);
}

static void dispatch(const CanMsg& msg) {
  if (!is_for_us(msg)) return;

  switch (msg.h.class_id) {
    case CLASS_COMMAND: {
      uint16_t cmd_id = msg.h.inst;
      if (cmd_id < ARRAY_SIZE(g_cmd_table) && g_cmd_table[cmd_id]) {
        g_cmd_table[cmd_id](msg);
      } else {
        // unknown command -> respond error if addressed to us
        if (msg.h.dst == CDH_NODE_ID) {
          send_cmd_resp(msg.h.src, msg.h.inst, /*status*/-1, /*seq*/msg.data[0]);
        }
      }
    } break;

    case CLASS_PARAMGET:  handle_param_get(msg); break;
    case CLASS_PARAMSET:  handle_param_set(msg); break;
    case CLASS_FILE:      handle_file_msg(msg);  break;

    case CLASS_HEARTBEAT:
      // update liveness map based on msg.h.src, msg.data fields, etc.
      // g_state.eps_alive = ...
      break;

    case CLASS_TIMESYNC:
      // if CDH is master you might ignore; if not, update time base
      g_state.last_timesync_ms = hw_uptime_ms();
      break;

    default:
      break;
  }
}

/* =========================
 *  Threads
 * ========================= */
static void can_rx_thread(void*, void*, void*) {
  uint32_t id29{};
  uint8_t dlc{};
  uint8_t data[8]{};

  while (true) {
    if (!hw_can_read_raw(id29, dlc, data)) {
      continue;
    }

    CanMsg m{};
    m.h   = decode_id(id29);
    m.dlc = dlc;
    for (uint8_t i = 0; i < 8; i++) m.data[i] = data[i];

    // Drop if queue full (or block if you prefer K_FOREVER)
    (void)k_msgq_put(&can_rx_q, &m, K_NO_WAIT);
  }
}

static void router_thread(void*, void*, void*) {
  CanMsg m{};
  while (true) {
    k_msgq_get(&can_rx_q, &m, K_FOREVER);
    dispatch(m);
  }
}

static void storage_thread(void*, void*, void*) {
  LogItem it{};
  while (true) {
    k_msgq_get(&log_q, &it, K_FOREVER);
    // TODO: write to eMMC / filesystem later
    // For now, just print
    printk("[LOG] t=%u type=%u code=0x%04x\n", it.t_ms, (uint8_t)it.type, it.code);
  }
}

/* =========================
 *  Periodic work items
 * ========================= */
static k_work_delayable hb_work;
static k_work_delayable timesync_work;
static k_work_delayable wdog_work;
static k_work_delayable telem_work;

static void hb_work_fn(k_work*) {
  CanMsg hb{};
  hb.h.prio     = 2;
  hb.h.class_id = CLASS_HEARTBEAT;
  hb.h.src      = CDH_NODE_ID;
  hb.h.dst      = BROADCAST;
  hb.h.inst     = 0;

  // Example heartbeat payload (map to doc):
  // state, version_major, version_minor, uptime_s (2 bytes), seq
  hb.dlc = 8;
  hb.data[0] = (uint8_t)g_state.mode;
  hb.data[1] = 1; // ver major
  hb.data[2] = 0; // ver minor
  uint32_t up_s = hw_uptime_ms() / 1000U;
  memcpy(&hb.data[3], &up_s, 4);
  hb.data[7] = (uint8_t)(g_state.hb_seq++);

  hw_can_send(hb);

  // reschedule
  k_work_schedule(&hb_work, K_MSEC(500));
}

static void timesync_work_fn(k_work*) {
  // If CDH is the time master: broadcast unix time (or mission time)
  // Here: just broadcast uptime_ms as placeholder.
  CanMsg ts{};
  ts.h.prio     = 0;
  ts.h.class_id = CLASS_TIMESYNC;
  ts.h.src      = CDH_NODE_ID;
  ts.h.dst      = BROADCAST;
  ts.h.inst     = 0;

  ts.dlc = 8;
  uint32_t t = hw_uptime_ms();
  memcpy(&ts.data[0], &t, 4);
  memset(&ts.data[4], 0, 4);
  hw_can_send(ts);

  k_work_schedule(&timesync_work, K_SECONDS(1));
}

static void wdog_work_fn(k_work*) {
  hw_watchdog_kick();
  k_work_schedule(&wdog_work, K_MSEC(250)); // tune to your watchdog window
}

static void telem_work_fn(k_work*) {
  // Push a log item (later: also send CAN telemetry packets)
  LogItem it{};
  it.type = LogType::TELEM;
  it.t_ms = hw_uptime_ms();
  it.code = 0x1000; // CDH_SYS placeholder
  (void)k_msgq_put(&log_q, &it, K_NO_WAIT);

  k_work_schedule(&telem_work, K_MSEC(1000));
}

/* =========================
 *  Thread definitions
 * ========================= */
K_THREAD_STACK_DEFINE(can_rx_stack, 2048);
K_THREAD_STACK_DEFINE(router_stack, 2048);
K_THREAD_STACK_DEFINE(storage_stack, 2048);

static k_thread can_rx_t;
static k_thread router_t;
static k_thread storage_t;

int main() {
  g_state.boot_ms = hw_uptime_ms();
  printk("CDH app start\n");

  // Init delayed work
  k_work_init_delayable(&hb_work, hb_work_fn);
  k_work_init_delayable(&timesync_work, timesync_work_fn);
  k_work_init_delayable(&wdog_work, wdog_work_fn);
  k_work_init_delayable(&telem_work, telem_work_fn);

  // Start threads
  k_thread_create(&can_rx_t, can_rx_stack, K_THREAD_STACK_SIZEOF(can_rx_stack),
                  can_rx_thread, nullptr, nullptr, nullptr,
                  /*prio*/ 1, 0, K_NO_WAIT);

  k_thread_create(&router_t, router_stack, K_THREAD_STACK_SIZEOF(router_stack),
                  router_thread, nullptr, nullptr, nullptr,
                  /*prio*/ 2, 0, K_NO_WAIT);

  k_thread_create(&storage_t, storage_stack, K_THREAD_STACK_SIZEOF(storage_stack),
                  storage_thread, nullptr, nullptr, nullptr,
                  /*prio*/ 5, 0, K_NO_WAIT);

  // Kick off periodic jobs
  k_work_schedule(&hb_work, K_MSEC(200));
  k_work_schedule(&timesync_work, K_SECONDS(1));
  k_work_schedule(&wdog_work, K_MSEC(250));
  k_work_schedule(&telem_work, K_MSEC(1000));

  // Mode enters SAFE after boot unless you decide otherwise
  request_mode(Mode::SAFE);

  // Main thread can sleep forever; system lives in threads/work items
  k_sleep(K_FOREVER);
  return 0;
}

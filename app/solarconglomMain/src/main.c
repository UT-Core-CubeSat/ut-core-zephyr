// /*
//  * UT-CORE Solar Conglomerate — Magnetorquer Controller
//  *
//  * Hardware: STM32U5 microcontroller, Zephyr RTOS.
//  * Bus:      FDCAN1 at 500 kbit/s, 29-bit extended IDs.
//  * Actuators: 4 magnetorquer coils driven via PWM on PC2-PC5.
//  *
//  * This node receives commands from the ADCS subsystem over CAN to
//  * energise magnetorquer coils at a given duty cycle for a specified
//  * duration, then automatically de-energises them.
//  *
//  * Thread overview (highest to lowest priority):
//  *   PRIO 0  watchdog_thread   — kicks hardware watchdog; must never starve
//  *   PRIO 1  can_rx_thread     — drains hardware FIFO into SW queue
//  *   PRIO 2  can_process_thread— decodes & dispatches CAN frames
//  *   PRIO 3  heartbeat_thread  — 1 Hz heartbeat broadcast to CDH
//  *   PRIO 4  mtq_thread        — manages magnetorquer PWM timing
//  *
//  * CAN protocol (from ADCS):
//  *   msg_class = CLS_COMMAND (2)
//  *   opcode    = OP_MTQ_SET (0x20)
//  *   data[2]   = channel bitmask  (bit 0 = MTQ0/PC2 … bit 3 = MTQ3/PC5)
//  *   data[3]   = duty cycle 0-100 (percent)
//  *   data[4:5] = duration in milliseconds (big-endian uint16)
//  *
//  *   opcode    = OP_MTQ_STOP (0x21)
//  *   Immediately de-energise all channels.
//  */

// #include <zephyr/kernel.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/drivers/can.h>
// #include <zephyr/drivers/pwm.h>
// #include <zephyr/logging/log.h>
// #include <stm32u5xx.h>
// #include <stdbool.h>
// #include <string.h>

// /* -------- ADJUST THIS PATH TO YOUR PROJECT LAYOUT -------- */
// #include "board_config.h"

// /*
//  * If your common CAN protocol header lives in the shared repo, include it.
//  * Otherwise the opcode/class defines below act as a self-contained fallback.
//  */
// #if __has_include("can_proto.h")
// #include "can_proto.h"
// #else
// /* Minimal CAN protocol defines — keep in sync with can_proto.h */
// #define CAN_BROADCAST   0xFF

// #define CDH_ID          0x01
// #define EPS_ID          0x02
// #define COMMS_ID        0x03
// #define ADCS_ID         0x04
// #define MOTOR_ID        0x05
// #define GNSS_ID         0x06
// #define STAR_ID         0x07
// #define SOLAR_ID        0x08   /* This node */

// #define CLS_HEARTBEAT   0
// #define CLS_COMMAND     2
// #define CLS_CMD_RESP    3

// #define OP_HEARTBEAT    0x00
// #define OP_SET_MODE     0x01

// /*
//  * Build a 29-bit extended CAN ID.
//  *   [28:26] priority (3 bits)
//  *   [21:14] src      (8 bits)
//  *   [13: 6] dst      (8 bits)
//  *   [ 5: 0] msg_class(6 bits)
//  */
// #define CAN_ID_FULL(prio, src, dst, cls) \
//     ( (((uint32_t)(prio) & 0x07) << 26) | \
//       (((uint32_t)(src)  & 0xFF) << 14) | \
//       (((uint32_t)(dst)  & 0xFF) <<  6) | \
//       (((uint32_t)(cls)  & 0x3F)      ) )

// #define CAN_DST(d)         (((uint32_t)(d) & 0xFF) << 6)
// #define CAN_DST_MASK_29    (0xFFu << 6)

// static inline void can_fill_payload(struct can_frame *f, uint8_t src,
//                                     uint8_t op, uint8_t v0, uint8_t v1,
//                                     uint8_t v2, uint8_t v3, uint8_t v4,
//                                     uint8_t v5)
// {
//     f->data[0] = src;
//     f->data[1] = op;
//     f->data[2] = v0;
//     f->data[3] = v1;
//     f->data[4] = v2;
//     f->data[5] = v3;
//     f->data[6] = v4;
//     f->data[7] = v5;
//     f->dlc     = 8;
//     f->flags   = CAN_FRAME_IDE; /* 29-bit extended */
// }
// #endif /* can_proto.h fallback */

// LOG_MODULE_REGISTER(solar, LOG_LEVEL_INF);

// /* ================= NODE IDENTITY ================= */
// #define NODE_ID     SOLAR_ID
// #define DST_ME      NODE_ID
// #define PRIO_LOW    3

// /* ================= THREAD PRIORITIES ================= */
// #define PRIO_WATCHDOG   0
// #define PRIO_CAN_RX     1
// #define PRIO_CAN_PROC   2
// #define PRIO_HEARTBEAT  3
// #define PRIO_MTQ        4

// #define STACK_SIZE      1024

// /* ================= MAGNETORQUER OPCODES ================= */
// #define OP_MTQ_SET   0x20   /* Set channels: mask, duty%, duration_ms */
// #define OP_MTQ_STOP  0x21   /* Immediate all-stop                     */

// #define MTQ_CHANNELS  4
// #define MTQ_MAX_DURATION_MS  60000  /* Safety clamp: 60 s max */

// /* ================= DEVICE HANDLES ================= */

// #define CAN_NODE     DT_NODELABEL(fdcan1)
// #define GPIOA_NODE   DT_NODELABEL(gpioa)

// static const struct device *can_dev = DEVICE_DT_GET(CAN_NODE);
// static const struct device *gpioa   = DEVICE_DT_GET(GPIOA_NODE);

// /*
//  * PWM device handles for the 4 magnetorquer channels.
//  * These come from the DTS overlay which maps PC2-PC5 to timer PWM outputs.
//  *
//  * If PWM is not available on the current board revision, the code falls
//  * back to GPIO bang-bang (on/off only, no duty-cycle control).
//  */
// #if DT_NODE_HAS_STATUS(DT_NODELABEL(mtq_pwm0), okay)
//   #define MTQ_USE_PWM 1
//   static const struct pwm_dt_spec mtq_pwm[MTQ_CHANNELS] = {
//       PWM_DT_SPEC_GET(DT_NODELABEL(mtq_pwm0)),
//       PWM_DT_SPEC_GET(DT_NODELABEL(mtq_pwm1)),
//       PWM_DT_SPEC_GET(DT_NODELABEL(mtq_pwm2)),
//       PWM_DT_SPEC_GET(DT_NODELABEL(mtq_pwm3)),
//   };
// #else
//   #define MTQ_USE_PWM 0
//   /*
//    * GPIO fallback — drive PC2-PC5 as plain outputs.
//    * duty > 50% = ON, else OFF.  No intermediate control.
//    */
//   #define MTQ_GPIO_NODE DT_NODELABEL(gpioc)
//   static const struct device *mtq_gpio;
//   static const uint8_t mtq_pins[MTQ_CHANNELS] = { 2, 3, 4, 5 };
// #endif

// /* CAN transceiver control pins (same as CDH board) */
// #define PIN_SILENT   9
// #define PIN_SHDN     10

// /* Hardware watchdog kick interval (ms) */
// #define WATCHDOG_KICK_MS 15

// /* ================= CAN QUEUES ================= */

// CAN_MSGQ_DEFINE(rxq, 16);

// #define CAN_PROC_Q_LEN 16
// K_MSGQ_DEFINE(can_proc_q, sizeof(struct can_frame), CAN_PROC_Q_LEN, 4);

// /* ================= DATA STRUCTURES ================= */

// typedef struct {
//     uint8_t  priority;
//     uint8_t  msg_class;
//     uint8_t  src;
//     uint8_t  dst;
//     uint8_t  data[8];
//     uint8_t  dlc;
// } can_packet_t;

// /*
//  * mtq_command_t — describes one magnetorquer actuation request.
//  *
//  * Written by handle_mtq_set(), consumed by mtq_thread.
//  * Protected by mtq_sem (binary semaphore used as a signal).
//  */
// typedef struct {
//     uint8_t  channel_mask;    /* Bit 0 = MTQ0 (PC2) … bit 3 = MTQ3 (PC5) */
//     uint8_t  duty_pct;        /* 0-100 percent duty cycle                  */
//     uint16_t duration_ms;     /* How long to energise (ms)                 */
// } mtq_command_t;

// /* Shared command buffer; written by CAN handler, read by mtq_thread. */
// static volatile mtq_command_t mtq_pending = {0};

// /* Signalled when a new MTQ command is ready. */
// K_SEM_DEFINE(mtq_sem, 0, 1);

// /* Signalled to force an immediate stop from the CAN handler. */
// static volatile bool mtq_stop_flag = false;

// /* ================= MODE STATE ================= */
// typedef enum {
//     MODE_SETUP,
//     MODE_STANDARD,
//     MODE_MISSION,
//     MODE_ERROR
// } node_mode_t;

// volatile node_mode_t current_mode = MODE_SETUP;

// /* ================= THREAD STACKS ================= */
// K_THREAD_STACK_DEFINE(watchdog_stack,  STACK_SIZE);
// K_THREAD_STACK_DEFINE(can_rx_stack,    STACK_SIZE);
// K_THREAD_STACK_DEFINE(can_proc_stack,  STACK_SIZE);
// K_THREAD_STACK_DEFINE(heartbeat_stack, STACK_SIZE);
// K_THREAD_STACK_DEFINE(mtq_stack,       STACK_SIZE);

// static struct k_thread watchdog_td;
// static struct k_thread can_rx_td;
// static struct k_thread can_proc_td;
// static struct k_thread heartbeat_td;
// static struct k_thread mtq_td;

// /* ================= FORWARD DECLARATIONS ================= */
// static void can_decode(const struct can_frame *f, can_packet_t *pkt);
// static void can_dispatch(const can_packet_t *pkt);
// static void handle_command(const can_packet_t *pkt);
// static void handle_heartbeat(const can_packet_t *pkt);

// /* ===================================================== */
// /* ================= UTILITY / CAN FUNCTIONS ============ */
// /* ===================================================== */

// static void send_simple(uint8_t dst, uint8_t cls, uint8_t op, uint8_t val)
// {
//     struct can_frame f = {0};
//     f.id    = CAN_ID_FULL(PRIO_LOW, NODE_ID, dst, cls);
//     f.flags = CAN_FRAME_IDE;
//     can_fill_payload(&f, NODE_ID, op, val, 0, 0, 0, 0, 0);
//     can_send(can_dev, &f, K_NO_WAIT, NULL, NULL);
// }

// static void tcan3403_wakeup(void)
// {
//     gpio_pin_configure(gpioa, PIN_SHDN,   GPIO_OUTPUT_INACTIVE);
//     gpio_pin_configure(gpioa, PIN_SILENT, GPIO_OUTPUT_INACTIVE);
//     k_msleep(1);
//     LOG_INF("TCAN3403 Awake");
// }

// static void can_setup(void)
// {
//     if (!device_is_ready(can_dev)) {
//         LOG_ERR("CAN not ready");
//         return;
//     }

//     can_set_bitrate(can_dev, 500000);
//     can_set_mode(can_dev, CAN_MODE_NORMAL);
//     can_start(can_dev);

//     /* Accept frames addressed to this node or broadcast */
//     const struct can_filter to_me = {
//         .id    = CAN_DST(DST_ME),
//         .mask  = CAN_DST_MASK_29,
//         .flags = CAN_FILTER_IDE,
//     };
//     const struct can_filter bcast = {
//         .id    = CAN_DST(CAN_BROADCAST),
//         .mask  = CAN_DST_MASK_29,
//         .flags = CAN_FILTER_IDE,
//     };

//     can_add_rx_filter_msgq(can_dev, &rxq, &to_me);
//     can_add_rx_filter_msgq(can_dev, &rxq, &bcast);

//     LOG_INF("CAN initialised — filters installed");
// }

// static void can_decode(const struct can_frame *f, can_packet_t *pkt)
// {
//     uint32_t id    = f->id;
//     pkt->priority  = (id >> 26) & 0x07;
//     pkt->src       = (id >> 14) & 0xFF;
//     pkt->dst       = (id >>  6) & 0xFF;
//     pkt->msg_class =  id        & 0x3F;
//     pkt->dlc       = f->dlc;
//     memcpy(pkt->data, f->data, f->dlc);
// }

// static void can_dispatch(const can_packet_t *pkt)
// {
//     switch (pkt->msg_class) {
//         case CLS_HEARTBEAT: handle_heartbeat(pkt); break;
//         case CLS_COMMAND:   handle_command(pkt);   break;
//         default:
//             LOG_WRN("Unhandled msg class: %d", pkt->msg_class);
//             break;
//     }
// }

// /* ===================================================== */
// /* ================= MAGNETORQUER CONTROL =============== */
// /* ===================================================== */

// /*
//  * mtq_set_channel() — Set one magnetorquer channel's duty cycle.
//  *
//  * @ch:       Channel index 0-3 (maps to PC2-PC5).
//  * @duty_pct: 0 = off, 1-100 = percent duty cycle.
//  *
//  * With PWM: sets the hardware PWM duty cycle directly.
//  * Without PWM (GPIO fallback): on if duty > 50, off otherwise.
//  */
// static void mtq_set_channel(uint8_t ch, uint8_t duty_pct)
// {
//     if (ch >= MTQ_CHANNELS) {
//         return;
//     }

// #if MTQ_USE_PWM
//     const struct pwm_dt_spec *spec = &mtq_pwm[ch];
//     uint32_t period_ns = spec->period;
//     uint32_t pulse_ns  = (uint32_t)((uint64_t)period_ns * duty_pct / 100);

//     int ret = pwm_set_pulse_dt(spec, pulse_ns);
//     if (ret < 0) {
//         LOG_ERR("PWM set ch%d failed: %d", ch, ret);
//     }
// #else
//     gpio_pin_set(mtq_gpio, mtq_pins[ch], duty_pct > 50 ? 1 : 0);
// #endif
// }

// /* Turn off all magnetorquer channels. */
// static void mtq_all_off(void)
// {
//     for (uint8_t ch = 0; ch < MTQ_CHANNELS; ch++) {
//         mtq_set_channel(ch, 0);
//     }
//     LOG_INF("MTQ: all channels OFF");
// }

// /* Initialise magnetorquer outputs. */
// static void mtq_init(void)
// {
// #if MTQ_USE_PWM
//     for (uint8_t ch = 0; ch < MTQ_CHANNELS; ch++) {
//         if (!device_is_ready(mtq_pwm[ch].dev)) {
//             LOG_ERR("PWM device for MTQ ch%d not ready", ch);
//         }
//     }
// #else
//     mtq_gpio = DEVICE_DT_GET(MTQ_GPIO_NODE);
//     if (!device_is_ready(mtq_gpio)) {
//         LOG_ERR("MTQ GPIO port not ready");
//         return;
//     }
//     for (uint8_t ch = 0; ch < MTQ_CHANNELS; ch++) {
//         gpio_pin_configure(mtq_gpio, mtq_pins[ch], GPIO_OUTPUT_INACTIVE);
//     }
// #endif
//     mtq_all_off();
//     LOG_INF("MTQ outputs initialised (%s mode)", MTQ_USE_PWM ? "PWM" : "GPIO");
// }

// /* ===================================================== */
// /* ================= MESSAGE HANDLERS =================== */
// /* ===================================================== */

// static void handle_heartbeat(const can_packet_t *pkt)
// {
//     LOG_DBG("Heartbeat from 0x%02X", pkt->src);
// }

// /*
//  * handle_command() — Dispatch incoming commands addressed to this node.
//  *
//  * Payload layout (matches can_proto.h convention):
//  *   data[0] = src node ID
//  *   data[1] = opcode
//  *   data[2…] = opcode-specific arguments
//  */
// static void handle_command(const can_packet_t *pkt)
// {
//     uint8_t opcode = pkt->data[1];

//     switch (opcode) {

//         case OP_MTQ_SET: {
//             /*
//              * data[2] = channel bitmask (bits 0-3)
//              * data[3] = duty cycle 0-100
//              * data[4] = duration_ms high byte
//              * data[5] = duration_ms low byte
//              */
//             uint8_t  mask     = pkt->data[2] & 0x0F;
//             uint8_t  duty     = pkt->data[3];
//             uint16_t dur_ms   = ((uint16_t)pkt->data[4] << 8) | pkt->data[5];

//             if (duty > 100) {
//                 duty = 100;
//             }
//             if (dur_ms > MTQ_MAX_DURATION_MS) {
//                 dur_ms = MTQ_MAX_DURATION_MS;
//                 LOG_WRN("MTQ duration clamped to %u ms", MTQ_MAX_DURATION_MS);
//             }

//             LOG_INF("MTQ SET: mask=0x%X duty=%u%% dur=%u ms",
//                      mask, duty, dur_ms);

//             mtq_pending.channel_mask = mask;
//             mtq_pending.duty_pct     = duty;
//             mtq_pending.duration_ms  = dur_ms;
//             mtq_stop_flag = false;

//             k_sem_give(&mtq_sem);   /* Wake mtq_thread */

//             /* ACK back to sender */
//             send_simple(pkt->src, CLS_CMD_RESP, OP_MTQ_SET, 0x00);
//             break;
//         }

//         case OP_MTQ_STOP:
//             LOG_INF("MTQ STOP command received");
//             mtq_stop_flag = true;
//             mtq_all_off();
//             send_simple(pkt->src, CLS_CMD_RESP, OP_MTQ_STOP, 0x00);
//             break;

//         case OP_SET_MODE:
//             current_mode = pkt->data[2];
//             LOG_INF("Mode changed to %d", current_mode);
//             break;

//         default:
//             LOG_WRN("Unknown opcode: 0x%02X", opcode);
//             break;
//     }
// }

// /* ===================================================== */
// /* ================= THREADS ============================ */
// /* ===================================================== */

// /*
//  * watchdog_thread() — Highest priority; kicks the hardware watchdog.
//  */
// void watchdog_thread(void *a, void *b, void *c)
// {
//     LOG_INF("watchdog_thread started");

//     while (1) {
//         /* TODO: gpio_pin_toggle(gpioa, WATCHDOG_PIN); */
//         LOG_DBG("watchdog kick");
//         k_sleep(K_MSEC(WATCHDOG_KICK_MS));
//     }
// }

// /*
//  * can_rx_thread() — Drain hardware CAN FIFO into the SW processing queue.
//  */
// void can_rx_thread(void *a, void *b, void *c)
// {
//     LOG_INF("can_rx_thread started");
//     struct can_frame rx;

//     while (1) {
//         if (current_mode == MODE_STANDARD ||
//             current_mode == MODE_MISSION)
//         {
//             if (k_msgq_get(&rxq, &rx, K_FOREVER) == 0) {
//                 LOG_DBG("RX: id=0x%08X dlc=%d", rx.id, rx.dlc);
//                 if (k_msgq_put(&can_proc_q, &rx, K_NO_WAIT) != 0) {
//                     LOG_WRN("CAN proc queue full — frame dropped");
//                 }
//             }
//         } else {
//             k_sleep(K_MSEC(200));
//         }
//     }
// }

// /*
//  * can_process_thread() — Decode and dispatch CAN frames from the SW queue.
//  */
// void can_process_thread(void *a, void *b, void *c)
// {
//     LOG_INF("can_process_thread started");
//     struct can_frame frame;
//     can_packet_t pkt;

//     while (1) {
//         if (k_msgq_get(&can_proc_q, &frame, K_FOREVER) == 0) {
//             can_decode(&frame, &pkt);

//             if (pkt.dst != NODE_ID && pkt.dst != CAN_BROADCAST) {
//                 continue;
//             }

//             can_dispatch(&pkt);
//         }
//     }
// }

// /*
//  * heartbeat_thread() — Send a 1 Hz heartbeat to CDH (broadcast).
//  *
//  * Uses uptime delta to avoid drift accumulation.
//  */
// void heartbeat_thread(void *a, void *b, void *c)
// {
//     LOG_INF("heartbeat_thread started");
//     int64_t last_hb = 0;

//     while (1) {
//         if (current_mode == MODE_STANDARD ||
//             current_mode == MODE_MISSION)
//         {
//             int64_t now = k_uptime_get();
//             if (now - last_hb >= 1000) {
//                 last_hb = now;
//                 send_simple(CAN_BROADCAST, CLS_HEARTBEAT, OP_HEARTBEAT, 0);
//                 LOG_DBG("Heartbeat sent");
//             }
//         }
//         k_sleep(K_MSEC(50));
//     }
// }

// /*
//  * mtq_thread() — Magnetorquer actuation manager.
//  *
//  * Waits on mtq_sem. When signalled:
//  *   1. Copies the pending command atomically.
//  *   2. Energises the requested channels at the specified duty cycle.
//  *   3. Sleeps for the requested duration (checking mtq_stop_flag
//  *      every 50 ms so a STOP command can interrupt early).
//  *   4. De-energises all channels.
//  *
//  * Only one actuation runs at a time. A new OP_MTQ_SET while an actuation
//  * is in progress will preempt: the stop flag interrupts the sleep loop,
//  * channels are turned off, and the new command takes over on the next
//  * semaphore wake.
//  */
// void mtq_thread(void *a, void *b, void *c)
// {
//     LOG_INF("mtq_thread started");

//     while (1) {
//         /* Block until a new command arrives */
//         k_sem_take(&mtq_sem, K_FOREVER);

//         /* Snapshot the command (CAN handler may overwrite pending) */
//         mtq_command_t cmd;
//         unsigned int key = irq_lock();
//         cmd = *(const mtq_command_t *)&mtq_pending;
//         irq_unlock(key);

//         if (cmd.channel_mask == 0 || cmd.duty_pct == 0 || cmd.duration_ms == 0) {
//             LOG_WRN("MTQ: no-op command (mask=0x%X duty=%u dur=%u)",
//                      cmd.channel_mask, cmd.duty_pct, cmd.duration_ms);
//             continue;
//         }

//         LOG_INF("MTQ: energising mask=0x%X duty=%u%% for %u ms",
//                  cmd.channel_mask, cmd.duty_pct, cmd.duration_ms);

//         /* Energise requested channels */
//         for (uint8_t ch = 0; ch < MTQ_CHANNELS; ch++) {
//             if (cmd.channel_mask & (1u << ch)) {
//                 mtq_set_channel(ch, cmd.duty_pct);
//             }
//         }

//         /* Wait for the requested duration, checking stop flag every 50 ms */
//         uint16_t remaining = cmd.duration_ms;

//         while (remaining > 0 && !mtq_stop_flag) {
//             uint16_t step = (remaining > 50) ? 50 : remaining;
//             k_sleep(K_MSEC(step));
//             remaining -= step;
//         }

//         /* De-energise */
//         mtq_all_off();

//         if (mtq_stop_flag) {
//             LOG_INF("MTQ: stopped early by command");
//             mtq_stop_flag = false;
//         } else {
//             LOG_INF("MTQ: actuation complete");
//         }
//     }
// }

// /* ===================================================== */
// /* ================= BOOT SEQUENCE ====================== */
// /* ===================================================== */

// static void setup_mode_init(void)
// {
//     LOG_INF("Entering SETUP MODE");

//     tcan3403_wakeup();
//     can_setup();
//     mtq_init();

//     LOG_INF("Setup complete — waiting 2 s for bus stabilisation");
//     k_sleep(K_MSEC(2000));

//     current_mode = MODE_STANDARD;
//     LOG_INF("Entered STANDARD mode");
// }

// /* ===================================================== */
// /* ================= MAIN =============================== */
// /* ===================================================== */

// int main(void)
// {
//     LOG_INF("UT-CORE Solar Conglomerate Booting...");

//     /* Prio 0 — watchdog */
//     k_thread_create(&watchdog_td, watchdog_stack, STACK_SIZE,
//                     watchdog_thread, NULL, NULL, NULL,
//                     PRIO_WATCHDOG, 0, K_NO_WAIT);

//     /* Prio 1 — CAN RX */
//     k_thread_create(&can_rx_td, can_rx_stack, STACK_SIZE,
//                     can_rx_thread, NULL, NULL, NULL,
//                     PRIO_CAN_RX, 0, K_NO_WAIT);

//     /* Prio 2 — CAN decode & dispatch */
//     k_thread_create(&can_proc_td, can_proc_stack, STACK_SIZE,
//                     can_process_thread, NULL, NULL, NULL,
//                     PRIO_CAN_PROC, 0, K_NO_WAIT);

//     /* Prio 3 — Heartbeat to CDH */
//     k_thread_create(&heartbeat_td, heartbeat_stack, STACK_SIZE,
//                     heartbeat_thread, NULL, NULL, NULL,
//                     PRIO_HEARTBEAT, 0, K_NO_WAIT);

//     /* Prio 4 — Magnetorquer actuation manager */
//     k_thread_create(&mtq_td, mtq_stack, STACK_SIZE,
//                     mtq_thread, NULL, NULL, NULL,
//                     PRIO_MTQ, 0, K_NO_WAIT);

//     setup_mode_init();

//     while (1) {
//         k_sleep(K_FOREVER);
//     }
// }

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mtq_test);

static const struct pwm_dt_spec mtq0 = PWM_DT_SPEC_GET(DT_NODELABEL(mtq_pwm0));
const struct device *gpioa = DEVICE_DT_GET(DT_NODELABEL(gpioa));
const struct device *gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));

int main(void)
{
    /* MUX select high */
    gpio_pin_configure(gpioa, 5, GPIO_OUTPUT_ACTIVE);

    /*
     * STEP 1: Toggle PC2 as raw GPIO first.
     * If you don't see this on the scope, the problem is electrical.
     * Comment this block out once confirmed.
     */
    gpio_pin_configure(gpioc, 2, GPIO_OUTPUT_ACTIVE);
    for (int i = 0; i < 20; i++) {
        gpio_pin_toggle(gpioc, 2);
        k_sleep(K_MSEC(500));
    }
    LOG_INF("GPIO toggle done — switching to PWM");

    /*
     * STEP 2: Now try PWM on the same pin.
     * The pinmux switches from GPIO to TIM3_CH3 AF2 when PWM inits.
     */
    if (!device_is_ready(mtq0.dev)) {
        LOG_ERR("PWM device not ready");
        return -1;
    }

    LOG_INF("period = %u ns", mtq0.period);

    /* Try channel as-is from overlay (2) */
    int ret = pwm_set_pulse_dt(&mtq0, mtq0.period / 2);
    LOG_INF("pwm_set_pulse_dt returned %d", ret);

    if (ret != 0) {
        /*
         * If it failed, the channel index is probably wrong.
         * Try setting it manually with channel 3 (1-based).
         */
        LOG_WRN("Retrying with channel 3 (1-based)...");
        ret = pwm_set(mtq0.dev, 3, mtq0.period, mtq0.period / 2, 0);
        LOG_INF("pwm_set ch3 returned %d", ret);
    }

    while (1) {
        k_sleep(K_FOREVER);
    }
}

// #include <zephyr/drivers/gpio.h>

// const struct device *gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));
// const struct device *gpioa = DEVICE_DT_GET(DT_NODELABEL(gpioa));

// int main(void)
// {
//     gpio_pin_configure(gpioc, 5, GPIO_OUTPUT_ACTIVE);
//     gpio_pin_configure(gpioa, 5, GPIO_OUTPUT_ACTIVE);
//     gpio_pin_set(gpioa, 5, 1);
//     while (1) {
//         gpio_pin_set(gpioc, 5, 1);
//         k_sleep(K_MSEC(500));
//         gpio_pin_set(gpioc, 5, 0);
//         k_sleep(K_MSEC(500));
//     }
// }
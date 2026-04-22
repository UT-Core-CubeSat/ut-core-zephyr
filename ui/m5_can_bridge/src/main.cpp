// M5 Stamp S3 CAN-to-UDP bridge for UT-CORE.
// Listens on the CAN bus via the ESP32-S3 TWAI peripheral and forwards every
// received frame as a 21-byte UDP packet to the laptop ground station.
//
// Wiring: M5 Stamp S3 <-> SN65HVD230 (or equivalent 3.3V CAN transceiver) <-> CAN bus
//   TWAI_TX_GPIO -> transceiver TXD
//   TWAI_RX_GPIO -> transceiver RXD
//   3V3 / GND    -> transceiver VCC / GND
//   Transceiver CANH/CANL -> bus CANH/CANL
//
// SIMULATE mode: define SIMULATE below to skip TWAI entirely and send synthetic
// CAN frames over UDP. Use this to test the ground station without any hardware.

#define SIMULATE   // comment out when connected to a real CAN bus

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#ifndef SIMULATE
#include "driver/twai.h"
#endif

// ---------- USER CONFIG ----------
static const char*     WIFI_SSID     = "ARICKSLAPTOP";
static const char*     WIFI_PASSWORD = "CUBESATISCOOL";
static const IPAddress LAPTOP_IP(192, 168, 137, 1);  // typical Windows hotspot gateway
static const uint16_t  UDP_PORT      = 5005;

#ifndef SIMULATE
static const gpio_num_t TWAI_TX_GPIO = GPIO_NUM_5;
static const gpio_num_t TWAI_RX_GPIO = GPIO_NUM_4;
static const twai_mode_t TWAI_MODE   = TWAI_MODE_LISTEN_ONLY;
#endif

static const int STATUS_LED = 21;
// ---------------------------------

static WiFiUDP udp;

#pragma pack(push, 1)
struct udp_can_frame {
    uint64_t timestamp_ms;
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  data[8];
};
#pragma pack(pop)
static_assert(sizeof(udp_can_frame) == 21, "udp_can_frame must be 21 bytes");

// ---------- CAN protocol constants (mirrors common/can_proto.h) ----------
#define CAN_PRIO(p)  (((uint32_t)(p)  & 0x07U) << 26)
#define CAN_SRC(s)   (((uint32_t)(s)  & 0xFFU) << 14)
#define CAN_DST(d)   (((uint32_t)(d)  & 0xFFU) <<  6)
#define CAN_CLASS(c) (((uint32_t)(c)  & 0x3FU)      )
#define CAN_ID_FULL(prio, src, dst, cls) \
    (CAN_PRIO(prio) | CAN_SRC(src) | CAN_DST(dst) | CAN_CLASS(cls))

#define CDH_ID    0x01
#define EPS_ID    0x02
#define COMMS_ID  0x03
#define ADCS_ID   0x04
#define MOTOR_ID  0x05
#define GNSS_ID   0x06
#define STAR_ID   0x07
#define SOLAR_ID  0x08
#define BCAST     0xFF

#define CLS_HEARTBEAT  0
#define CLS_COMMAND    2
#define CLS_TELEMETRY  4
#define CLS_HEALTH    10

#define OP_PING       0x01
#define OP_HEARTBEAT  0x30

// -------------------------------------------------------------------------

static void wifi_connect() {
    Serial.printf("Connecting to WiFi '%s'", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nConnected. IP=%s  -> %s:%u\n",
                  WiFi.localIP().toString().c_str(),
                  LAPTOP_IP.toString().c_str(), UDP_PORT);
}

static void send_frame(uint32_t can_id, uint8_t d0, uint8_t d1,
                        uint8_t d2, uint8_t d3, uint8_t d4,
                        uint8_t d5, uint8_t d6, uint8_t d7) {
    udp_can_frame pkt = {};
    pkt.timestamp_ms = (uint64_t)millis();
    pkt.can_id       = can_id;
    pkt.dlc          = 8;
    pkt.data[0] = d0; pkt.data[1] = d1; pkt.data[2] = d2; pkt.data[3] = d3;
    pkt.data[4] = d4; pkt.data[5] = d5; pkt.data[6] = d6; pkt.data[7] = d7;
    udp.beginPacket(LAPTOP_IP, UDP_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    udp.endPacket();
    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
}

// -------------------------------------------------------------------------
#ifdef SIMULATE

// Simulated sensor state that drifts over time to make the plots interesting.
static float sim_temp_c   = 22.0f;    // degC, drifts ± a few degrees
static float sim_vbus_mv  = 3300.0f;  // mV, ripples around 3.3 V
static float sim_motor_rpm = 0.0f;
static uint8_t sim_mode = 1;

static void simulate_tick() {
    static uint32_t last_hb_ms    = 0;
    static uint32_t last_tlm_ms   = 0;
    static uint32_t last_health_ms = 0;
    static uint8_t  hb_node_idx   = 0;
    static uint32_t tick          = 0;

    const uint8_t nodes[] = {
        CDH_ID, EPS_ID, COMMS_ID, ADCS_ID,
        MOTOR_ID, GNSS_ID, STAR_ID, SOLAR_ID
    };
    const uint8_t N = sizeof(nodes);
    uint32_t now = millis();

    // Drift simulated values.
    tick++;
    sim_temp_c  += sinf(tick * 0.05f) * 0.3f;
    sim_vbus_mv += sinf(tick * 0.03f) * 20.0f;
    sim_motor_rpm = 500.0f + 300.0f * sinf(tick * 0.07f);

    // Heartbeats: round-robin one node every 500 ms, so each node beats ~4 s.
    if (now - last_hb_ms >= 500) {
        last_hb_ms = now;
        uint8_t src = nodes[hb_node_idx % N];
        hb_node_idx++;
        uint32_t id = CAN_ID_FULL(1, src, BCAST, CLS_HEARTBEAT);
        // data[0]=src, data[1]=opcode(HEARTBEAT), data[2]=mode, rest=0
        send_frame(id, src, OP_HEARTBEAT, sim_mode, 0, 0, 0, 0, 0);
        Serial.printf("[SIM] HB from 0x%02X\n", src);
    }

    // Telemetry: CDH sends temp + vbus every 1 s.
    if (now - last_tlm_ms >= 1000) {
        last_tlm_ms = now;
        uint32_t id = CAN_ID_FULL(2, CDH_ID, BCAST, CLS_TELEMETRY);
        // Pack temp as Q4 fixed-point (value * 16) in p2:p3 (little-endian).
        int16_t temp_q4 = (int16_t)(sim_temp_c * 16.0f);
        uint16_t vbus   = (uint16_t)sim_vbus_mv;
        send_frame(id,
            CDH_ID, 0x01,                        // src, opcode
            (uint8_t)(temp_q4 & 0xFF),           // p2: temp low byte
            (uint8_t)((temp_q4 >> 8) & 0xFF),    // p3: temp high byte
            (uint8_t)(vbus & 0xFF),               // p4: vbus low byte
            (uint8_t)((vbus >> 8) & 0xFF),        // p5: vbus high byte
            0, 0);
        Serial.printf("[SIM] TLM temp=%.1f C  vbus=%.0f mV\n",
                      sim_temp_c, sim_vbus_mv);
    }

    // Health: EPS sends power readings every 2 s.
    if (now - last_health_ms >= 2000) {
        last_health_ms = now;
        uint32_t id = CAN_ID_FULL(2, EPS_ID, CDH_ID, CLS_HEALTH);
        uint16_t rpm = (uint16_t)sim_motor_rpm;
        send_frame(id,
            EPS_ID, 0x02,
            (uint8_t)(rpm & 0xFF),
            (uint8_t)((rpm >> 8) & 0xFF),
            0x01,  // load switches bitmask (mock)
            0, 0, 0);
        Serial.printf("[SIM] HEALTH motor_rpm=%.0f\n", sim_motor_rpm);
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== SIMULATE mode ===");
    pinMode(STATUS_LED, OUTPUT);
    wifi_connect();
    udp.begin(UDP_PORT);
}

void loop() {
    simulate_tick();
    if (WiFi.status() != WL_CONNECTED) wifi_connect();
    delay(50);  // ~20 Hz tick
}

// -------------------------------------------------------------------------
#else  // real TWAI

static bool twai_start_500k() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE);
    g.rx_queue_len = 32;
    g.tx_queue_len = 0;

    twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("twai_driver_install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("twai_start failed");
        return false;
    }
    Serial.println("TWAI up at 500 kbps");
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(200);
    pinMode(STATUS_LED, OUTPUT);
    wifi_connect();
    udp.begin(UDP_PORT);
    if (!twai_start_500k()) {
        while (true) {
            digitalWrite(STATUS_LED, HIGH); delay(100);
            digitalWrite(STATUS_LED, LOW);  delay(100);
        }
    }
}

void loop() {
    twai_message_t msg;
    esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(10));
    if (err != ESP_OK) {
        if (WiFi.status() != WL_CONNECTED) wifi_connect();
        return;
    }

    send_frame(msg.identifier, msg.data[0], msg.data[1], msg.data[2],
               msg.data[3], msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
}

#endif  // SIMULATE

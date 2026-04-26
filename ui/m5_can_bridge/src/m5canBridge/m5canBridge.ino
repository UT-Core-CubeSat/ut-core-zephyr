#include <Arduino.h>
#include "driver/twai.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// ---------- WIFI CONFIG ----------
static const char* WIFI_SSID     = "braydonxps";
static const char* WIFI_PASSWORD = "chimichanga";
static const IPAddress LAPTOP_IP(192, 168, 137, 1);
static const uint16_t UDP_PORT   = 5005;
// --------------------------------

static WiFiUDP udp;
static bool udp_ready = false;

#pragma pack(push, 1)
struct udp_can_frame {
    uint64_t timestamp_ms;
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
};
#pragma pack(pop)

// ---------- CONFIG ----------
static const gpio_num_t TWAI_TX_GPIO = GPIO_NUM_8;
static const gpio_num_t TWAI_RX_GPIO = GPIO_NUM_4;
static const int STATUS_LED = 21;
static constexpr uint32_t HEARTBEAT_PERIOD_MS = 1000U;
static constexpr uint8_t CAN_BROADCAST = 0xFF;
static constexpr uint8_t COMMS_ID = 0x03;
static constexpr uint8_t CLS_HEARTBEAT = 0x00;
static constexpr uint8_t OP_HEARTBEAT = 0x30;
static constexpr uint8_t PRIO_LOW = 6;
static constexpr twai_mode_t TWAI_RUN_MODE = TWAI_MODE_NORMAL;  // ACK-capable node.
static constexpr bool ENABLE_TX_TEST = true;
static constexpr bool ENABLE_LOCAL_GPIO_LOOPBACK_TEST = false;   // Only true for raw GPIO test.
static constexpr uint8_t TX_TARGET_MOTOR = 1;                    // 1..4, or 0 for all motors.
static constexpr int16_t TX_RPM_REF_LOW = 1200;                  // Signed RPM command.
static constexpr int16_t TX_RPM_REF_HIGH = 5000;                 // Signed RPM command.
static constexpr uint32_t TX_RPM_SWITCH_MS = 5000;               // Toggle command every 5 seconds.
// ----------------------------

static const char* twai_mode_name() {
    switch (TWAI_RUN_MODE) {
        case TWAI_MODE_NORMAL: return "NORMAL / ACKING";
        case TWAI_MODE_NO_ACK: return "NO_ACK TEST";
        case TWAI_MODE_LISTEN_ONLY: return "LISTEN_ONLY";
        default: return "UNKNOWN";
    }
}

static const char* twai_state_name(twai_state_t st) {
    switch (st) {
        case TWAI_STATE_STOPPED: return "STOPPED";
        case TWAI_STATE_RUNNING: return "RUNNING";
        case TWAI_STATE_BUS_OFF: return "BUS_OFF";
        case TWAI_STATE_RECOVERING: return "RECOVERING";
        default: return "UNKNOWN";
    }
}


static bool twai_init() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        TWAI_TX_GPIO,
        TWAI_RX_GPIO,
        TWAI_RUN_MODE
    );

    g.rx_queue_len = 64;
    g.tx_queue_len = 8;

    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) {
        Serial.println("TWAI driver install failed");
        return false;
    }

    if (twai_start() != ESP_OK) {
        Serial.println("TWAI start failed");
        return false;
    }

    return true;
}

static bool twai_ensure_running() {
    twai_status_info_t st = {};
    if (twai_get_status_info(&st) != ESP_OK) {
        Serial.println("TWAI status read failed");
        return false;
    }

    if (st.state == TWAI_STATE_RUNNING) {
        return true;
    }

    if (st.state == TWAI_STATE_STOPPED) {
        const esp_err_t ret = twai_start();
        if (ret == ESP_OK) {
            Serial.println("TWAI restarted from STOPPED");
            return true;
        }
        Serial.printf("TWAI restart failed ret=%d\n", (int)ret);
        return false;
    }

    return false;
}

static void print_frame(const twai_message_t& msg) {
    Serial.print("CAN RX | ");

    if (msg.extd) {
        Serial.printf("EXT ID: 0x%08lX ", msg.identifier);
    } else {
        Serial.printf("STD ID: 0x%03lX ", msg.identifier);
    }

    Serial.printf("| DLC: %u | DATA: ", msg.data_length_code);

    for (int i = 0; i < msg.data_length_code; i++) {
        Serial.printf("%02X ", msg.data[i]);
    }

    Serial.println();
}

static void wifi_task() {
    static bool started = false;
    static bool initialized = false;

    if (!started) {
        Serial.printf("Connecting to WiFi '%s'...\n", WIFI_SSID);
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        started = true;
    }

    if (WiFi.status() == WL_CONNECTED && !initialized) {
        Serial.printf("WiFi connected! IP=%s\n",
                      WiFi.localIP().toString().c_str());

        udp.begin(UDP_PORT);
        udp_ready = true;
        initialized = true;
    }
}

static void send_frame_udp(uint32_t can_id, uint8_t dlc, const uint8_t* data) {
    if (!udp_ready) return;

    udp_can_frame pkt = {};
    pkt.timestamp_ms = millis();
    pkt.can_id = can_id;
    pkt.dlc = dlc;

    for (uint8_t i = 0; i < dlc && i < 8; i++) {
        pkt.data[i] = data[i];
    }

    udp.beginPacket(LAPTOP_IP, UDP_PORT);
    udp.write((const uint8_t*)&pkt, sizeof(pkt));
    udp.endPacket();
}

static uint32_t make_can_id(uint8_t prio, uint8_t src, uint8_t dst, uint8_t cls) {
    return (((uint32_t)prio & 0x07U) << 26) |
           (((uint32_t)src  & 0xFFU) << 14) |
           (((uint32_t)dst  & 0xFFU) << 6)  |
           ((uint32_t)cls   & 0x3FU);
}

static void send_bridge_heartbeat() {
    twai_message_t hb = {};
    hb.extd = 1;
    hb.identifier = make_can_id(PRIO_LOW, COMMS_ID, CAN_BROADCAST, CLS_HEARTBEAT);
    hb.data_length_code = 8;
    hb.data[0] = COMMS_ID;
    hb.data[1] = OP_HEARTBEAT;
    hb.data[2] = 0;
    hb.data[3] = 0;
    hb.data[4] = 0;
    hb.data[5] = 0;
    hb.data[6] = 0;
    hb.data[7] = 0;

    const esp_err_t tx_ret = twai_transmit(&hb, pdMS_TO_TICKS(20));
    if (tx_ret != ESP_OK) {
        Serial.printf("CAN TX HEARTBEAT failed ret=%d\n", (int)tx_ret);
    }

    // Mirror heartbeat directly to UDP so the ground station can always show COMMS health.
    send_frame_udp(hb.identifier, hb.data_length_code, hb.data);
}

void setup() {
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    Serial.begin(115200);
    delay(1200); // Give host monitor time to attach after reset/flash.
    uint32_t serial_wait_start = millis();
    Serial.begin(115200);
    delay(1200);

    Serial.println("\n=== CAN MONITOR MODE ===");
    for (int i = 0; i < 6; i++) {
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        Serial.printf("BOOT alive %d\n", i);
        delay(120);
    }
    digitalWrite(STATUS_LED, LOW);

    if (!twai_init()) {
        Serial.println("TWAI INIT FAILED");
        // Keep running so serial output proves firmware is alive.
    }

    Serial.printf("TWAI READY (%s)\n", twai_mode_name());
    Serial.printf("TX TEST %s\n", ENABLE_TX_TEST ? "ENABLED" : "DISABLED");
}

void loop() {
    wifi_task();
    twai_message_t msg;
    static uint32_t last_status_ms = 0;
    static uint32_t last_tx_ms = 0;
    static uint32_t last_switch_ms = 0;
    static uint32_t last_alive_ms = 0;
    static uint32_t last_heartbeat_ms = 0;
    static int16_t current_rpm_ref = TX_RPM_REF_LOW;
    const uint32_t now = millis();

    (void)twai_ensure_running();

    if (now - last_heartbeat_ms >= HEARTBEAT_PERIOD_MS) {
        last_heartbeat_ms = now;
        send_bridge_heartbeat();
    }

    if (ENABLE_TX_TEST && (now - last_switch_ms >= TX_RPM_SWITCH_MS)) {
        last_switch_ms = now;
        current_rpm_ref = (current_rpm_ref == TX_RPM_REF_LOW) ? TX_RPM_REF_HIGH : TX_RPM_REF_LOW;
    }

    if (ENABLE_TX_TEST && (now - last_tx_ms >= 1000U)) {
        last_tx_ms = now;
        twai_message_t tx = {};
        tx.extd = 1;
        tx.identifier = 0x08013FC2;  // prio=2 src=ADCS(0x04) dst=MOTOR(0x05) cls=COMMAND(0x02)
        tx.data_length_code = 8;
        tx.data[0] = 0x04;  // src = ADCS
        tx.data[1] = 0x50;  // op = OP_SET_WHEEL_RPM
        tx.data[2] = TX_TARGET_MOTOR;
        tx.data[3] = (uint8_t)((current_rpm_ref >> 8) & 0xFF);
        tx.data[4] = (uint8_t)(current_rpm_ref & 0xFF);
        tx.data[5] = 0;
        tx.data[6] = 0;
        tx.data[7] = 0;
        const esp_err_t tx_ret = twai_transmit(&tx, pdMS_TO_TICKS(20));
        Serial.printf("CAN TX RPM | ret=%d | motor=%u | rpm=%d\n",
                      (int)tx_ret, (unsigned)TX_TARGET_MOTOR, (int)current_rpm_ref);
    }

    // Wait up to 100 ms for a frame so TX test and status logs stay responsive.
    if (twai_receive(&msg, pdMS_TO_TICKS(100)) == ESP_OK) {
       print_frame(msg);
       send_frame_udp(msg.identifier, msg.data_length_code, msg.data);
       digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
    } else if (now - last_status_ms >= 2000U) {
        last_status_ms = now;
        twai_status_info_t st = {};
        if (twai_get_status_info(&st) == ESP_OK) {
            Serial.printf("CAN idle... state=%s(%d) tx_failed=%lu rx_missed=%lu rx_overrun=%lu\n",
                          twai_state_name(st.state), (int)st.state,
                          (unsigned long)st.tx_failed_count,
                          (unsigned long)st.rx_missed_count,
                          (unsigned long)st.rx_overrun_count);
            if (st.state == TWAI_STATE_BUS_OFF) {
                if (twai_initiate_recovery() == ESP_OK) {
                    Serial.println("TWAI recovery initiated");
                } else {
                    Serial.println("TWAI recovery initiate failed");
                }
            } else if (st.state == TWAI_STATE_STOPPED) {
                if (twai_start() == ESP_OK) {
                    Serial.println("TWAI start requested from STOPPED");
                } else {
                    Serial.println("TWAI start request failed");
                }
            }
        } else {
            Serial.println("CAN idle... (status unavailable)");
        }
    }

    if (now - last_alive_ms >= 3000U) {
        last_alive_ms = now;
        Serial.println("Bridge alive");
    }
}
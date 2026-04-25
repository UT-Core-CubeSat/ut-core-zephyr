#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/twai.h"

// ---------- USER CONFIG ----------
static const char* WIFI_SSID     = "braydonxps";
static const char* WIFI_PASSWORD = "chimichanga";
static const IPAddress LAPTOP_IP(192, 168, 137, 1);
static const uint16_t UDP_PORT   = 5005;
static const gpio_num_t TWAI_TX_GPIO = GPIO_NUM_5;
static const gpio_num_t TWAI_RX_GPIO = GPIO_NUM_4;
static const int STATUS_LED = 21;
// ---------------------------------

// Keep RX stable in listen mode, briefly switch to NORMAL only to send commands.
static constexpr bool COMMAND_MODE_ENABLE = true;

// UT-CORE protocol fields expected by bldcHallL6234.
static constexpr uint8_t ADCS_ID = 0x04;
static constexpr uint8_t MOTOR_ID = 0x05;
static constexpr uint8_t CLS_COMMAND = 0x02;
static constexpr uint8_t OP_SET_WHEEL_RPM = 0x50;

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

static uint32_t make_can_id(uint8_t prio, uint8_t src, uint8_t dst, uint8_t cls) {
    return (((uint32_t)prio & 0x07U) << 26) |
           (((uint32_t)src  & 0xFFU) << 14) |
           (((uint32_t)dst  & 0xFFU) << 6)  |
           (((uint32_t)cls  & 0x3FU));
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
        Serial.printf("WiFi connected! IP=%s\n", WiFi.localIP().toString().c_str());
        udp.begin(UDP_PORT);
        udp_ready = true;
        initialized = true;
    }
}

static void send_frame(uint32_t can_id, uint8_t dlc, const uint8_t* data) {
    if (!udp_ready) return;
    udp_can_frame pkt = {};
    pkt.timestamp_ms = millis();
    pkt.can_id = can_id;
    pkt.dlc = dlc;
    for (uint8_t i = 0; i < dlc && i < 8; i++) pkt.data[i] = data[i];
    udp.beginPacket(LAPTOP_IP, UDP_PORT);
    udp.write((const uint8_t*)&pkt, sizeof(pkt));
    udp.endPacket();
}

static bool twai_start_with_mode(twai_mode_t mode, int rx_len, int tx_len) {
    twai_stop();
    twai_driver_uninstall();

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, mode);
    g.rx_queue_len = rx_len;
    g.tx_queue_len = tx_len;

    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
    if (twai_start() != ESP_OK) return false;
    return true;
}

static bool twai_init_listen_only() {
    return twai_start_with_mode(TWAI_MODE_LISTEN_ONLY, 64, 0);
}

static bool send_motor1_rpm_ref_once(int16_t rpm_ref) {
    if (!twai_start_with_mode(TWAI_MODE_NORMAL, 8, 8)) {
        Serial.println("TWAI switch->NORMAL failed");
        (void)twai_init_listen_only();
        return false;
    }

    twai_message_t tx = {};
    tx.extd = 1;
    tx.data_length_code = 8;
    tx.identifier = make_can_id(2, ADCS_ID, MOTOR_ID, CLS_COMMAND);
    tx.data[0] = ADCS_ID;
    tx.data[1] = OP_SET_WHEEL_RPM;
    tx.data[2] = 1;
    tx.data[3] = (uint8_t)((rpm_ref >> 8) & 0xFF);
    tx.data[4] = (uint8_t)(rpm_ref & 0xFF);
    tx.data[5] = 0;
    tx.data[6] = 0;
    tx.data[7] = 0;

    const esp_err_t ret = twai_transmit(&tx, pdMS_TO_TICKS(20));
    if (ret == ESP_OK) {
        send_frame(tx.identifier, tx.data_length_code, tx.data);
    } else {
        Serial.printf("TX FAIL err=%d\n", (int)ret);
    }

    if (!twai_init_listen_only()) {
        Serial.println("TWAI restore LISTEN_ONLY failed");
    }
    return ret == ESP_OK;
}

static void print_frame(const twai_message_t& msg) {
    Serial.print("CAN RX | ");
    if (msg.extd) Serial.printf("EXT ID: 0x%08lX ", msg.identifier);
    else Serial.printf("STD ID: 0x%03lX ", msg.identifier);
    Serial.printf("| DLC: %u | DATA: ", msg.data_length_code);
    for (int i = 0; i < msg.data_length_code; i++) Serial.printf("%02X ", msg.data[i]);
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("\n=== CAN + WIFI MODE (HYBRID CMD) ===");
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);
    if (!twai_init_listen_only()) {
        Serial.println("TWAI LISTEN_ONLY init failed");
        while (true) delay(1000);
    }
    Serial.println("TWAI READY (LISTEN_ONLY)");
}

void loop() {
    wifi_task();

    static uint32_t last_cmd_ms = 0;
    static bool high_ref = false;
    const uint32_t now = millis();
    if (COMMAND_MODE_ENABLE && (now - last_cmd_ms >= 1000U)) {
        last_cmd_ms = now;
        const int16_t rpm_cmd = high_ref ? 5000 : 1000;
        high_ref = !high_ref;
        const bool ok = send_motor1_rpm_ref_once(rpm_cmd);
        Serial.printf("TX motor1 rpm_ref=%d -> %s\n", rpm_cmd, ok ? "OK" : "FAIL");
    }

    // Bound RX work so heartbeat traffic does not starve periodic command TX.
    twai_message_t msg;
    int rx_processed = 0;
    while (rx_processed < 32 && twai_receive(&msg, 0) == ESP_OK) {
        print_frame(msg);
        send_frame(msg.identifier, msg.data_length_code, msg.data);
        digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        rx_processed++;
    }
}
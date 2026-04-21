// M5 Stamp S3 CAN-to-UDP bridge for UT-CORE.
// Listens on the CAN bus via the ESP32-S3 TWAI peripheral and forwards every
// received frame as a 21-byte UDP packet to the laptop ground station.
//
// Wiring: M5 Stamp S3 <-> SN65HVD230 (or equivalent 3.3V CAN transceiver) <-> CAN bus
//   TWAI_TX_GPIO -> transceiver TXD
//   TWAI_RX_GPIO -> transceiver RXD
//   3V3 / GND    -> transceiver VCC / GND
//   Transceiver CANH/CANL -> bus CANH/CANL

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/twai.h"

// ---------- USER CONFIG ----------
static const char*    WIFI_SSID     = "ARICKSLAPTOP";
static const char*    WIFI_PASSWORD = "CUBESATISCOOL";
static const IPAddress LAPTOP_IP(192, 168, 137, 1);  // typical Windows hotspot gateway
static const uint16_t  UDP_PORT     = 5005;

static const gpio_num_t TWAI_TX_GPIO = GPIO_NUM_5;  // to transceiver TXD
static const gpio_num_t TWAI_RX_GPIO = GPIO_NUM_4;  // to transceiver RXD

// Set to TWAI_MODE_NORMAL if this node also needs to transmit / ACK.
// LISTEN_ONLY is safer for sniffing: the M5 will not ACK frames.
static const twai_mode_t TWAI_MODE = TWAI_MODE_LISTEN_ONLY;

// Built-in status LED on most M5 Stamp S3 boards.
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

static void wifi_connect() {
    Serial.printf("Connecting to WiFi '%s'", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nConnected. IP=%s  -> sending to %s:%u\n",
                  WiFi.localIP().toString().c_str(),
                  LAPTOP_IP.toString().c_str(), UDP_PORT);
}

static bool twai_start_500k() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE);
    g.rx_queue_len = 32;
    g.tx_queue_len = 0;  // no TX needed in listen-only

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
    digitalWrite(STATUS_LED, LOW);

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
        // no frame this window; also opportunistic WiFi reconnect
        if (WiFi.status() != WL_CONNECTED) wifi_connect();
        return;
    }

    udp_can_frame pkt = {};
    pkt.timestamp_ms = (uint64_t)millis();
    pkt.can_id       = msg.identifier;  // extended 29-bit
    pkt.dlc          = msg.data_length_code;
    memcpy(pkt.data, msg.data, sizeof(pkt.data));

    udp.beginPacket(LAPTOP_IP, UDP_PORT);
    udp.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    udp.endPacket();

    digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
}

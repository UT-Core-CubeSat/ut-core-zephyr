#include <Arduino.h>
#include "driver/twai.h"

// ---------- CONFIG ----------
static const gpio_num_t TWAI_TX_GPIO = GPIO_NUM_8;
static const gpio_num_t TWAI_RX_GPIO = GPIO_NUM_4;
static const int STATUS_LED = 21;
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

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("\n=== CAN MONITOR MODE ===");

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    if (!twai_init()) {
        Serial.println("TWAI INIT FAILED");
        while (true) delay(1000);
    }

    Serial.printf("TWAI READY (%s)\n", twai_mode_name());
    Serial.printf("TX TEST %s\n", ENABLE_TX_TEST ? "ENABLED" : "DISABLED");
}

void loop() {
    twai_message_t msg;
    static uint32_t last_status_ms = 0;
    static uint32_t last_tx_ms = 0;
    static uint32_t last_switch_ms = 0;
    static int16_t current_rpm_ref = TX_RPM_REF_LOW;
    const uint32_t now = millis();

    (void)twai_ensure_running();

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

        // Blink LED on activity
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
            Serial.println("CAN idle...");
        }
    }
}
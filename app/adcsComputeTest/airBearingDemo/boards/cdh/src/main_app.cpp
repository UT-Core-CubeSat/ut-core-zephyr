#include <zephyr/kernel.h>

void periodic_soh() {
    int64_t now = k_uptime_get();

    if (now - last_soh_ < SOH_PERIOD_MS) 
        return;
    last_soh_ = now;


    internal_readings r{}; 
    (void)internal_sensors_read(&r);
    soh_packet p{}; 
    p.mcu_temp_c10 = r.mcu_temp_c10; 
    p.bus_v_mv     = r.vbus_mv; 
    p.mode         = (uint8_t)mode_;
    soh_update_comm_stats(&p, rx_count_, tx_count_);


    /* Transmit a compact SOH frame (example) to broadcast */
    can_bus_send_u8(can_, CAN_ID(2, CAN_BROADCAST, CLS_SOH),
    CDH_NODE_ID, OP_SOH_FRAME,
    (uint8_t)(p.bus_v_mv/20), /*coarse*/
    (uint8_t)(p.mcu_temp_c10/5),
    p.mode,
    (uint8_t)(p.rx_count & 0xFF), (uint8_t)(p.tx_count & 0xFF), 0);
}


/* === Link & experiment scheduling (stub hooks) === */
void periodic_schedule() {
    int64_t now = k_uptime_get();
    if (now - last_sched_ < SCHED_PERIOD_MS) return;
    last_sched_ = now;
    // TODO: compute next downlink/uplink window; compute next experiment pass time
}


/* === Mode management === */
void periodic_modes() {
    int64_t now = k_uptime_get();
    if (now - last_mode_ < MODE_PERIOD_MS) return;
    last_mode_ = now;


    /* Simple example: if bus voltage low, go low-power */
    // TODO: read real numbers; for now, keep OPERATIONAL
    mode_ = CDH_MODE_OPERATIONAL;
}


/* === Watchdog / health === */
void periodic_watchdog() {
    int64_t now = k_uptime_get();
    if (now - last_wd_ < WD_PERIOD_MS) return;
    last_wd_ = now;
    // TODO: feed hardware watchdog; monitor subsystem pings, mark faults
}


private:
const struct device *can_ = nullptr;
enum cdh_mode mode_ = CDH_MODE_STANDBY;
bool led_on_ = false;
uint16_t rx_count_ = 0; 
tx_count_ = 0; // hook into can callbacks to maintain
int64_t last_soh_{} , last_sched_{}, last_mode_{}, last_wd_{};
};


extern "C" int main(void) {
    CdhApp app;
    if (app.init() != 0) {
        LOG_ERR("CDH init failed");
        return 0;
    }
    LOG_INF("CDH app started");
    app.run();
    return 0;
}
#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

/*
 * Motor-board CAN layer (FDCAN1, 500 kbit/s, 29-bit IDs).
 *
 * RX: an internal thread drains wheel-cmd frames from ADCS and writes the
 *     4 per-wheel setpoints through motor_can_apply_rpm_setpoint() — a
 *     callback supplied by main.c so this TU does not know about rpm_ref_cmd.
 *     The timestamp of the most recent frame is tracked so the control
 *     thread can enforce a failsafe on RX timeout.
 *
 * TX: a periodic thread pulls measured RPM via the supplied getter and
 *     sends wheel-telemetry frames to ADCS at WHEEL_TX_PERIOD_MS.
 *
 * Units: signed mechanical RPM at the wheel rotor.
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*motor_can_apply_fn)(unsigned wheel_idx, float rpm_signed);
typedef float (*motor_can_measure_fn)(unsigned wheel_idx);

struct motor_can_cfg {
	motor_can_apply_fn   apply_setpoint; /* called from RX thread, per wheel */
	motor_can_measure_fn measure_rpm;    /* called from TX thread, per wheel */
};

int  motor_can_init(const struct motor_can_cfg *cfg);

/*
 * Milliseconds since the last received wheel-cmd frame. Returns INT64_MAX
 * if no frame has ever been received.
 */
int64_t motor_can_rx_age_ms(void);

uint32_t motor_can_rx_count(void);
uint32_t motor_can_tx_count(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CAN_H */

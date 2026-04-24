#pragma once
/*
 * UT-CORE Reaction-Wheel CAN Protocol
 *
 * Extends common/can_proto.h (29-bit PRIO|SRC|DST|CLASS layout, 500 kbit/s).
 *
 * ADCS (SRC=ADCS_ID)  --CLS_WHEEL_CMD-->  MOTOR (DST=MOTOR_ID)    @ 40 Hz
 * MOTOR (SRC=MOTOR_ID) --CLS_WHEEL_TLM-->  ADCS  (DST=ADCS_ID)    @ 20 Hz
 *
 * Frame payload is 8 bytes — four little-endian int16 RPM values, one per
 * wheel. Signed: +rpm = forward, -rpm = reverse. No src/op wrapper; all 8
 * bytes carry data (which is why these classes bypass can_fill_payload).
 *
 * Units: mechanical RPM at the wheel rotor.
 * Range: saturated to [-WHEEL_RPM_LIMIT, +WHEEL_RPM_LIMIT] by sender.
 *
 * Bus hazard note: on the ADCS and motor-board overlays, PA10 (canen) and
 * PA9 (canstb) are repurposed by the apps (LED / motor phase-B enable).
 * The transceiver IC on those boards must therefore be always-on by
 * hardware default — the CAN driver does not drive those pins.
 */

#include "can_proto.h"
#include <stdint.h>

#define CLS_WHEEL_CMD  0x20U  /* ADCS -> MOTOR: 8B RPM setpoint  */
#define CLS_WHEEL_TLM  0x21U  /* MOTOR -> ADCS: 8B measured RPM  */

#define WHEEL_RPM_LIMIT   10000   /* hard cap for both sides, mech RPM */
#define WHEEL_PRIO_CMD    2       /* setpoint: elevated; loss causes failsafe */
#define WHEEL_PRIO_TLM    4       /* telemetry: nominal                       */

#define WHEEL_COUNT 4

struct __attribute__((packed)) wheel_rpm_frame {
	int16_t rpm[WHEEL_COUNT];
};

/* CAN IDs with SRC baked in so RX filters can match SRC+CLASS together. */
#define WHEEL_CMD_ID \
	CAN_ID_FULL(WHEEL_PRIO_CMD, ADCS_ID,  MOTOR_ID, CLS_WHEEL_CMD)

#define WHEEL_TLM_ID \
	CAN_ID_FULL(WHEEL_PRIO_TLM, MOTOR_ID, ADCS_ID,  CLS_WHEEL_TLM)

/* Full-ID mask: PRIO | SRC | DST | CLASS (all 29 significant bits). */
#define WHEEL_ID_MASK_29 \
	(CAN_PRIO(0x07U) | CAN_SRC(0xFFU) | CAN_DST(0xFFU) | CAN_CLASS(0x3FU))

static inline int16_t wheel_rpm_saturate_i16(float rpm)
{
	if (rpm >  (float)WHEEL_RPM_LIMIT) return  (int16_t)WHEEL_RPM_LIMIT;
	if (rpm < -(float)WHEEL_RPM_LIMIT) return (int16_t)(-WHEEL_RPM_LIMIT);
	return (int16_t)(rpm >= 0.f ? (rpm + 0.5f) : (rpm - 0.5f));
}

static inline void wheel_rpm_pack(const float rpm_f[WHEEL_COUNT],
				  struct wheel_rpm_frame *f)
{
	for (int i = 0; i < WHEEL_COUNT; i++) {
		f->rpm[i] = wheel_rpm_saturate_i16(rpm_f[i]);
	}
}

static inline void wheel_rpm_unpack(const struct wheel_rpm_frame *f,
				    float rpm_f[WHEEL_COUNT])
{
	for (int i = 0; i < WHEEL_COUNT; i++) {
		rpm_f[i] = (float)f->rpm[i];
	}
}

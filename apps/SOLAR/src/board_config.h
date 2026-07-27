/*
 * board_config.h — Solar Conglomerate (Magnetorquer Controller) board defines
 *
 * Pin assignments:
 *   PC2  (pin  8) — MTQ channel 0 (PWM or GPIO)
 *   PC3  (pin  9) — MTQ channel 1
 *   PC4  (pin 10) — MTQ channel 2
 *   PC5  (pin 11) — MTQ channel 3
 *
 *   PA9  — TCAN3403 SILENT (active high = silent mode)
 *   PA10 — TCAN3403 SHDN   (active high = shutdown)
 *   PA11 — FDCAN1 RX
 *   PA12 — FDCAN1 TX
 *
 *   PA6  — MCU role select strap
 *   PA2  — Hardware watchdog kick (active edge TBD)
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* ================= NODE IDENTITY ================= */
/*
 * CAN bus node ID for the Solar Conglomerate board.
 * Must match the value used in can_proto.h / the CDH node table.
 */
#define BOARD_NODE_ID       0x08   /* SOLAR_ID */
#define BOARD_NODE_NAME     "SOLAR"

/* ================= CAN TRANSCEIVER PINS ================= */
#define PIN_CAN_SILENT      9      /* PA9  — TCAN3403 silent mode        */
#define PIN_CAN_SHDN        10     /* PA10 — TCAN3403 shutdown           */

/* ================= MAGNETORQUER PIN MAP ================= */
/*
 * PC2-PC5 drive the magnetorquer coils.  When PWM timer channels are
 * available these pins are muxed to the timer; otherwise they are used
 * as plain GPIO outputs for on/off control.
 *
 * Channel index | GPIO port | Pin | Package pin | Timer AF (if PWM)
 * --------------|-----------|-----|-------------|-------------------
 *       0       |   GPIOC   |  2  |      8      | TIM3_CH3 (AF2)  *
 *       1       |   GPIOC   |  3  |      9      | TIM3_CH4 (AF2)  *
 *       2       |   GPIOC   |  4  |     10      | — (GPIO only)   *
 *       3       |   GPIOC   |  5  |     11      | — (GPIO only)   *
 *
 * (*) TIM3_CH3/CH4 are available on PC2/PC3 via AF2 on STM32U5A5.
 *     PC4/PC5 do not have a general-purpose timer AF, so they are
 *     always GPIO-driven.  If true PWM is needed on all four channels,
 *     consider re-routing to pins with TIM4/TIM5 AF support.
 *
 * The DTS overlay enables the PWM channels that are available;
 * main.c auto-detects PWM vs GPIO at compile time.
 */
#define MTQ0_PORT       gpioc
#define MTQ0_PIN        2
#define MTQ1_PORT       gpioc
#define MTQ1_PIN        3
#define MTQ2_PORT       gpioc
#define MTQ2_PIN        4
#define MTQ3_PORT       gpioc
#define MTQ3_PIN        5

/* Package pin numbers (for schematic cross-reference only) */
#define MTQ0_PKG_PIN    8
#define MTQ1_PKG_PIN    9
#define MTQ2_PKG_PIN    10
#define MTQ3_PKG_PIN    11

/* ================= WATCHDOG ================= */
#define WATCHDOG_GPIO_PORT  gpioa
#define WATCHDOG_GPIO_PIN   2      /* PA2 — external WD IC kick */

/* ================= MCU SELECT ================= */
#define MCU_SELECT_PORT     gpioa
#define MCU_SELECT_PIN      6      /* PA6 — role strap */

/* ================= PWM PARAMETERS ================= */
/*
 * Default PWM frequency for magnetorquer drive.
 * 20 kHz is above audible range and keeps core losses low in the coils.
 * Period = 1 / 20 kHz = 50 µs = 50000 ns.
 *
 * This value is set in the DTS overlay (pwms property period field).
 * Defined here for reference and for any runtime reconfiguration logic.
 */
#define MTQ_PWM_FREQ_HZ         20000
#define MTQ_PWM_PERIOD_NS       (1000000000UL / MTQ_PWM_FREQ_HZ)  /* 50000 ns */

/* ================= SAFETY LIMITS ================= */
/*
 * Maximum single actuation duration.  Protects against a stuck-on
 * coil draining the battery if the timer command is corrupt.
 */
#define MTQ_MAX_ON_TIME_MS      60000   /* 60 seconds */

/*
 * Maximum allowed duty cycle.  Depending on coil thermal design
 * this may need to be < 100% to prevent overheating.
 * Set to 100 during initial testing; tighten after thermal characterisation.
 */
#define MTQ_MAX_DUTY_PCT        100

#endif /* BOARD_CONFIG_H */
#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>

#include "motor_can.h"
#include "can_wheel.h"

#define USER_NODE DT_PATH(zephyr_user)

/* Setpoint is zeroed if no ADCS frame arrives within this many ms. */
#define CAN_RX_WATCHDOG_MS 200

/*
 * Board overlay (e.g. ut_core.overlay) defines M1..M4 on zephyr,user.
 * Flip motor_enable[] to test one motor; all four stay in the build.
 */
#define NMOTORS 4U
#define M3_IDX  2U
#define M4_IDX  3U

/* Per-motor: false = PWM/enables off (bench one motor at a time). */
static const bool motor_enable[] = {
	true,  /* M1 — PI speed control */
	true,  /* M2 — PI speed control */
	true,  /* M3 — PI speed control */
	true,  /* M4 — PI speed control */
};

BUILD_ASSERT(ARRAY_SIZE(motor_enable) == NMOTORS, "motor_enable vs NMOTORS");

static inline bool motor_is_active(unsigned mi)
{
	return (mi < NMOTORS) && motor_enable[mi];
}

#define PWM_PERIOD_NS PWM_HZ(20000)

/* 8 magnetic poles → 4 pole pairs: mech_rpm = (ω_elec / 2π) * 60 / pole_pairs */
#define M1_MAGNETIC_POLES 8U
#define POLE_PAIRS (M1_MAGNETIC_POLES / 2U)

BUILD_ASSERT((M1_MAGNETIC_POLES % 2U) == 0U && M1_MAGNETIC_POLES >= 2U,
	     "M1_MAGNETIC_POLES must be an even pole count");

/*
 * If hall-derived RPM reads backward vs how you define “forward”, flip this
 * between -1.f and +1.f.
 */
#define M1_RPM_MEAS_SIGN (+1.f)

/* ADCS wheel envelope (rpm, mechanical) — clamps telemetry and dt_max. */
#define WHEEL_RPM_ABS_MAX 10000

#define M1_DUTY_MAX_PCT 95U
#define M2_DUTY_MAX_PCT 95U

/* M1 speed-loop setup: PI closes RPM with duty clamped to M1_DUTY_MAX_PCT. */
/*
 * Reference shape (pick one):
 *   M1_REF_MODE_TRIANGLE — ramp MIN→MAX→MIN forever (M1_REF_PROFILE_PERIOD_MS).
 *   M1_REF_MODE_STEP     — repeating square: MIN for M1_STEP_LOW_HOLD_MS, then
 *                          MAX for M1_STEP_HIGH_HOLD_MS, then repeat (triangle
 *                          code stays in this file).
 *   M1_REF_MODE_SINE     — sinusoid over REF_PROFILE_PERIOD_MS with optional
 *                          phase offset in degrees.
 */
#define M1_REF_MODE_TRIANGLE 0
#define M1_REF_MODE_STEP     1
#define M1_REF_MODE_SINE     2
#define M1_REF_MODE          M1_REF_MODE_SINE

#define M1_REF_RPM_MIN            -10000.f
#define M1_REF_RPM_MAX            10000.f
#define M1_REF_PROFILE_PERIOD_MS  20000
#define M1_REF_PHASE_DEG          0.f
/* Step / square-wave mode: plateau durations (ms) at each command level. */
#define M1_STEP_LOW_HOLD_MS   8000U
#define M1_STEP_HIGH_HOLD_MS  8000U
#define M1_SPEED_KP_DUTY_PCT  0.002f
#define M1_SPEED_KI_DUTY_PCT  0.020f
#define M1_SPEED_CTRL_MS      5U
#define M1_SPEED_MEAS_TAU_MS  25.f
#define M1_SPEED_RPM_PER_DUTY_PCT 125.876603f
#define M1_OPEN_LOOP_REVERSE  0 /* 1 = reverse commutation table */

#define M2_RPM_MEAS_SIGN (+1.f)
#define M2_REF_MODE          M1_REF_MODE_SINE
#define M2_REF_RPM_MIN            -10000.f
#define M2_REF_RPM_MAX            10000.f
#define M2_REF_PROFILE_PERIOD_MS  20000
#define M2_REF_PHASE_DEG          33.f
#define M2_STEP_LOW_HOLD_MS   8000U
#define M2_STEP_HIGH_HOLD_MS  8000U
#define M2_SPEED_KP_DUTY_PCT  0.002f
#define M2_SPEED_KI_DUTY_PCT  0.020f
#define M2_SPEED_CTRL_MS      5U
#define M2_SPEED_MEAS_TAU_MS  25.f
#define M2_SPEED_RPM_PER_DUTY_PCT 125.416687f
#define M2_OPEN_LOOP_REVERSE  0 /* 1 = reverse commutation table */

#define M3_DUTY_MAX_PCT 95U
#define M3_RPM_MEAS_SIGN (+1.f)
#define M3_REF_MODE          M1_REF_MODE_SINE
#define M3_REF_RPM_MIN            -10000.f
#define M3_REF_RPM_MAX            10000.f
#define M3_REF_PROFILE_PERIOD_MS  20000
#define M3_REF_PHASE_DEG          66.f
#define M3_STEP_LOW_HOLD_MS   8000U
#define M3_STEP_HIGH_HOLD_MS  8000U
#define M3_SPEED_KP_DUTY_PCT  0.002f
#define M3_SPEED_KI_DUTY_PCT  0.020f
#define M3_SPEED_CTRL_MS      5U
#define M3_SPEED_MEAS_TAU_MS  25.f
#define M3_SPEED_RPM_PER_DUTY_PCT 125.416687f
#define M3_OPEN_LOOP_REVERSE  0 /* 1 = reverse commutation table */

#define M4_DUTY_MAX_PCT 95U
#define M4_RPM_MEAS_SIGN (+1.f)
#define M4_REF_MODE          M1_REF_MODE_SINE
#define M4_REF_RPM_MIN            -10000.f
#define M4_REF_RPM_MAX            10000.f
#define M4_REF_PROFILE_PERIOD_MS  20000
#define M4_REF_PHASE_DEG          99.f
#define M4_STEP_LOW_HOLD_MS   8000U
#define M4_STEP_HIGH_HOLD_MS  8000U
#define M4_SPEED_KP_DUTY_PCT  0.002f
#define M4_SPEED_KI_DUTY_PCT  0.020f
#define M4_SPEED_CTRL_MS      5U
#define M4_SPEED_MEAS_TAU_MS  25.f
#define M4_SPEED_RPM_PER_DUTY_PCT 125.416687f
#define M4_OPEN_LOOP_REVERSE  0 /* 1 = reverse commutation table */

/* Drop duty to zero until measured speed is small before flipping direction. */
#define MOTOR_DIR_SWITCH_RPM 200.f

/* Hall sequence for +rpm (tune order if your wiring differs). */
static const uint8_t hall_ring_fwd[6] = {1, 5, 4, 6, 2, 3};

#define TELEMETRY_MS 500U
#define MOTOR_CTRL_BASE_MS 1U

#define MOTOR_CTRL_THREAD_PRIO 1
#define TELEMETRY_THREAD_PRIO 5
#define APP_THREAD_STACK_SIZE 2048

/* Consecutive invalid hall samples (0/7) before latching m_run=false. */
#define HALL_INVALID_DEBOUNCE 8U

/* Motor 1 — TIM4 CH1..3 → phases A,B,C */
#define M1_CH_A 1U
#define M1_CH_B 2U
#define M1_CH_C 3U

/* Motor 2 — TIM3: A CH1, B CH4, C CH3 */
#define M2_CH_A 1U
#define M2_CH_B 4U
#define M2_CH_C 3U

/* Motor 3 — TIM8: A CH3, B CH2, C CH1 */
#define M3_CH_A 3U
#define M3_CH_B 2U
#define M3_CH_C 1U

/* Motor 4 — TIM1: A CH3, B CH4, C CH2 */
#define M4_CH_A 3U
#define M4_CH_B 4U
#define M4_CH_C 2U

static const struct gpio_dt_spec hall_a   = GPIO_DT_SPEC_GET(USER_NODE, hall_a_gpios);
static const struct gpio_dt_spec hall_b   = GPIO_DT_SPEC_GET(USER_NODE, hall_b_gpios);
static const struct gpio_dt_spec hall_c   = GPIO_DT_SPEC_GET(USER_NODE, hall_c_gpios);
static const struct gpio_dt_spec fault_in = GPIO_DT_SPEC_GET(USER_NODE, fault_gpios);
static const struct gpio_dt_spec kill_in  = GPIO_DT_SPEC_GET(USER_NODE, kill_gpios);
static const struct gpio_dt_spec en_a     = GPIO_DT_SPEC_GET(USER_NODE, en_a_gpios);
static const struct gpio_dt_spec en_b     = GPIO_DT_SPEC_GET(USER_NODE, en_b_gpios);
static const struct gpio_dt_spec en_c     = GPIO_DT_SPEC_GET(USER_NODE, en_c_gpios);

static const struct gpio_dt_spec m2_hall_a = GPIO_DT_SPEC_GET(USER_NODE, m2_hall_a_gpios);
static const struct gpio_dt_spec m2_hall_b = GPIO_DT_SPEC_GET(USER_NODE, m2_hall_b_gpios);
static const struct gpio_dt_spec m2_hall_c = GPIO_DT_SPEC_GET(USER_NODE, m2_hall_c_gpios);
static const struct gpio_dt_spec m2_en_a   = GPIO_DT_SPEC_GET(USER_NODE, m2_en_a_gpios);
static const struct gpio_dt_spec m2_en_b   = GPIO_DT_SPEC_GET(USER_NODE, m2_en_b_gpios);
static const struct gpio_dt_spec m2_en_c   = GPIO_DT_SPEC_GET(USER_NODE, m2_en_c_gpios);

static const struct gpio_dt_spec m3_hall_a = GPIO_DT_SPEC_GET(USER_NODE, m3_hall_a_gpios);
static const struct gpio_dt_spec m3_hall_b = GPIO_DT_SPEC_GET(USER_NODE, m3_hall_b_gpios);
static const struct gpio_dt_spec m3_hall_c = GPIO_DT_SPEC_GET(USER_NODE, m3_hall_c_gpios);
static const struct gpio_dt_spec m3_en_a   = GPIO_DT_SPEC_GET(USER_NODE, m3_en_a_gpios);
static const struct gpio_dt_spec m3_en_b   = GPIO_DT_SPEC_GET(USER_NODE, m3_en_b_gpios);
static const struct gpio_dt_spec m3_en_c   = GPIO_DT_SPEC_GET(USER_NODE, m3_en_c_gpios);

static const struct gpio_dt_spec m4_hall_a = GPIO_DT_SPEC_GET(USER_NODE, m4_hall_a_gpios);
static const struct gpio_dt_spec m4_hall_b = GPIO_DT_SPEC_GET(USER_NODE, m4_hall_b_gpios);
static const struct gpio_dt_spec m4_hall_c = GPIO_DT_SPEC_GET(USER_NODE, m4_hall_c_gpios);
static const struct gpio_dt_spec m4_en_a   = GPIO_DT_SPEC_GET(USER_NODE, m4_en_a_gpios);
static const struct gpio_dt_spec m4_en_b   = GPIO_DT_SPEC_GET(USER_NODE, m4_en_b_gpios);
static const struct gpio_dt_spec m4_en_c   = GPIO_DT_SPEC_GET(USER_NODE, m4_en_c_gpios);

typedef struct {
	const struct device *pwm;
	uint32_t tim_ch[3];
	const struct gpio_dt_spec *hall[3];
	const struct gpio_dt_spec *en[3];
	bool halls_on_gpiod;
} motor_desc_t;

static const uint32_t m1_ch[3] = { M1_CH_A, M1_CH_B, M1_CH_C };
static const struct gpio_dt_spec *const m1_hall[3] = { &hall_a, &hall_b, &hall_c };
static const struct gpio_dt_spec *const m1_en[3]  = { &en_a, &en_b, &en_c };

static const uint32_t m2_ch[3] = { M2_CH_A, M2_CH_B, M2_CH_C };
static const struct gpio_dt_spec *const m2_hall[3] = { &m2_hall_a, &m2_hall_b, &m2_hall_c };
static const struct gpio_dt_spec *const m2_en[3]  = { &m2_en_a, &m2_en_b, &m2_en_c };

static const uint32_t m3_ch[3] = { M3_CH_A, M3_CH_B, M3_CH_C };
static const struct gpio_dt_spec *const m3_hall[3] = { &m3_hall_a, &m3_hall_b, &m3_hall_c };
static const struct gpio_dt_spec *const m3_en[3]  = { &m3_en_a, &m3_en_b, &m3_en_c };

static const uint32_t m4_ch[3] = { M4_CH_A, M4_CH_B, M4_CH_C };
static const struct gpio_dt_spec *const m4_hall[3] = { &m4_hall_a, &m4_hall_b, &m4_hall_c };
static const struct gpio_dt_spec *const m4_en[3]  = { &m4_en_a, &m4_en_b, &m4_en_c };

static const motor_desc_t motor_desc[] = {
	{
		.pwm = DEVICE_DT_GET(DT_NODELABEL(pwm4)),
		.tim_ch = { m1_ch[0], m1_ch[1], m1_ch[2] },
		.hall = { m1_hall[0], m1_hall[1], m1_hall[2] },
		.en = { m1_en[0], m1_en[1], m1_en[2] },
		.halls_on_gpiod = true,
	},
	{
		.pwm = DEVICE_DT_GET(DT_NODELABEL(pwm3)),
		.tim_ch = { m2_ch[0], m2_ch[1], m2_ch[2] },
		.hall = { m2_hall[0], m2_hall[1], m2_hall[2] },
		.en = { m2_en[0], m2_en[1], m2_en[2] },
		.halls_on_gpiod = true,
	},
	{
		.pwm = DEVICE_DT_GET(DT_NODELABEL(pwm8)),
		.tim_ch = { m3_ch[0], m3_ch[1], m3_ch[2] },
		.hall = { m3_hall[0], m3_hall[1], m3_hall[2] },
		.en = { m3_en[0], m3_en[1], m3_en[2] },
		.halls_on_gpiod = true,
	},
	{
		.pwm = DEVICE_DT_GET(DT_NODELABEL(pwm1)),
		.tim_ch = { m4_ch[0], m4_ch[1], m4_ch[2] },
		.hall = { m4_hall[0], m4_hall[1], m4_hall[2] },
		.en = { m4_en[0], m4_en[1], m4_en[2] },
		.halls_on_gpiod = false,
	},
};

BUILD_ASSERT(ARRAY_SIZE(motor_desc) == NMOTORS, "motor_desc count");

struct comm_step {
	int8_t hi;
	int8_t lo;
};

static const struct comm_step fwd_table[8] = {
	[0] = {-1, -1},
	[1] = {0, 2},
	[2] = {1, 0},
	[3] = {1, 2},
	[4] = {2, 1},
	[5] = {0, 1},
	[6] = {2, 0},
	[7] = {-1, -1},
};

/* Reverse = swap hi/lo vs forward for each valid hall. */
static const struct comm_step rev_table[8] = {
	[0] = {-1, -1},
	[1] = {2, 0},
	[2] = {0, 1},
	[3] = {2, 1},
	[4] = {1, 2},
	[5] = {1, 0},
	[6] = {0, 2},
	[7] = {-1, -1},
};

/* 0..100 % of PWM period (clamp active phase to M1_DUTY_MAX_PCT on M1 path). */
static volatile float duty_pct[NMOTORS];
static volatile uint8_t  m_hall[NMOTORS];
static volatile uint32_t m_trans[NMOTORS];
/* True while hall code is 1..6; false on 0/7 — that motor is not driven. */
static volatile bool m_run[NMOTORS];
static volatile uint8_t hall_inv_streak[NMOTORS];

static volatile bool m_use_rev[NMOTORS];
/* Hall edge timing for M1 (ISR writes, control loop reads). */
static uint32_t m1_cyc_hz;
static uint32_t m1_hall_dt_min_cyc;
static volatile uint32_t m1_prev_edge_cyc;
static volatile bool m1_have_prev_edge;
static volatile uint32_t m1_last_period_cyc;
static volatile int8_t m1_last_period_sign;
static volatile uint32_t m1_last_edge_cyc;

static uint32_t speed_trans_prev_telem[NMOTORS];
static uint32_t speed_trans_prev_ctrl[NMOTORS];
static float speed_pi_integral_pct[NMOTORS];
static float speed_rpm_ctrl_filt[NMOTORS];
static bool speed_rpm_ctrl_filt_valid[NMOTORS];
static volatile float rpm_ref_cmd[NMOTORS];
static atomic_t app_run = ATOMIC_INIT(1);

static struct gpio_callback hall_cb_gpiod;
static struct gpio_callback hall_cb_gpioe;
static struct k_thread motor_ctrl_thread_data;
static struct k_thread telemetry_thread_data;
K_THREAD_STACK_DEFINE(motor_ctrl_stack, APP_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(telemetry_stack, APP_THREAD_STACK_SIZE);

static bool motor_has_speed_ctrl(unsigned mi)
{
	return mi == 0U || mi == 1U || mi == M3_IDX || mi == M4_IDX;
}

static float motor_duty_max_pct(unsigned mi)
{
	switch (mi) {
	case 0U:
		return (float)M1_DUTY_MAX_PCT;
	case 1U:
		return (float)M2_DUTY_MAX_PCT;
	case M3_IDX:
		return (float)M3_DUTY_MAX_PCT;
	case M4_IDX:
		return (float)M4_DUTY_MAX_PCT;
	default:
		return 100.f;
	}
}

static float motor_rpm_meas_sign(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_RPM_MEAS_SIGN;
	case 1U:
		return M2_RPM_MEAS_SIGN;
	case M3_IDX:
		return M3_RPM_MEAS_SIGN;
	case M4_IDX:
		return M4_RPM_MEAS_SIGN;
	default:
		return 1.f;
	}
}

static uint32_t motor_speed_ctrl_ms(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_SPEED_CTRL_MS;
	case 1U:
		return M2_SPEED_CTRL_MS;
	case M3_IDX:
		return M3_SPEED_CTRL_MS;
	case M4_IDX:
		return M4_SPEED_CTRL_MS;
	default:
		return 0U;
	}
}

static float motor_speed_kp(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_SPEED_KP_DUTY_PCT;
	case 1U:
		return M2_SPEED_KP_DUTY_PCT;
	case M3_IDX:
		return M3_SPEED_KP_DUTY_PCT;
	case M4_IDX:
		return M4_SPEED_KP_DUTY_PCT;
	default:
		return 0.f;
	}
}

static float motor_speed_ki(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_SPEED_KI_DUTY_PCT;
	case 1U:
		return M2_SPEED_KI_DUTY_PCT;
	case M3_IDX:
		return M3_SPEED_KI_DUTY_PCT;
	case M4_IDX:
		return M4_SPEED_KI_DUTY_PCT;
	default:
		return 0.f;
	}
}

static float motor_speed_meas_tau_ms(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_SPEED_MEAS_TAU_MS;
	case 1U:
		return M2_SPEED_MEAS_TAU_MS;
	case M3_IDX:
		return M3_SPEED_MEAS_TAU_MS;
	case M4_IDX:
		return M4_SPEED_MEAS_TAU_MS;
	default:
		return 0.f;
	}
}

static float motor_speed_rpm_per_duty_pct(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_SPEED_RPM_PER_DUTY_PCT;
	case 1U:
		return M2_SPEED_RPM_PER_DUTY_PCT;
	case M3_IDX:
		return M3_SPEED_RPM_PER_DUTY_PCT;
	case M4_IDX:
		return M4_SPEED_RPM_PER_DUTY_PCT;
	default:
		return 0.f;
	}
}

static int motor_ref_mode(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_REF_MODE;
	case 1U:
		return M2_REF_MODE;
	case M3_IDX:
		return M3_REF_MODE;
	case M4_IDX:
		return M4_REF_MODE;
	default:
		return M1_REF_MODE_STEP;
	}
}

static float motor_ref_rpm_min(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_REF_RPM_MIN;
	case 1U:
		return M2_REF_RPM_MIN;
	case M3_IDX:
		return M3_REF_RPM_MIN;
	case M4_IDX:
		return M4_REF_RPM_MIN;
	default:
		return 0.f;
	}
}

static float motor_ref_rpm_max(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_REF_RPM_MAX;
	case 1U:
		return M2_REF_RPM_MAX;
	case M3_IDX:
		return M3_REF_RPM_MAX;
	case M4_IDX:
		return M4_REF_RPM_MAX;
	default:
		return 0.f;
	}
}

static uint32_t motor_ref_profile_period_ms(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_REF_PROFILE_PERIOD_MS;
	case 1U:
		return M2_REF_PROFILE_PERIOD_MS;
	case M3_IDX:
		return M3_REF_PROFILE_PERIOD_MS;
	case M4_IDX:
		return M4_REF_PROFILE_PERIOD_MS;
	default:
		return 0U;
	}
}

static float motor_ref_phase_deg(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_REF_PHASE_DEG;
	case 1U:
		return M2_REF_PHASE_DEG;
	case M3_IDX:
		return M3_REF_PHASE_DEG;
	case M4_IDX:
		return M4_REF_PHASE_DEG;
	default:
		return 0.f;
	}
}

static uint32_t motor_step_low_hold_ms(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_STEP_LOW_HOLD_MS;
	case 1U:
		return M2_STEP_LOW_HOLD_MS;
	case M3_IDX:
		return M3_STEP_LOW_HOLD_MS;
	case M4_IDX:
		return M4_STEP_LOW_HOLD_MS;
	default:
		return 0U;
	}
}

static uint32_t motor_step_high_hold_ms(unsigned mi)
{
	switch (mi) {
	case 0U:
		return M1_STEP_HIGH_HOLD_MS;
	case 1U:
		return M2_STEP_HIGH_HOLD_MS;
	case M3_IDX:
		return M3_STEP_HIGH_HOLD_MS;
	case M4_IDX:
		return M4_STEP_HIGH_HOLD_MS;
	default:
		return 0U;
	}
}

static void pwm_all_off(const struct device *pwm, const uint32_t tim_ch[3],
			const struct gpio_dt_spec *const pen[3])
{
	for (int i = 0; i < 3; i++) {
		pwm_set(pwm, tim_ch[i], PWM_PERIOD_NS, 0, PWM_POLARITY_NORMAL);
		gpio_pin_set_dt(pen[i], 0);
	}
}

static bool apply_sixstep(const struct device *pwm, const uint32_t tim_ch[3],
			  const struct gpio_dt_spec *const pen[3],
			  const struct comm_step *tbl, uint8_t hall, unsigned mi)
{
	const struct comm_step *s = &tbl[hall & 0x07];

	if (s->hi < 0) {
		pwm_all_off(pwm, tim_ch, pen);
		return true;
	}

	float d = duty_pct[mi];

	if (d < 0.f) {
		d = 0.f;
	}
	if (d > motor_duty_max_pct(mi)) {
		d = motor_duty_max_pct(mi);
	}

	float pulse_f = ((float)PWM_PERIOD_NS * d) / 100.0f;
	uint32_t pulse = (uint32_t)(pulse_f + 0.5f);

	if (pulse > PWM_PERIOD_NS) {
		pulse = PWM_PERIOD_NS;
	}

	for (int i = 0; i < 3; i++) {
		if (i == s->hi) {
			pwm_set(pwm, tim_ch[i], PWM_PERIOD_NS, pulse,
				PWM_POLARITY_NORMAL);
			gpio_pin_set_dt(pen[i], 1);
		} else if (i == s->lo) {
			pwm_set(pwm, tim_ch[i], PWM_PERIOD_NS, 0,
				PWM_POLARITY_NORMAL);
			gpio_pin_set_dt(pen[i], 1);
		} else {
			pwm_set(pwm, tim_ch[i], PWM_PERIOD_NS, 0,
				PWM_POLARITY_NORMAL);
			gpio_pin_set_dt(pen[i], 0);
		}
	}
	return false;
}

static uint8_t read_hall_code(const struct gpio_dt_spec *const h[3])
{
	int a = gpio_pin_get_dt(h[0]) > 0 ? 1 : 0;
	int b = gpio_pin_get_dt(h[1]) > 0 ? 1 : 0;
	int c = gpio_pin_get_dt(h[2]) > 0 ? 1 : 0;

	return (uint8_t)(a | (b << 1) | (c << 2));
}

static bool hall_code_valid(uint8_t h)
{
	return (h != 0U) && (h != 7U);
}

/* +1 one step forward in hall_ring_fwd, -1 backward, 0 ambiguous/jump. */
static int8_t hall_transition_dir(uint8_t from, uint8_t to)
{
	int from_idx = -1;
	int to_idx = -1;

	if (!hall_code_valid(from) || !hall_code_valid(to)) {
		return 0;
	}

	for (int i = 0; i < 6; i++) {
		if (hall_ring_fwd[i] == from) {
			from_idx = i;
		}
		if (hall_ring_fwd[i] == to) {
			to_idx = i;
		}
	}
	if (from_idx < 0 || to_idx < 0) {
		return 0;
	}

	int diff = (to_idx - from_idx + 6) % 6;

	if (diff == 1) {
		return 1;
	}
	if (diff == 5) {
		return -1;
	}
	return 0;
}

static void refresh_outputs(void)
{
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		const motor_desc_t *m = &motor_desc[mi];
		float d = duty_pct[mi];

		if (!motor_is_active(mi)) {
			pwm_all_off(m->pwm, m->tim_ch, m->en);
			continue;
		}

		uint8_t h_live = read_hall_code(m->hall);
		/*
		 * When running, prefer the last ISR-step hall if it still matches GPIO
		 * (avoids sampling dither vs. edge-locked commutation).
		 */
		uint8_t h = h_live;

		if (m_run[mi] && hall_code_valid(m_hall[mi]) && (h_live == m_hall[mi])) {
			h = m_hall[mi];
		}

		/* Live halls: do not re-apply stale m_hall while inputs read 0/7. */
		if (!hall_code_valid(h_live) || !m_run[mi]) {
			pwm_all_off(m->pwm, m->tim_ch, m->en);
			continue;
		}

		if (d <= 0.f) {
			pwm_all_off(m->pwm, m->tim_ch, m->en);
			continue;
		}

		const struct comm_step *tbl = m_use_rev[mi] ? rev_table : fwd_table;

		(void)apply_sixstep(m->pwm, m->tim_ch, m->en, tbl, h, mi);
	}
}

static void process_motor(unsigned mi)
{
	if (!motor_is_active(mi)) {
		return;
	}

	const motor_desc_t *m = &motor_desc[mi];
	uint8_t h = read_hall_code(m->hall);

	if (!hall_code_valid(h)) {
		pwm_all_off(m->pwm, m->tim_ch, m->en);
		if (m_run[mi]) {
			uint8_t st = hall_inv_streak[mi] + 1U;

			if (st >= HALL_INVALID_DEBOUNCE) {
				m_run[mi] = false;
				if (mi == 0U) {
					m1_have_prev_edge = false;
				}
				m_hall[mi] = h;
				hall_inv_streak[mi] = 0U;
			} else {
				hall_inv_streak[mi] = st;
			}
		} else {
			m_hall[mi] = h;
			hall_inv_streak[mi] = 0U;
		}
		return;
	}

	hall_inv_streak[mi] = 0U;

	if ((h == m_hall[mi]) && m_run[mi]) {
		return;
	}

	if (mi == 0U && m_run[mi] && hall_code_valid(m_hall[mi])) {
		int8_t dir = hall_transition_dir(m_hall[mi], h);
		/*
		 * If hall_ring_fwd order does not match the sensor's Gray walk, dir==0
		 * on real steps — timing must still run or dt spans multiple 60° steps
		 * and edge-RPM reads ~1/P too low.
		 */
		int8_t dir_eff = dir;

		if (dir == 0 && hall_code_valid(h) && h != m_hall[mi]) {
			dir_eff = (m1_last_period_sign != 0) ? m1_last_period_sign : 1;
		}

		if (dir_eff != 0 && m1_cyc_hz > 0U) {
			uint32_t c = k_cycle_get_32();
			/*
			 * Three hall lines often produce multiple IRQs within tens of µs for
			 * one 60° step. dt_min must exceed that bunching but stay below one
			 * real step at WHEEL_RPM_ABS_MAX (computed at boot in main).
			 */
			uint32_t dt_min =
				m1_hall_dt_min_cyc > 0U ? m1_hall_dt_min_cyc : (m1_cyc_hz / 8000U);
			uint32_t dt_max = m1_cyc_hz; /* >~1 s between edges → ignore */

			if (m1_have_prev_edge) {
				uint32_t dt = c - m1_prev_edge_cyc;

				if (dt >= dt_min && dt <= dt_max) {
					m1_last_period_cyc = dt;
					m1_last_period_sign = dir_eff;
					m1_prev_edge_cyc = c;
				} else if (dt > dt_max) {
					/* Baseline too old (stall); restart from this edge. */
					m1_prev_edge_cyc = c;
				}
				/*
				 * If dt < dt_min: GPIO bunching on one 60° step — do NOT move
				 * m1_prev_edge_cyc, or dt spans multiple steps and edge-RPM lies.
				 */
			} else {
				m1_prev_edge_cyc = c;
				m1_have_prev_edge = true;
			}
			m1_last_edge_cyc = c;
		}
	}

	m_hall[mi] = h;
	m_run[mi] = true;
	m_trans[mi]++;

	if (duty_pct[mi] <= 0.f) {
		pwm_all_off(m->pwm, m->tim_ch, m->en);
	} else {
		const struct comm_step *tbl = m_use_rev[mi] ? rev_table : fwd_table;

		(void)apply_sixstep(m->pwm, m->tim_ch, m->en, tbl, h, mi);
	}
}

static void hall_isr_gpiod(const struct device *dev, struct gpio_callback *cb,
			   uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		if (!motor_is_active(mi)) {
			continue;
		}
		if (motor_desc[mi].halls_on_gpiod) {
			process_motor(mi);
		}
	}
}

static void hall_isr_gpioe(const struct device *dev, struct gpio_callback *cb,
			   uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		if (!motor_is_active(mi)) {
			continue;
		}
		if (!motor_desc[mi].halls_on_gpiod) {
			process_motor(mi);
		}
	}
}

/*
 * M3 halls PD8/PD9 and M4 halls PE8/PE9 share STM32 EXTI8 / EXTI9 mux (one port
 * per line). With both motors active, edge IRQs cannot be correct for both;
 * polling keeps commutation aligned with the rotor (no duty reduction).
 */
#ifndef HALL_M3M4_POLL_INTERVAL_US
#define HALL_M3M4_POLL_INTERVAL_US 80U
#endif

static void hall_m3m4_poll_work(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!motor_is_active(M3_IDX) || !motor_is_active(M4_IDX)) {
		return;
	}

	process_motor(M3_IDX);
	process_motor(M4_IDX);
}

static K_WORK_DEFINE(hall_m3m4_work, hall_m3m4_poll_work);

static void hall_m3m4_poll_timer_cb(struct k_timer *t)
{
	ARG_UNUSED(t);
	(void)k_work_submit(&hall_m3m4_work);
}

K_TIMER_DEFINE(hall_m3m4_timer, hall_m3m4_poll_timer_cb, NULL);

/* Mechanical rpm from last 60° hall period and time since last edge (no PLL). */
static float m1_rpm_from_hall_edges(void)
{
	if (!motor_is_active(0U) || !m_run[0] || m1_cyc_hz == 0U) {
		return 0.f;
	}

	const uint32_t c = k_cycle_get_32();
	const uint32_t age = c - m1_last_edge_cyc;
	const uint32_t dt = m1_last_period_cyc;
	uint32_t stale = m1_cyc_hz;

	if (dt > 0U && dt < m1_cyc_hz) {
		const uint32_t six = dt * 6U;

		if (six > stale) {
			stale = six;
		}
	}
	if (age > stale || dt == 0U || dt > m1_cyc_hz) {
		return 0.f;
	}

	const float sgn = (float)m1_last_period_sign;
	const float fclk = (float)m1_cyc_hz;
	const float P = (float)POLE_PAIRS;
	const float age_f = fmaxf((float)age, 1.f);
	const float dt_f = fmaxf((float)dt, 1.f);
	const float mag_dt = fclk * 10.f / (dt_f * P);
	const float mag_age = fclk * 10.f / (age_f * P);
	const float mag = fminf(mag_dt, mag_age);
	float rpm = M1_RPM_MEAS_SIGN * sgn * mag;

	if (rpm > (float)WHEEL_RPM_ABS_MAX) {
		rpm = (float)WHEEL_RPM_ABS_MAX;
	} else if (rpm < -(float)WHEEL_RPM_ABS_MAX) {
		rpm = -(float)WHEEL_RPM_ABS_MAX;
	}
	return rpm;
}

static float clampf(float x, float lo, float hi)
{
	if (x < lo) {
		return lo;
	}
	if (x > hi) {
		return hi;
	}
	return x;
}

static float motor_rpm_from_transition_window(unsigned mi, uint32_t trans_now,
					      uint32_t trans_prev, uint32_t dt_ms)
{
	if (dt_ms == 0U) {
		return 0.f;
	}

	const uint32_t dtrans = trans_now - trans_prev;
	const float mag =
		(float)dtrans * 1000.f * 10.f /
		((float)POLE_PAIRS * (float)dt_ms);
	float meas_sign = 0.f;
	const float cmd_sign = m_use_rev[mi] ? -1.f : 1.f;

	if (mi == 0U) {
		meas_sign =
			motor_rpm_meas_sign(mi) * (float)m1_last_period_sign;
	}

	return copysignf(mag, (meas_sign != 0.f) ? meas_sign : cmd_sign);
}

static float motor_speed_duty_ff_pct(unsigned mi, float rpm_ref)
{
	const float rpm_abs = fabsf(rpm_ref);
	const float rpm_per_duty = motor_speed_rpm_per_duty_pct(mi);

	if (rpm_per_duty <= 0.f) {
		return 0.f;
	}

	return clampf(rpm_abs / rpm_per_duty, 0.f, motor_duty_max_pct(mi));
}

static void motor_speed_ctrl_reset(unsigned mi)
{
	speed_pi_integral_pct[mi] = 0.f;
	speed_rpm_ctrl_filt[mi] = 0.f;
	speed_rpm_ctrl_filt_valid[mi] = false;
	speed_trans_prev_ctrl[mi] = m_trans[mi];
}

/*
 * Waveform reference generators — retained for bench bring-up when CAN is
 * unavailable. No longer wired into the runtime (ADCS CAN drives rpm_ref_cmd
 * now); the __attribute__((unused)) silences warnings. Safe to delete once
 * the CAN link is validated on hardware.
 */
__attribute__((unused))
static float motor_speed_ref_triangle(unsigned mi, int64_t t_ms)
{
	const float lo = fminf(motor_ref_rpm_min(mi), motor_ref_rpm_max(mi));
	const float hi = fmaxf(motor_ref_rpm_min(mi), motor_ref_rpm_max(mi));
	const int64_t p = (int64_t)motor_ref_profile_period_ms(mi);

	if (hi <= lo) {
		return lo;
	}
	if (p <= 0) {
		return hi;
	}

	int64_t ph = t_ms % p;

	if (ph < 0) {
		ph += p;
	}

	const int64_t half = p / 2;
	float u;

	if (half <= 0) {
		return hi;
	}

	if (ph < half) {
		u = (float)ph / (float)half;
	} else {
		u = (float)(p - ph) / (float)half;
	}

	return lo + (hi - lo) * u;
}

__attribute__((unused))
static float motor_speed_ref_step(unsigned mi, int64_t t_ms)
{
	const float lo = fminf(motor_ref_rpm_min(mi), motor_ref_rpm_max(mi));
	const float hi = fmaxf(motor_ref_rpm_min(mi), motor_ref_rpm_max(mi));
	const int64_t low_ms = (int64_t)motor_step_low_hold_ms(mi);
	const int64_t high_ms = (int64_t)motor_step_high_hold_ms(mi);
	const int64_t period = low_ms + high_ms;

	if (period <= 0) {
		return lo;
	}

	int64_t ph = t_ms % period;

	if (ph < 0) {
		ph += period;
	}

	if (ph < low_ms) {
		return lo;
	}

	return hi;
}

__attribute__((unused))
static float motor_speed_ref_sine(unsigned mi, int64_t t_ms)
{
	const float lo = fminf(motor_ref_rpm_min(mi), motor_ref_rpm_max(mi));
	const float hi = fmaxf(motor_ref_rpm_min(mi), motor_ref_rpm_max(mi));
	const int64_t p = (int64_t)motor_ref_profile_period_ms(mi);
	const float center = 0.5f * (lo + hi);
	const float amp = 0.5f * (hi - lo);
	const float two_pi = 6.28318530717958647692f;

	if (p <= 0 || amp <= 0.f) {
		return center;
	}

	int64_t ph = t_ms % p;

	if (ph < 0) {
		ph += p;
	}

	const float u = (float)ph / (float)p;
	const float phase_rad = motor_ref_phase_deg(mi) * (two_pi / 360.f);
	const float y = center + amp * sinf(two_pi * u + phase_rad);

	return clampf(y, lo, hi);
}

__attribute__((unused))
static float motor_speed_ref_at_uptime(unsigned mi, int64_t t_ms)
{
	if (motor_ref_mode(mi) == M1_REF_MODE_STEP) {
		return motor_speed_ref_step(mi, t_ms);
	}
	if (motor_ref_mode(mi) == M1_REF_MODE_SINE) {
		return motor_speed_ref_sine(mi, t_ms);
	}

	return motor_speed_ref_triangle(mi, t_ms);
}

static void motor_speed_pi_step(unsigned mi, uint32_t dt_ms, float rpm_ref_signed)
{
	if (!motor_is_active(mi) || !motor_has_speed_ctrl(mi) || dt_ms == 0U) {
		return;
	}

	const uint32_t trn = m_trans[mi];
	const float rpm_meas_raw =
		fabsf(motor_rpm_from_transition_window(mi, trn,
						      speed_trans_prev_ctrl[mi],
						      dt_ms));
	const float rpm_ref = fabsf(rpm_ref_signed);
	const bool rev_req = (rpm_ref_signed < 0.f);
	const float duty_ff = motor_speed_duty_ff_pct(mi, rpm_ref);
	const float dt_s = (float)dt_ms / 1000.f;
	const float alpha =
		clampf(dt_s / (dt_s + (motor_speed_meas_tau_ms(mi) / 1000.f)),
		       0.f, 1.f);

	if (!speed_rpm_ctrl_filt_valid[mi]) {
		speed_rpm_ctrl_filt[mi] = rpm_meas_raw;
		speed_rpm_ctrl_filt_valid[mi] = true;
	} else {
		speed_rpm_ctrl_filt[mi] +=
			alpha * (rpm_meas_raw - speed_rpm_ctrl_filt[mi]);
	}

	/*
	 * Reverse only after the rotor has slowed down enough that swapping the
	 * six-step table will not command an abrupt torque reversal.
	 */
	if (rev_req != m_use_rev[mi]) {
		if (speed_rpm_ctrl_filt[mi] > MOTOR_DIR_SWITCH_RPM) {
			duty_pct[mi] = 0.f;
			speed_pi_integral_pct[mi] = 0.f;
			refresh_outputs();
			speed_trans_prev_ctrl[mi] = trn;
			return;
		}

		m_use_rev[mi] = rev_req;
		motor_speed_ctrl_reset(mi);
		duty_pct[mi] = 0.f;
		refresh_outputs();
		return;
	}

	const float err = rpm_ref - speed_rpm_ctrl_filt[mi];
	const float integ_candidate =
		speed_pi_integral_pct[mi] + (motor_speed_ki(mi) * err * dt_s);
	float duty_unsat = duty_ff + (motor_speed_kp(mi) * err) + integ_candidate;
	float duty_cmd = clampf(duty_unsat, 0.f, motor_duty_max_pct(mi));

	/*
	 * Integrate unless that would push farther into saturation.
	 * This keeps the loop from winding up while duty is pinned.
	 */
	if ((duty_unsat == duty_cmd) ||
	    ((duty_unsat > motor_duty_max_pct(mi)) && (err < 0.f)) ||
	    ((duty_unsat < 0.f) && (err > 0.f))) {
		speed_pi_integral_pct[mi] = integ_candidate;
		duty_unsat =
			duty_ff + (motor_speed_kp(mi) * err) +
			speed_pi_integral_pct[mi];
		duty_cmd = clampf(duty_unsat, 0.f, motor_duty_max_pct(mi));
	}

	duty_pct[mi] = duty_cmd;
	speed_trans_prev_ctrl[mi] = trn;
	refresh_outputs();
}

static void all_motors_off(void)
{
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		const motor_desc_t *m = &motor_desc[mi];

		pwm_all_off(m->pwm, m->tim_ch, m->en);
	}
}

/*
 * Called from motor_can_rx thread once per received wheel-cmd frame, per wheel.
 * Writes straight into the existing volatile setpoint array the PI loop
 * already reads — no extra synchronization needed for scalar per-wheel floats.
 */
static void can_apply_setpoint_cb(unsigned mi, float rpm_signed)
{
	if (mi >= NMOTORS) {
		return;
	}
	if (!motor_is_active(mi) || !motor_has_speed_ctrl(mi)) {
		return;
	}

	float r = rpm_signed;
	if (r >  (float)WHEEL_RPM_LIMIT) r =  (float)WHEEL_RPM_LIMIT;
	if (r < -(float)WHEEL_RPM_LIMIT) r = -(float)WHEEL_RPM_LIMIT;
	rpm_ref_cmd[mi] = r;
}

/*
 * Called from motor_can_tx thread to assemble the outgoing telemetry frame.
 * Returns measured RPM with sign applied from the six-step direction table,
 * using the control-loop-filtered value for stability.
 */
static float can_measure_rpm_cb(unsigned mi)
{
	if (mi >= NMOTORS || !motor_is_active(mi) || !motor_has_speed_ctrl(mi)) {
		return 0.f;
	}
	if (!speed_rpm_ctrl_filt_valid[mi]) {
		return 0.f;
	}
	const float mag = speed_rpm_ctrl_filt[mi];
	return m_use_rev[mi] ? -mag : mag;
}

static void motor_ctrl_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	int64_t t_last_ctrl[NMOTORS];
	const int64_t t0 = k_uptime_get();

	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		t_last_ctrl[mi] = t0;
	}

	while (atomic_get(&app_run) != 0) {
		const int64_t now = k_uptime_get();

		/*
		 * CAN failsafe: if ADCS has gone silent, zero all setpoints so
		 * the PI loop winds duty down to zero and the wheels coast to
		 * rest. Applied every iteration (not just on the edge) so that
		 * any concurrent RX that arrives here will re-arm naturally on
		 * the next frame via the RX thread.
		 */
		if (motor_can_rx_age_ms() > CAN_RX_WATCHDOG_MS) {
			for (unsigned mi = 0; mi < NMOTORS; mi++) {
				rpm_ref_cmd[mi] = 0.f;
			}
		}

		for (unsigned mi = 0; mi < NMOTORS; mi++) {
			if (!motor_is_active(mi) || !motor_has_speed_ctrl(mi)) {
				continue;
			}
			const uint32_t ctrl_ms = motor_speed_ctrl_ms(mi);

			if ((now - t_last_ctrl[mi]) >= (int64_t)ctrl_ms) {
				motor_speed_pi_step(mi,
						    (uint32_t)(now - t_last_ctrl[mi]),
						    rpm_ref_cmd[mi]);
				t_last_ctrl[mi] = now;
			}
		}

		k_msleep(MOTOR_CTRL_BASE_MS);
	}
}

static void telemetry_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	int64_t t_last_telem = k_uptime_get();

	while (atomic_get(&app_run) != 0) {
		const int64_t now = k_uptime_get();

		if ((now - t_last_telem) >= TELEMETRY_MS) {
			t_last_telem = now;

			for (unsigned mi = 0; mi < NMOTORS; mi++) {
				if (!motor_is_active(mi) || !motor_has_speed_ctrl(mi)) {
					continue;
				}
				const uint32_t trn = m_trans[mi];
				const uint32_t dte = trn - speed_trans_prev_telem[mi];
				const float rpm_win =
					motor_rpm_from_transition_window(mi, trn,
									 speed_trans_prev_telem[mi],
									 TELEMETRY_MS);
				const float rpm_ctrl =
					speed_rpm_ctrl_filt_valid[mi] ?
					speed_rpm_ctrl_filt[mi] : fabsf(rpm_win);
				const float rpm_ctrl_signed =
					copysignf(rpm_ctrl, m_use_rev[mi] ? -1.f : 1.f);
				const float rpm_ref = rpm_ref_cmd[mi];
				const float rpm_err = fabsf(rpm_ref) - rpm_ctrl;
				const float rpm_edge = (mi == 0U) ? m1_rpm_from_hall_edges() : 0.f;
				const float meas_sign =
					(mi == 0U) ?
					(motor_rpm_meas_sign(mi) * (float)m1_last_period_sign) : 0.f;

				uint32_t dp = (uint32_t)(duty_pct[mi] * 1000.f + 0.5f);
				const char *cmd_dir = m_use_rev[mi] ? "rev" : "fwd";
				const char *meas_dir =
					(mi == 0U) ?
					((meas_sign > 0.f) ? "fwd" :
					 (meas_sign < 0.f) ? "rev" : "unk") :
					"n/a";

				speed_trans_prev_telem[mi] = trn;

				printk("M%uLOG,%lld,%d,%d\n",
				       mi + 1U, (long long)now,
				       (int)lrintf(rpm_ref),
				       (int)lrintf(rpm_ctrl_signed));

				printk("M%u ref=%d rpm_edge=%d rpm_win=%d rpm_ctrl=%d err=%d duty=%u.%03u pct cmd=%s meas=%s hall=%u dtrans=%u trans=%u\n",
				       mi + 1U, (int)lrintf(rpm_ref),
				       (int)lrintf(rpm_edge), (int)lrintf(rpm_win),
				       (int)lrintf(rpm_ctrl_signed),
				       (int)lrintf(rpm_err),
				       (unsigned)(dp / 1000U),
				       (unsigned)(dp % 1000U),
				       cmd_dir, meas_dir,
				       (unsigned)m_hall[mi], (unsigned)dte,
				       (unsigned)trn);
			}

			for (unsigned mi = 0; mi < NMOTORS; mi++) {
				if (motor_has_speed_ctrl(mi) && motor_is_active(mi)) {
					continue;
				}
				printk("M%u %s\n", mi + 1U,
				       motor_is_active(mi) ? "(enabled)"
							   : "(disabled)");
			}

			{
				const int64_t age = motor_can_rx_age_ms();
				const char *state =
					(age == INT64_MAX)          ? "no-rx" :
					(age > CAN_RX_WATCHDOG_MS)  ? "stale-WATCHDOG" :
								      "fresh";
				if (age == INT64_MAX) {
					printk("CANLOG rx=%u tx=%u age=never state=%s\n",
					       (unsigned)motor_can_rx_count(),
					       (unsigned)motor_can_tx_count(),
					       state);
				} else {
					printk("CANLOG rx=%u tx=%u age=%lld ms state=%s\n",
					       (unsigned)motor_can_rx_count(),
					       (unsigned)motor_can_tx_count(),
					       (long long)age, state);
				}
			}

			if (gpio_pin_get_dt(&kill_in) != 0) {
				printk("KILL asserted\n");
				atomic_set(&app_run, 0);
			}
		}

		k_msleep(1);
	}
}

int main(void)
{
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		if (!device_is_ready(motor_desc[mi].pwm)) {
			printk("ERR: PWM M%u not ready\n", mi + 1U);
			return -1;
		}
	}

	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		const motor_desc_t *m = &motor_desc[mi];

		for (int j = 0; j < 3; j++) {
			if (!gpio_is_ready_dt(m->hall[j])) {
				printk("ERR: M%u hall GPIO not ready\n", mi + 1U);
				return -1;
			}
			if (!gpio_is_ready_dt(m->en[j])) {
				printk("ERR: M%u enable GPIO not ready\n", mi + 1U);
				return -1;
			}
		}
	}

	if (!gpio_is_ready_dt(&fault_in) || !gpio_is_ready_dt(&kill_in)) {
		printk("ERR: Fault/Kill GPIO not ready\n");
		return -1;
	}

	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		const motor_desc_t *m = &motor_desc[mi];

		for (int j = 0; j < 3; j++) {
			gpio_pin_configure_dt(m->hall[j], GPIO_INPUT);
			gpio_pin_configure_dt(m->en[j], GPIO_OUTPUT_INACTIVE);
		}
	}

	gpio_pin_configure_dt(&fault_in, GPIO_INPUT);
	gpio_pin_configure_dt(&kill_in, GPIO_INPUT);

	all_motors_off();

	m1_cyc_hz = sys_clock_hw_cycles_per_sec();

	/* Hall edges/sec at max rated mech rpm → cycles per 60° step; dt_min ~30% of that. */
	{
		uint32_t edges_ps =
			(6U * POLE_PAIRS * (uint32_t)WHEEL_RPM_ABS_MAX) / 60U;

		if (edges_ps > 0U && m1_cyc_hz > edges_ps) {
			uint32_t cyc_per_step = m1_cyc_hz / edges_ps;

			m1_hall_dt_min_cyc = (cyc_per_step * 3U) / 10U;
		} else {
			m1_hall_dt_min_cyc = 4000U;
		}
		if (m1_hall_dt_min_cyc < 2500U) {
			m1_hall_dt_min_cyc = 2500U;
		}
	}

	m1_have_prev_edge = false;
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		m_use_rev[mi] = false;
	}
	m_use_rev[0] = (M1_OPEN_LOOP_REVERSE != 0);
	m_use_rev[1] = (M2_OPEN_LOOP_REVERSE != 0);
	m_use_rev[M3_IDX] = (M3_OPEN_LOOP_REVERSE != 0);
	m_use_rev[M4_IDX] = (M4_OPEN_LOOP_REVERSE != 0);
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		duty_pct[mi] = 0.f;
		rpm_ref_cmd[mi] = 0.f;
	}

	/*
	 * Start with zero setpoint on every wheel. The CAN RX watchdog in
	 * motor_ctrl_thread keeps it there until the first ADCS frame arrives;
	 * the PI loop therefore holds duty at zero during bring-up, which is
	 * the safe rotor state.
	 */
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		rpm_ref_cmd[mi] = 0.f;
		duty_pct[mi] = 0.f;
	}

	printk("\n============================================\n");
	printk("  BLDC - ISR 6-step  (%u motors)\n", NMOTORS);
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		printk("    M%u: %s\n", mi + 1U,
		       motor_enable[mi] ? "enabled" : "disabled");
	}
	printk("============================================\n");
	printk("Clock: 48 MHz (PLL)   PWM: 20 kHz\n");
	printk("Wheel |omega| max: %u rpm (mech)\n", WHEEL_RPM_ABS_MAX);
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		if (!motor_is_active(mi)) {
			continue;
		}
		if (!motor_has_speed_ctrl(mi)) {
			continue;
		}
		const uint32_t kp_ppm =
			(uint32_t)lrintf(motor_speed_kp(mi) * 1000000.f);
		const uint32_t ki_ppm =
			(uint32_t)lrintf(motor_speed_ki(mi) * 1000000.f);
		const uint32_t ff_rpm_per_pct =
			(uint32_t)lrintf(motor_speed_rpm_per_duty_pct(mi) * 1000.f);

		printk("M%u  CAN-driven  PI kp=%u ppm ki=%u ppm  FF %u.%03u rpm/pct  reverse=%u  duty cap %u%%  |omega| cap %u rpm\n",
		       mi + 1U,
		       (unsigned)kp_ppm, (unsigned)ki_ppm,
		       (unsigned)(ff_rpm_per_pct / 1000U),
		       (unsigned)(ff_rpm_per_pct % 1000U),
		       (unsigned)m_use_rev[mi],
		       (unsigned)motor_duty_max_pct(mi),
		       (unsigned)WHEEL_RPM_LIMIT);
	}
	printk("M1 hall dt_min: %u cycles (reject sub-step GPIO IRQ bunching)\n\n",
	       m1_hall_dt_min_cyc);

	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		uint8_t h = read_hall_code(motor_desc[mi].hall);

		m_hall[mi]  = h;
		m_trans[mi] = 0;
		hall_inv_streak[mi] = 0U;

		if (!motor_is_active(mi)) {
			m_run[mi] = false;
			printk("M%u hall: %u (inactive)\n", mi + 1U, h);
			continue;
		}

		if (hall_code_valid(h)) {
			m_run[mi] = true;
			printk("M%u hall: %u (running)\n", mi + 1U, h);
		} else {
			m_run[mi] = false;
			printk("M%u hall: %u — invalid, motor held off\n", mi + 1U, h);
		}
	}

	printk("Fault: %d  Kill: %d\n",
	       gpio_pin_get_dt(&fault_in), gpio_pin_get_dt(&kill_in));
	if (gpio_pin_get_dt(&kill_in) != 0) {
		printk("ABORT: KILL active\n");
		return -1;
	}

	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		const motor_desc_t *m = &motor_desc[mi];

		for (int j = 0; j < 3; j++) {
			gpio_flags_t irq_flags = motor_is_active(mi) ? GPIO_INT_EDGE_BOTH
								       : GPIO_INT_MODE_DISABLED;
			int err = gpio_pin_interrupt_configure_dt(m->hall[j], irq_flags);

			if (err != 0) {
				printk("WARN: M%u hall pin %d irq cfg err %d\n",
				       mi + 1U, j, err);
			}
		}
	}

	uint32_t mask_d = 0U;
	const struct device *port_d = NULL;

	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		const motor_desc_t *m = &motor_desc[mi];

		if (!motor_is_active(mi) || !m->halls_on_gpiod) {
			continue;
		}
		if (port_d == NULL) {
			port_d = m->hall[0]->port;
		} else if (m->hall[0]->port != port_d) {
			printk("ERR: active GPIOD hall motors must share one GPIO port\n");
			return -1;
		}
		for (int j = 0; j < 3; j++) {
			mask_d |= BIT(m->hall[j]->pin);
		}
	}

	if (mask_d != 0U) {
		gpio_init_callback(&hall_cb_gpiod, hall_isr_gpiod, mask_d);
		gpio_add_callback(port_d, &hall_cb_gpiod);
	}

	uint32_t mask_e = 0U;
	const struct device *port_e = NULL;

	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		const motor_desc_t *m = &motor_desc[mi];

		if (!motor_is_active(mi) || m->halls_on_gpiod) {
			continue;
		}
		if (port_e == NULL) {
			port_e = m->hall[0]->port;
		} else if (m->hall[0]->port != port_e) {
			printk("ERR: active non-GPIOD hall motors must share one GPIO port\n");
			return -1;
		}
		for (int j = 0; j < 3; j++) {
			mask_e |= BIT(m->hall[j]->pin);
		}
	}
	if (mask_e != 0U) {
		gpio_init_callback(&hall_cb_gpioe, hall_isr_gpioe, mask_e);
		gpio_add_callback(port_e, &hall_cb_gpioe);
	}

	if (motor_is_active(M3_IDX) && motor_is_active(M4_IDX)) {
		printk("\nNOTE: M3 (PD8/PD9) and M4 (PE8/PE9) share STM32 EXTI8/EXTI9 — "
		       "one EXTI mux cannot serve both; %u us hall poll keeps M3+M4 "
		       "in sync (fix hardware by moving halls off conflicting lines).\n\n",
		       (unsigned)HALL_M3M4_POLL_INTERVAL_US);
		k_timer_start(&hall_m3m4_timer,
			      K_USEC(HALL_M3M4_POLL_INTERVAL_US),
			      K_USEC(HALL_M3M4_POLL_INTERVAL_US));
	}

	refresh_outputs();

	printk("\nRunning - PI speed control on enabled controlled motors\n\n");

	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		speed_trans_prev_telem[mi] = m_trans[mi];
		speed_trans_prev_ctrl[mi] = m_trans[mi];
		motor_speed_ctrl_reset(mi);
	}
	atomic_set(&app_run, 1);
	(void)k_thread_create(&motor_ctrl_thread_data, motor_ctrl_stack,
			      K_THREAD_STACK_SIZEOF(motor_ctrl_stack),
			      motor_ctrl_thread, NULL, NULL, NULL,
			      MOTOR_CTRL_THREAD_PRIO, 0, K_NO_WAIT);

	const struct motor_can_cfg can_cfg = {
		.apply_setpoint = can_apply_setpoint_cb,
		.measure_rpm    = can_measure_rpm_cb,
	};
	if (motor_can_init(&can_cfg) != 0) {
		printk("WARN: motor_can_init failed — wheels will stay idle "
		       "(watchdog will hold rpm_ref_cmd=0)\n");
	}

	(void)k_thread_create(&telemetry_thread_data, telemetry_stack,
			      K_THREAD_STACK_SIZEOF(telemetry_stack),
			      telemetry_thread, NULL, NULL, NULL,
			      TELEMETRY_THREAD_PRIO, 0, K_NO_WAIT);

	while (atomic_get(&app_run) != 0) {
		for (unsigned mi = 0; mi < NMOTORS; mi++) {
			if (motor_is_active(mi) && !m_run[mi]) {
				process_motor(mi);
			}
		}
		k_msleep(1);
	}

	k_thread_join(&motor_ctrl_thread_data, K_MSEC(200));
	k_thread_join(&telemetry_thread_data, K_MSEC(200));

	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		const motor_desc_t *m = &motor_desc[mi];

		for (int j = 0; j < 3; j++) {
			gpio_pin_interrupt_configure_dt(m->hall[j],
							GPIO_INT_DISABLE);
		}
	}

	k_timer_stop(&hall_m3m4_timer);

	all_motors_off();
	printk("\nStopped.");
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		printk("  M%u trans=%u", mi + 1U, (unsigned)m_trans[mi]);
	}
	printk("\n");

	return 0;
}

#include <zephyr/kernel.h>
#include <zephyr/sys_clock.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

#define USER_NODE DT_PATH(zephyr_user)

/*
 * Open-loop duty sweep for one selected motor: logs duty_pct,rpm on UART.
 * Build with -DBLDC_MOTOR_CHARACTERIZATION=y (see app CMakeLists.txt).
 *
 * Board overlay (e.g. ut_core.overlay) defines M1..M4 on zephyr,user.
 * Flip motor_enable[] to test one motor; all four stay in the build.
 */
#define NMOTORS 4U
#define M3_IDX  2U
#define M4_IDX  3U
/* Characterization target motor index: M1=0, M2=1, M3=2, M4=3. */
#define CHAR_MOTOR_IDX 1U

/* Per-motor: false = PWM/enables off (bench one motor at a time). */
static const bool motor_enable[] = {
	false,  /* M1 — duty sweep */
	true, /* M2 */
	false, /* M3 */
	false, /* M4 */
};

BUILD_ASSERT(ARRAY_SIZE(motor_enable) == NMOTORS, "motor_enable vs NMOTORS");
BUILD_ASSERT(CHAR_MOTOR_IDX < NMOTORS, "CHAR_MOTOR_IDX out of range");

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

/* UART CSV sweep 0.0 .. 95.0 % in 0.1 % steps (~951 lines). */
#define CHAR_SETTLE_MS   400U
#define CHAR_SAMPLE_MS   400U
#define M1_OPEN_LOOP_REVERSE  0 /* 1 = reverse commutation table */

/* Hall sequence for +rpm (tune order if your wiring differs). */
static const uint8_t hall_ring_fwd[6] = {1, 5, 4, 6, 2, 3};

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

static volatile bool m1_use_rev;
/* Hall edge timing for M1 (ISR writes, control loop reads). */
static uint32_t m1_cyc_hz;
static uint32_t m1_hall_dt_min_cyc;
static volatile uint32_t m1_prev_edge_cyc;
static volatile bool m1_have_prev_edge;
static volatile uint32_t m1_last_period_cyc;
static volatile int8_t m1_last_period_sign;
static volatile uint32_t m1_last_edge_cyc;

static struct gpio_callback hall_cb_gpiod;
static struct gpio_callback hall_cb_gpioe;

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
	if (mi == CHAR_MOTOR_IDX && d > (float)M1_DUTY_MAX_PCT) {
		d = (float)M1_DUTY_MAX_PCT;
	} else if (mi != CHAR_MOTOR_IDX && d > 100.f) {
		d = 100.f;
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

		const struct comm_step *tbl =
			(mi == 0U) ? (m1_use_rev ? rev_table : fwd_table) : fwd_table;

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
		const struct comm_step *tbl =
			(mi == 0U) ? (m1_use_rev ? rev_table : fwd_table) : fwd_table;

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

static float m1_rpm_from_transition_window(uint32_t trans_now, uint32_t trans_prev,
					    uint32_t dt_ms)
{
	if (dt_ms == 0U) {
		return 0.f;
	}

	const uint32_t dtrans = trans_now - trans_prev;
	const float mag =
		(float)dtrans * 1000.f * 10.f /
		((float)POLE_PAIRS * (float)dt_ms);
	const float meas_sign =
		M1_RPM_MEAS_SIGN * (float)m1_last_period_sign;
	const float cmd_sign = m1_use_rev ? -1.f : 1.f;
	const float sgn = (meas_sign != 0.f) ? meas_sign : cmd_sign;

	return copysignf(mag, sgn);
}

static void all_motors_off(void)
{
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		const motor_desc_t *m = &motor_desc[mi];

		pwm_all_off(m->pwm, m->tim_ch, m->en);
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
	m1_use_rev = (M1_OPEN_LOOP_REVERSE != 0);
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		duty_pct[mi] = 0.f;
	}

	printk("\n============================================\n");
	printk("  BLDC - motor characterization  (%u motors)\n", NMOTORS);
	for (unsigned mi = 0; mi < NMOTORS; mi++) {
		printk("    M%u: %s\n", mi + 1U,
		       motor_enable[mi] ? "enabled" : "disabled");
	}
	printk("============================================\n");
	printk("Clock: 48 MHz (PLL)   PWM: 20 kHz\n");
	printk("Wheel |omega| max: %u rpm (mech)\n", WHEEL_RPM_ABS_MAX);
	printk("M%u duty sweep 0.0..%u%% step 0.1%%  settle %u ms  sample %u ms  reverse=%u\n",
	       (unsigned)(CHAR_MOTOR_IDX + 1U), (unsigned)M1_DUTY_MAX_PCT,
	       (unsigned)CHAR_SETTLE_MS, (unsigned)CHAR_SAMPLE_MS,
	       (unsigned)M1_OPEN_LOOP_REVERSE);
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

	printk("\nCSV (duty_pct,rpm_abs) — lines starting with '#' are comments\n\n");

	printk("# begin duty_pct,rpm_abs\n");

	for (int d10 = 0; d10 <= 950; d10++) {
		const float duty = (float)d10 / 10.f;

		if (gpio_pin_get_dt(&kill_in) != 0) {
			printk("# abort kill\n");
			break;
		}

		duty_pct[CHAR_MOTOR_IDX] = clampf(duty, 0.f, (float)M1_DUTY_MAX_PCT);
		refresh_outputs();

		for (int w = 0; w < 5; w++) {
			if (motor_is_active(CHAR_MOTOR_IDX) && !m_run[CHAR_MOTOR_IDX]) {
				process_motor(CHAR_MOTOR_IDX);
			}
			k_msleep(2);
		}

		k_msleep(CHAR_SETTLE_MS);

		const uint32_t tr_a = m_trans[CHAR_MOTOR_IDX];

		k_msleep(CHAR_SAMPLE_MS);

		const uint32_t tr_b = m_trans[CHAR_MOTOR_IDX];
		const float rpm_win =
			fabsf(m1_rpm_from_transition_window(tr_b, tr_a, CHAR_SAMPLE_MS));

		const int duty_i = (int)lrintf(duty * 1000.f);
		const int rpm_i = (int)lrintf(rpm_win);

		printk("%d.%03u,%d\n",
		       duty_i / 1000,
		       (unsigned)(duty_i % 1000),
		       rpm_i);

		for (unsigned mi = 0; mi < NMOTORS; mi++) {
			if (motor_is_active(mi) && !m_run[mi]) {
				process_motor(mi);
			}
		}

		k_msleep(1);
	}

	printk("# end duty_pct,rpm_abs\n");

	duty_pct[CHAR_MOTOR_IDX] = 0.f;
	refresh_outputs();
	all_motors_off();

	while (1) {
		if (gpio_pin_get_dt(&kill_in) != 0) {
			break;
		}

		k_msleep(500);
	}

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

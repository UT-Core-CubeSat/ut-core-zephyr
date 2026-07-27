#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

#define USER_NODE    DT_PATH(zephyr_user)
#define PWM_DEV_NODE DT_NODELABEL(pwm4)

#define PWM_PERIOD_NS  PWM_HZ(37000)
#define PWM_CHANNEL_A  1U
#define PWM_CHANNEL_B  2U
#define PWM_CHANNEL_C  3U

#define DUTY_PERCENT 75U

static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_DEV_NODE);

static const struct gpio_dt_spec hall_a  = GPIO_DT_SPEC_GET(USER_NODE, hall_a_gpios);
static const struct gpio_dt_spec hall_b  = GPIO_DT_SPEC_GET(USER_NODE, hall_b_gpios);
static const struct gpio_dt_spec hall_c  = GPIO_DT_SPEC_GET(USER_NODE, hall_c_gpios);
static const struct gpio_dt_spec fault_in = GPIO_DT_SPEC_GET(USER_NODE, fault_gpios);
static const struct gpio_dt_spec kill_in  = GPIO_DT_SPEC_GET(USER_NODE, kill_gpios);
static const struct gpio_dt_spec en_out   = GPIO_DT_SPEC_GET(USER_NODE, en_gpios);

static const uint32_t phase_ch[] = { PWM_CHANNEL_A, PWM_CHANNEL_B, PWM_CHANNEL_C };

struct comm_step {
	int8_t high;
	int8_t low;
};

/*
 * Derived from slow-step equilibrium test:
 *   Phase 0 (PD12) equilibrium at hall=4 → 0° electrical
 *   Phase 2 (PD14) equilibrium at hall=2 → 120°
 *   Phase 1 (PD13) equilibrium at hall=1 → 240°
 *
 * Forward direction (hand-spin): 4→6→2→3→1→5 (increasing angle)
 *
 * Each step's field = midpoint of (high_phase_axis, opposite_of_low_phase_axis).
 * For 90° lead at each sector center:
 *
 *   hall=4 (0°-60°):   field at 120° → {2,1}  P2 hi, P1 lo
 *   hall=6 (60°-120°):  field at 180° → {2,0}  P2 hi, P0 lo
 *   hall=2 (120°-180°): field at 240° → {1,0}  P1 hi, P0 lo
 *   hall=3 (180°-240°): field at 300° → {1,2}  P1 hi, P2 lo
 *   hall=1 (240°-300°): field at 0°   → {0,2}  P0 hi, P2 lo
 *   hall=5 (300°-360°): field at 60°  → {0,1}  P0 hi, P1 lo
 */
static const struct comm_step table[8] = {
	[0] = {-1, -1},
	[1] = { 0,  2},
	[2] = { 1,  0},
	[3] = { 1,  2},
	[4] = { 2,  1},
	[5] = { 0,  1},
	[6] = { 2,  0},
	[7] = {-1, -1},
};

static int set_phase(uint32_t channel, uint8_t percent)
{
	uint64_t pulse = ((uint64_t)PWM_PERIOD_NS * percent) / 100U;
	return pwm_set(pwm_dev, channel, PWM_PERIOD_NS, (uint32_t)pulse, PWM_POLARITY_NORMAL);
}

static void apply_step(const struct comm_step *s)
{
	for (int i = 0; i < 3; i++) {
		if (i == s->high)
			set_phase(phase_ch[i], DUTY_PERCENT);
		else
			set_phase(phase_ch[i], 0);
	}
}

static void all_off(void)
{
	set_phase(PWM_CHANNEL_A, 0);
	set_phase(PWM_CHANNEL_B, 0);
	set_phase(PWM_CHANNEL_C, 0);
}

static uint8_t read_halls(void)
{
	uint8_t a = gpio_pin_get_dt(&hall_a) > 0 ? 1U : 0U;
	uint8_t b = gpio_pin_get_dt(&hall_b) > 0 ? 1U : 0U;
	uint8_t c = gpio_pin_get_dt(&hall_c) > 0 ? 1U : 0U;
	return (uint8_t)(a | (b << 1) | (c << 2));
}

int main(void)
{
	if (!device_is_ready(pwm_dev) ||
	    !gpio_is_ready_dt(&hall_a) || !gpio_is_ready_dt(&hall_b) ||
	    !gpio_is_ready_dt(&hall_c) || !gpio_is_ready_dt(&fault_in) ||
	    !gpio_is_ready_dt(&kill_in) || !gpio_is_ready_dt(&en_out)) {
		printk("BLDC: device not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&hall_a,   GPIO_INPUT);
	gpio_pin_configure_dt(&hall_b,   GPIO_INPUT);
	gpio_pin_configure_dt(&hall_c,   GPIO_INPUT);
	gpio_pin_configure_dt(&fault_in, GPIO_INPUT);
	gpio_pin_configure_dt(&kill_in,  GPIO_INPUT);
	gpio_pin_configure_dt(&en_out,   GPIO_OUTPUT_INACTIVE);

	all_off();
	gpio_pin_set_dt(&en_out, 1);

	printk("BLDC: 90-deg-lead table, hall-based\n");

	uint8_t prev_hall = read_halls();
	const struct comm_step *s = &table[prev_hall];
	if (s->high >= 0)
		apply_step(s);

	uint32_t transitions = 0;
	int64_t last_print = k_uptime_get();

	while (1) {
		uint8_t hall = read_halls();

		if (hall != prev_hall) {
			s = &table[hall];
			if (s->high >= 0)
				apply_step(s);
			prev_hall = hall;
			transitions++;
		}

		int64_t now = k_uptime_get();
		if ((now - last_print) >= 2000) {
			uint32_t erpm = (transitions * 60U * 1000U) /
					(6U * (uint32_t)(now - last_print));
			uint32_t rpm = erpm / 4U;
			printk("RPM=%u  trans=%u\n", rpm, transitions);
			transitions = 0;
			last_print = now;
		}

		k_busy_wait(100);
	}
}

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

#define USER_NODE    DT_PATH(zephyr_user)
#define PWM_DEV_NODE DT_NODELABEL(pwm4)

#define PWM_PERIOD_NS  PWM_HZ(20000)
#define PWM_CHANNEL_A  1U
#define PWM_CHANNEL_B  2U
#define PWM_CHANNEL_C  3U

#define DUTY_PERCENT   50U

static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_DEV_NODE);

static const struct gpio_dt_spec hall_a   = GPIO_DT_SPEC_GET(USER_NODE, hall_a_gpios);
static const struct gpio_dt_spec hall_b   = GPIO_DT_SPEC_GET(USER_NODE, hall_b_gpios);
static const struct gpio_dt_spec hall_c   = GPIO_DT_SPEC_GET(USER_NODE, hall_c_gpios);
static const struct gpio_dt_spec fault_in = GPIO_DT_SPEC_GET(USER_NODE, fault_gpios);
static const struct gpio_dt_spec kill_in  = GPIO_DT_SPEC_GET(USER_NODE, kill_gpios);

static const struct gpio_dt_spec en_a = GPIO_DT_SPEC_GET(USER_NODE, en_a_gpios);
static const struct gpio_dt_spec en_b = GPIO_DT_SPEC_GET(USER_NODE, en_b_gpios);
static const struct gpio_dt_spec en_c = GPIO_DT_SPEC_GET(USER_NODE, en_c_gpios);

static const uint32_t phase_ch[3] = {
	PWM_CHANNEL_A,
	PWM_CHANNEL_B,
	PWM_CHANNEL_C,
};

static const struct gpio_dt_spec *const phase_en[3] = {
	&en_a,
	&en_b,
	&en_c,
};

struct comm_step {
	int8_t high;
	int8_t low;
};

/*
 * Your proven-good hall mapping:
 * hall order observed: 4 -> 6 -> 2 -> 3 -> 1 -> 5
 *
 * This is the table that spun well before, now reused with:
 *   high phase  = EN on, PWM on IN
 *   low phase   = EN on, IN low
 *   float phase = EN off
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

static int set_phase_pwm(uint32_t channel, uint8_t percent)
{
	uint64_t pulse = ((uint64_t)PWM_PERIOD_NS * percent) / 100U;
	return pwm_set(pwm_dev, channel, PWM_PERIOD_NS, (uint32_t)pulse, PWM_POLARITY_NORMAL);
}

static int set_phase_low(uint32_t channel)
{
	return pwm_set(pwm_dev, channel, PWM_PERIOD_NS, 0U, PWM_POLARITY_NORMAL);
}

static void set_enable(int phase, bool on)
{
	(void)gpio_pin_set_dt(phase_en[phase], on ? 1 : 0);
}

static void all_off(void)
{
	(void)set_phase_low(PWM_CHANNEL_A);
	(void)set_phase_low(PWM_CHANNEL_B);
	(void)set_phase_low(PWM_CHANNEL_C);

	set_enable(0, false);
	set_enable(1, false);
	set_enable(2, false);
}

static void apply_step(const struct comm_step *s)
{
	for (int i = 0; i < 3; i++) {
		if (i == s->high) {
			(void)set_phase_pwm(phase_ch[i], DUTY_PERCENT);
			set_enable(i, true);
		} else if (i == s->low) {
			(void)set_phase_low(phase_ch[i]);
			set_enable(i, true);
		} else {
			(void)set_phase_low(phase_ch[i]);
			set_enable(i, false);
		}
	}
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
	    !gpio_is_ready_dt(&kill_in) ||
	    !gpio_is_ready_dt(&en_a) || !gpio_is_ready_dt(&en_b) ||
	    !gpio_is_ready_dt(&en_c)) {
		printk("BLDC: device not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&hall_a,   GPIO_INPUT);
	gpio_pin_configure_dt(&hall_b,   GPIO_INPUT);
	gpio_pin_configure_dt(&hall_c,   GPIO_INPUT);
	gpio_pin_configure_dt(&fault_in, GPIO_INPUT);
	gpio_pin_configure_dt(&kill_in,  GPIO_INPUT);

	gpio_pin_configure_dt(&en_a, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&en_b, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&en_c, GPIO_OUTPUT_INACTIVE);

	all_off();
	k_msleep(50);

	int flt_prev  = gpio_pin_get_dt(&fault_in);
	int kill_prev = gpio_pin_get_dt(&kill_in);
	uint8_t hall0 = read_halls();

	printk("BLDC: advanced table with real float\n");
	printk("BOOT: FLT=%d KILL=%d HALL=%u\n", flt_prev, kill_prev, hall0);

	const struct comm_step *s = &table[hall0];
	if (s->high >= 0) {
		apply_step(s);
	} else {
		printk("BOOT: invalid hall state %u\n", hall0);
		all_off();
		return 0;
	}

	uint8_t prev_hall = hall0;
	uint32_t transitions = 0;
	int64_t last_print = k_uptime_get();

	while (1) {
		// int flt = gpio_pin_get_dt(&fault_in);
		// if (flt != flt_prev) {
		// 	printk("FLT changed: %d -> %d\n", flt_prev, flt);
		// 	flt_prev = flt;
		// }

		int kill = gpio_pin_get_dt(&kill_in);
		if (kill != kill_prev) {
			printk("KILL changed: %d -> %d\n", kill_prev, kill);
			kill_prev = kill;
		}

		if (kill != 0) {
			printk("KILL asserted\n");
			all_off();
			return 0;
		}

		uint8_t hall = read_halls();

		if (hall != prev_hall) {
			s = &table[hall];
			if (s->high >= 0) {
				apply_step(s);
			} else {
				printk("Invalid hall state: %u\n", hall);
				all_off();
				return 0;
			}
			prev_hall = hall;
			transitions++;
		}

		int64_t now = k_uptime_get();
		if ((now - last_print) >= 2000) {
			uint32_t erpm = (transitions * 60U * 1000U) /
					(6U * (uint32_t)(now - last_print));
			uint32_t rpm = erpm / 4U; /* 8-pole motor => 4 pole pairs */

			printk("hall=%u rpm=%u trans=%u flt=%d kill=%d\n",
			       prev_hall, rpm, transitions, flt_prev, kill_prev);

			transitions = 0;
			last_print = now;
		}

		k_busy_wait(100);
	}
}
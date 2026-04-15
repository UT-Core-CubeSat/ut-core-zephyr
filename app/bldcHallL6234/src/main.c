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

#define TEST_DUTY  20U

static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_DEV_NODE);

static const struct gpio_dt_spec hall_a   = GPIO_DT_SPEC_GET(USER_NODE, hall_a_gpios);
static const struct gpio_dt_spec hall_b   = GPIO_DT_SPEC_GET(USER_NODE, hall_b_gpios);
static const struct gpio_dt_spec hall_c   = GPIO_DT_SPEC_GET(USER_NODE, hall_c_gpios);
static const struct gpio_dt_spec fault_in = GPIO_DT_SPEC_GET(USER_NODE, fault_gpios);
static const struct gpio_dt_spec kill_in  = GPIO_DT_SPEC_GET(USER_NODE, kill_gpios);

static const struct gpio_dt_spec en_a = GPIO_DT_SPEC_GET(USER_NODE, en_a_gpios);
static const struct gpio_dt_spec en_b = GPIO_DT_SPEC_GET(USER_NODE, en_b_gpios);
static const struct gpio_dt_spec en_c = GPIO_DT_SPEC_GET(USER_NODE, en_c_gpios);

static const uint32_t phase_ch[3] = { PWM_CHANNEL_A, PWM_CHANNEL_B, PWM_CHANNEL_C };
static const struct gpio_dt_spec *const phase_en[3] = { &en_a, &en_b, &en_c };
static const char phase_name[3] = { 'A', 'B', 'C' };

static void set_enable(int phase, bool on)
{
	gpio_pin_set_dt(phase_en[phase], on ? 1 : 0);
}

static void all_off(void)
{
	for (int i = 0; i < 3; i++) {
		pwm_set(pwm_dev, phase_ch[i], PWM_PERIOD_NS, 0, PWM_POLARITY_NORMAL);
		set_enable(i, false);
	}
}

static void drive_pair(int high, int low)
{
	uint64_t pulse = ((uint64_t)PWM_PERIOD_NS * TEST_DUTY) / 100U;
	for (int i = 0; i < 3; i++) {
		if (i == high) {
			pwm_set(pwm_dev, phase_ch[i], PWM_PERIOD_NS, (uint32_t)pulse, PWM_POLARITY_NORMAL);
			set_enable(i, true);
		} else if (i == low) {
			pwm_set(pwm_dev, phase_ch[i], PWM_PERIOD_NS, 0, PWM_POLARITY_NORMAL);
			set_enable(i, true);
		} else {
			pwm_set(pwm_dev, phase_ch[i], PWM_PERIOD_NS, 0, PWM_POLARITY_NORMAL);
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

static uint8_t read_halls_stable(void)
{
	uint8_t val = read_halls();
	for (int i = 0; i < 10; i++) {
		k_msleep(20);
		uint8_t v2 = read_halls();
		if (v2 != val) {
			val = v2;
			i = 0;
		}
	}
	return val;
}

int main(void)
{
	if (!device_is_ready(pwm_dev) ||
	    !gpio_is_ready_dt(&hall_a) || !gpio_is_ready_dt(&hall_b) ||
	    !gpio_is_ready_dt(&hall_c) || !gpio_is_ready_dt(&fault_in) ||
	    !gpio_is_ready_dt(&kill_in) ||
	    !gpio_is_ready_dt(&en_a) || !gpio_is_ready_dt(&en_b) ||
	    !gpio_is_ready_dt(&en_c)) {
		printk("device not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&hall_a,   GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_configure_dt(&hall_b,   GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_configure_dt(&hall_c,   GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_configure_dt(&fault_in, GPIO_INPUT);
	gpio_pin_configure_dt(&kill_in,  GPIO_INPUT);
	gpio_pin_configure_dt(&en_a, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&en_b, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&en_c, GPIO_OUTPUT_INACTIVE);

	all_off();

	printk("\n=== Drive pair -> hall equilibrium mapping ===\n");
	printk("Duty: %u%%  Testing all 6 drive pairs\n\n", TEST_DUTY);

	struct { int high; int low; } pairs[6] = {
		{0, 1}, {0, 2}, {1, 0}, {1, 2}, {2, 0}, {2, 1},
	};

	uint8_t equil[6];

	for (int p = 0; p < 6; p++) {
		int h = pairs[p].high;
		int l = pairs[p].low;

		all_off();
		k_msleep(300);

		drive_pair(h, l);
		k_msleep(800);

		uint8_t settled = read_halls_stable();
		equil[p] = settled;

		printk("  %c->%c  settles at hall=%u\n", phase_name[h], phase_name[l], settled);

		all_off();
		k_msleep(300);
	}

	printk("\n--- Build table from results ---\n");
	printk("Copy this into 6stepWorking.c:\n\n");

	/*
	 * For correct forward commutation, each drive pair's equilibrium
	 * tells us which hall state it "owns." The table entry for a hall
	 * state should be the drive pair whose equilibrium is one step
	 * AHEAD -- i.e., the pair that pulls the rotor forward through
	 * the current state.
	 *
	 * We need the hall sequence to figure out "one step ahead."
	 * From manual spin: 3 -> 2 -> 6 -> 4 -> 5 -> 1 (one direction)
	 * Reverse:          1 -> 5 -> 4 -> 6 -> 2 -> 3 (other direction)
	 *
	 * Print both options so the user can pick.
	 */

	/* direction A: 3->2->6->4->5->1 */
	static const uint8_t seq_a[6] = {3, 2, 6, 4, 5, 1};
	/* direction B: 1->5->4->6->2->3 */
	static const uint8_t seq_b[6] = {1, 5, 4, 6, 2, 3};

	for (int dir = 0; dir < 2; dir++) {
		const uint8_t *seq = (dir == 0) ? seq_a : seq_b;
		printk("\n// Direction %c: %u->%u->%u->%u->%u->%u\n",
		       'A' + dir, seq[0], seq[1], seq[2], seq[3], seq[4], seq[5]);
		printk("static const struct comm_step table[8] = {\n");
		printk("\t[0] = {-1, -1},\n");

		for (int s = 0; s < 6; s++) {
			uint8_t hall_now = seq[s];
			uint8_t hall_next = seq[(s + 1) % 6];

			int best = -1;
			for (int p = 0; p < 6; p++) {
				if (equil[p] == hall_next) {
					best = p;
					break;
				}
			}

			if (best >= 0) {
				printk("\t[%u] = {%2d, %2d},   /* %c->%c */\n",
				       hall_now,
				       pairs[best].high, pairs[best].low,
				       phase_name[pairs[best].high],
				       phase_name[pairs[best].low]);
			} else {
				printk("\t[%u] = {-1, -1},   /* NO MATCH */\n", hall_now);
			}
		}

		printk("\t[7] = {-1, -1},\n};\n");
	}

	printk("\n=== done ===\n");
	return 0;
}

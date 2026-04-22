#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/printk.h>

#include <stm32_ll_tim.h>

/* Mux enable */
static const struct device *const gpioa = DEVICE_DT_GET(DT_NODELABEL(gpioa));

/* PWM devices */
static const struct device *const pwm1_dev = DEVICE_DT_GET(DT_NODELABEL(pwm1));
static const struct device *const pwm3_dev = DEVICE_DT_GET(DT_NODELABEL(pwm3));

#define PWM_FREQ    20000U
#define PWM_PERIOD  PWM_HZ(PWM_FREQ)

/* Channel numbers */
#define TIM1_CH4    4U
#define TIM3_CH1    1U
#define TIM3_CH2    2U
#define TIM3_CH4    4U

/* Enable TIM1 complementary output for CH4N on PC5 */
static void tim1_enable_ch4n(void)
{
	TIM1->CCER |= TIM_CCER_CC4NE | TIM_CCER_CC4E;
	TIM1->BDTR |= TIM_BDTR_MOE;
}

/* Set all 4 channels to the same duty cycle */
static void set_all_duty(uint32_t duty_percent)
{
	uint32_t pulse_tim = ((uint64_t)PWM_PERIOD * duty_percent) / 100U;

	/* PC5: TIM1_CH4N */
	pwm_set(pwm1_dev, TIM1_CH4, PWM_PERIOD, pulse_tim, PWM_POLARITY_NORMAL);
	TIM1->CCER |= TIM_CCER_CC4NE | TIM_CCER_CC4E;
	TIM1->BDTR |= TIM_BDTR_MOE;

	// /* PC6: TIM3_CH1 */
	// pwm_set(pwm3_dev, TIM3_CH1, PWM_PERIOD, pulse_tim, PWM_POLARITY_NORMAL);

	// /* PA7: TIM3_CH2 */
	// pwm_set(pwm3_dev, TIM3_CH2, PWM_PERIOD, pulse_tim, PWM_POLARITY_NORMAL);

	// /* PC9: TIM3_CH4 */
	// pwm_set(pwm3_dev, TIM3_CH4, PWM_PERIOD, pulse_tim, PWM_POLARITY_NORMAL);
}

int main(void)
{
	int ret;

	printk("\n=== Solarconglomerator PWM Test (4 channels) ===\n");

	/* PA5: mux enable */
	if (!device_is_ready(gpioa)) {
		printk("ERROR: GPIOA not ready!\n");
		return 0;
	}
	gpio_pin_configure(gpioa, 5, GPIO_OUTPUT_ACTIVE);
	gpio_pin_set(gpioa, 5, 0);
	printk("PA5: mux enable LOW\n");

    const struct device *const gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));
    gpio_pin_configure(gpioc, 2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpioc, 3, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpioc, 4, GPIO_OUTPUT_INACTIVE);
    gpio_pin_set(gpioc, 2, 0);
    gpio_pin_set(gpioc, 3, 0);
    gpio_pin_set(gpioc, 4, 0);

	/* Check devices */
	if (!device_is_ready(pwm1_dev)) {
		printk("ERROR: TIM1 PWM not ready!\n");
		return 0;
	}
	if (!device_is_ready(pwm3_dev)) {
		printk("ERROR: TIM3 PWM not ready!\n");
		return 0;
	}

	/* --- Set all channels to 50% --- */
	uint32_t half = PWM_PERIOD / 2;

	/* PC5: TIM1_CH4N */
	ret = pwm_set(pwm1_dev, TIM1_CH4, PWM_PERIOD, half, PWM_POLARITY_NORMAL);
	tim1_enable_ch4n();
	printk("PC5 TIM1_CH4N: %s\n", ret ? "FAIL" : "OK");

	// /* PC6: TIM3_CH1 */
	// ret = pwm_set(pwm3_dev, TIM3_CH1, PWM_PERIOD, half, PWM_POLARITY_NORMAL);
	// printk("PC6 TIM3_CH1:  %s\n", ret ? "FAIL" : "OK");

	// /* PA7: TIM3_CH2 */
	// ret = pwm_set(pwm3_dev, TIM3_CH2, PWM_PERIOD, half, PWM_POLARITY_NORMAL);
	// printk("PA7 TIM3_CH2:  %s\n", ret ? "FAIL" : "OK");

	// /* PC9: TIM3_CH4 */
	// ret = pwm_set(pwm3_dev, TIM3_CH4, PWM_PERIOD, half, PWM_POLARITY_NORMAL);
	// printk("PC9 TIM3_CH4:  %s\n", ret ? "FAIL" : "OK");

	printk("\nAll channels at 50%% -- verify with scope.\n");
	printk("Sweep starts in 5 s ...\n\n");
	k_msleep(5000);

	/* Sweep all channels together */
	// while (1) {
	// 	for (int duty = 0; duty <= 100; duty += 5) {
	// 		set_all_duty(duty);
	// 		printk("duty=%3d%%\n", duty);
	// 		k_msleep(3000);
	// 	}

	// 	for (int duty = 100; duty >= 0; duty -= 5) {
	// 		set_all_duty(duty);
	// 		printk("duty=%3d%%\n", duty);
	// 		k_msleep(3000);
	// 	}
	// }
while (1) {
    set_all_duty(35);
    k_msleep(10000);
}
	return 0;
}
/**
 * @file main.c
 * @brief Bench PWM validation tool for the Solar Conglomerate (magnetorquer)
 *        board's 4 coil-drive channels.
 *
 * Drives all 4 magnetorquer PWM outputs (TIM1_CH4N on PC5, TIM3_CH1/CH2/CH4
 * on PC6/PA7/PC9) to the same duty cycle simultaneously, sweeping 0-100%
 * and back in 5% steps for oscilloscope verification. Not flight firmware
 * — no CAN, no per-face control, just a raw hardware bring-up check.
 */


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

/**
 * @brief Enable TIM1's complementary CH4N output and set the timer's
 *        master output enable bit.
 *
 * PC5 is driven via TIM1's complementary channel output, which requires
 * CC4NE/CC4E in CCER and MOE in BDTR to be set or the pin stays gated
 * off regardless of what pwm_set() configures — Zephyr's PWM API does
 * not expose these bits directly, hence the raw register access.
 */
static void tim1_enable_ch4n(void)
{
    TIM1->CCER |= TIM_CCER_CC4NE | TIM_CCER_CC4E;
    TIM1->BDTR |= TIM_BDTR_MOE;
}

/**
 * @brief Set all 4 magnetorquer PWM channels to the same duty cycle.
 * @param duty_percent Duty cycle, 0-100.
 *
 * PC5 (TIM1_CH4N) is electrically inverted relative to the other three
 * channels, so its pulse value is computed as (PWM_PERIOD - pulse_tim)
 * to keep it tracking the same logical duty cycle as PC6/PA7/PC9.
 */
static void set_all_duty(uint32_t duty_percent)
{
    uint32_t pulse_tim = ((uint64_t)PWM_PERIOD * duty_percent) / 100U;
    uint32_t pulse_inv = PWM_PERIOD - pulse_tim;  /* inverted for CH4N */

    /* PC5: TIM1_CH4N — complementary, so invert */
    pwm_set(pwm1_dev, TIM1_CH4, PWM_PERIOD, pulse_inv, PWM_POLARITY_NORMAL);
    TIM1->CCER |= TIM_CCER_CC4NE | TIM_CCER_CC4E;
    TIM1->BDTR |= TIM_BDTR_MOE;

    /* PC6: TIM3_CH1 */
    pwm_set(pwm3_dev, TIM3_CH1, PWM_PERIOD, pulse_tim, PWM_POLARITY_NORMAL);

    /* PA7: TIM3_CH2 */
    pwm_set(pwm3_dev, TIM3_CH2, PWM_PERIOD, pulse_tim, PWM_POLARITY_NORMAL);

    /* PC9: TIM3_CH4 */
    pwm_set(pwm3_dev, TIM3_CH4, PWM_PERIOD, pulse_tim, PWM_POLARITY_NORMAL);
}

/**
 * @brief Entry point: verifies PWM/GPIO hardware, sets all 4 channels to
 *        50% for a scope check, then sweeps duty 0→100→0% indefinitely.
 *
 * Boot sequence: configure PA5 (mux enable) low, verify TIM1/TIM3 PWM
 * devices are ready, set all 4 channels to 50% duty (printing per-channel
 * pass/fail), wait 5 s, then loop forever sweeping duty in 5% steps every
 * 8 seconds, first ascending 0→100% then descending back to 0%.
 *
 * @note A commented-out block configures PC2/PC3/PC4 as inactive GPIO
 *       outputs — looks like an earlier or alternate mux/enable
 *       approach that was abandoned or is still under investigation,
 *       left in as dead code rather than removed.
 *
 * @return 0 if GPIOA, TIM1 PWM, or TIM3 PWM device isn't ready; otherwise
 *         does not return (infinite sweep loop).
 */
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

    // const struct device *const gpioc = DEVICE_DT_GET(DT_NODELABEL(gpioc));
    // gpio_pin_configure(gpioc, 2, GPIO_OUTPUT_INACTIVE);
    // gpio_pin_configure(gpioc, 3, GPIO_OUTPUT_INACTIVE);
    // gpio_pin_configure(gpioc, 4, GPIO_OUTPUT_INACTIVE);
    // gpio_pin_set(gpioc, 2, 0);
    // gpio_pin_set(gpioc, 3, 0);
    // gpio_pin_set(gpioc, 4, 0);

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

    /* PC6: TIM3_CH1 */
    ret = pwm_set(pwm3_dev, TIM3_CH1, PWM_PERIOD, half, PWM_POLARITY_NORMAL);
    printk("PC6 TIM3_CH1:  %s\n", ret ? "FAIL" : "OK");

    /* PA7: TIM3_CH2 */
    ret = pwm_set(pwm3_dev, TIM3_CH2, PWM_PERIOD, half, PWM_POLARITY_NORMAL);
    printk("PA7 TIM3_CH2:  %s\n", ret ? "FAIL" : "OK");

    /* PC9: TIM3_CH4 */
    ret = pwm_set(pwm3_dev, TIM3_CH4, PWM_PERIOD, half, PWM_POLARITY_NORMAL);
    printk("PC9 TIM3_CH4:  %s\n", ret ? "FAIL" : "OK");

    printk("\nAll channels at 50%% -- verify with scope.\n");
    printk("Sweep starts in 5 s ...\n\n");
    k_msleep(5000);

    /* Sweep all channels together */
    while (1) {
        for (int duty = 0; duty <= 100; duty += 5) {
            set_all_duty(duty);
            printk("duty=%3d%%\n", duty);
            k_msleep(8000);
        }

        for (int duty = 100; duty >= 0; duty -= 5) {
            set_all_duty(duty);
            printk("duty=%3d%%\n", duty);
            k_msleep(8000);
        }
    }
    return 0;
}

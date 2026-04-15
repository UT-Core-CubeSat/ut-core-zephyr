# BLDC Hall Commutation with ST L6234

## Hardware Setup

The ST L6234 is a triple half-bridge motor driver. It takes three PWM inputs
(IN_A, IN_B, IN_C) and one global enable (EN). The brushless DC motor has
three hall-effect sensors (A, B, C) that report the rotor's angular position.
FAULT and KILL are safety inputs that immediately shut down the driver.

### Pin Map (STM32U5A5VJT)

| Signal   | Pin  | Direction | Notes                        |
|----------|------|-----------|------------------------------|
| PWM A    | PD12 | Output    | TIM4_CH1, L6234 IN_A        |
| PWM B    | PD13 | Output    | TIM4_CH2, L6234 IN_B        |
| PWM C    | PD14 | Output    | TIM4_CH3, L6234 IN_C        |
| EN       | PC9  | Output    | L6234 global enable          |
| Hall A   | PD0  | Input     | Pull-up enabled              |
| Hall B   | PD1  | Input     | Pull-up enabled              |
| Hall C   | PD3  | Input     | Pull-up enabled              |
| FAULT    | PE0  | Input     | Active high, from L6234      |
| KILL     | PE4  | Input     | Active high, external kill   |
| LED0     | PC13 | Output    | Status LED                   |
| LED1     | PD15 | Output    | Status LED                   |

## Overlay

The overlay does three things:

1. **Enables TIM4 as a PWM peripheral** with three channels routed to
   PD12/PD13/PD14 via STM32 alternate function AF2. `st,prescaler = <0>`
   means no prescaling — the timer runs at full bus clock speed for maximum
   PWM resolution.

2. **Declares GPIOs** for hall sensors, fault, kill, and enable under the
   `zephyr,user` node so the C code can grab them with devicetree macros.

3. **Defines LEDs** on PC13/PD15/PE14 under a `gpio-leds` compatible node.

## How Six-Step Hall Commutation Works

A BLDC motor with 3 hall sensors produces a 3-bit code (0-7) that cycles
through 6 valid states as the rotor turns. Each state tells you which
60-degree sector the rotor is in. Two states (000 and 111) are
invalid/impossible.

The lookup table in `main.c` maps each hall code to which phase should be
driven:

| Hall Code (CBA) | Driven Phase |
|------------------|--------------|
| 001              | A            |
| 010              | B            |
| 011              | A            |
| 100              | C            |
| 101              | C            |
| 110              | B            |
| 000, 111         | Invalid      |

## The Control Loop

Every 1 ms the main loop executes:

1. **Check safety first** — read FAULT (PE0) and KILL (PE4). If either is
   active, immediately pull EN low and zero all PWM outputs. The motor
   coasts to a stop.

2. **If safe** — read the 3 hall pins into a 3-bit code, assert EN high,
   then call `apply_commutation()`.

3. **`apply_commutation()`** turns off all three PWM channels first, then
   turns on only the one phase that matches the current hall position at
   25% duty cycle.

## PWM Output

`set_pwm_percent()` converts a percentage (0-100) into a pulse width in
nanoseconds. At 20 kHz the period is 50 us. At 25% duty the pulse is
12.5 us high, 37.5 us low.

## What the L6234 Does With This

The L6234 has three half-bridges. Each IN pin controls the high-side FET of
one phase. EN gates all three bridges globally. At any moment:

- One phase is being PWM'd (current flows in)
- The other two are off (current returns through body diodes / freewheeling)
- The rotor advances, hall code changes, and the code switches to the next phase

This creates a rotating magnetic field that keeps the rotor spinning.

## Tuning on Hardware

- **`DUTY_PERCENT`** — start at 25%, increase for more torque/speed. Go too
  high too fast and you will trip the L6234's thermal shutdown.
- **Hall table** — the mapping depends on your motor's winding order and hall
  sensor placement. If the motor vibrates instead of spinning, rearrange the
  table entries. Swap two phases or rotate the table by one step.
- **Direction** — to reverse, invert the hall-to-phase mapping.

## Build

```
west build -p always -b ut_core ut-core\app\bldcHallL6234
```

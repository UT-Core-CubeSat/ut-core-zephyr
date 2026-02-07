/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * -----------------------------------------------------------------------------
 * CUBESAT TEMPERATURE TELEMETRY - I2C temp sensor reader
 * -----------------------------------------------------------------------------
 * Reads multiple I2C temperature sensors (e.g. TMP102/TMP75 style) and fills
 * a telemetry struct. Used for thermal monitoring on the satellite (e.g.
 * battery, payload, OBC, solar panels). All temps are stored in Q4 fixed-point
 * for precision without floating point.
 * -----------------------------------------------------------------------------
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>
#include <stdbool.h>

// ===================== Devicetree / I2C config ===================== 

#define I2C_BUS_NODE DT_NODELABEL(i2c1)
static const struct device *const i2c_bus = DEVICE_DT_GET(I2C_BUS_NODE);

// Temperature register address on the sensor.
// TI TMP275 (and compatible) parts use register 0x00 for the temperature value. 
// If you use a different sensor family, check its datasheet and change this.
#define TEMP_REG 0x00

// I2C 7-bit slave addresses for our temp sensors.
// 0x48..0x4D are the address range for the TMP275 (pins A0/A1/A2 set to GND/VCC combinations). 
// Each physical sensor on the bus must have a unique address. 
// Order here defines index 0..5 in the telemetry array.
static const uint8_t temp_addrs[] = { 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D };
#define NUM_SENSORS ((uint8_t)(sizeof(temp_addrs) / sizeof(temp_addrs[0])))

// ===================== Telemetry Types ===================== 

// Single temperature sample from one sensor.
struct temp_sample {
	uint8_t addr;     // I2C 7-bit address we read from (for logging/debug) 
	int16_t temp_q4;  // Temperature in Q4 format (see below); 0 if read failed
	int status;       // 0 = read OK, non-zero = I2C error code (e.g. -EIO)
};

// Q4 FORMAT (what the fuck is Q4?)
// --------------------------------
// Q4 is "fixed-point" with 4 fractional bits: value = degC * 16.
// Examples:  0 degC -> 0,  25.0 degC -> 400,  25.5 degC -> 408.
// So: real_degC = temp_q4 / 16.0  (or (temp_q4 + 8) / 16 for rounded integer).
// We use this so we can store fractional degrees without using float (handy
// on small MCUs and for deterministic telemetry).

// Full telemetry snapshot: timestamp + one sample per sensor.
struct temp_telemetry {
	uint32_t t_ms;                 	    // Time of sample: kernel uptime in milliseconds (probably going to change to RTC time? idk yet)
	struct temp_sample s[NUM_SENSORS];  // One entry per sensor, same order as temp_addrs[]
};

// ===================== Small Helpers ===================== 

// Print a Q4 temperature as an integer degC (rounded).
// Q4 -> degC: divide by 16; +8 before /16 gives rounding to nearest integer.
static void print_temp_q4(int16_t t_q4) {
	int32_t temp_c = (t_q4 + 8) / 16;  // +8 for rounding
	printk("%ld", (long)temp_c);
}

// ===================== Read-all-temps: the main function =====================
//
// temp_telemetry_read_all
// -----------------------
// Reads all configured I2C temperature sensors and fills a telemetry struct.
//
// INPUTS:
//   bus  - Zephyr I2C device (e.g. i2c1). Must be valid and ready.
//   out  - Pointer to a struct temp_telemetry. Must not be NULL. This is
//          where we write the timestamp and one temp_sample per sensor.
//          The struct lives in RAM (e.g. in main() as "struct temp_telemetry telem").
//          We fill that one place in memory; saving to eMMC / downlink is a separate
//          step (copy this struct into a buffer or log, then write to storage).
//
// OUTPUTS:
//   out->t_ms       - Set to current uptime in ms (when we took the snapshot).
//   out->s[i]       - For each sensor i:
//                       addr   = I2C address we used
//                       temp_q4= temperature in Q4 (degC * 16), or 0 on error
//                       status = 0 on success, or I2C error code on failure
//
// RETURN VALUE:
//   Number of sensors that read successfully (status == 0). So 0..NUM_SENSORS.
//
// FLOW (step by step):
//   1. Validate bus and out; if either is NULL, return 0.
//   2. Record snapshot time in out->t_ms (uptime in milliseconds).
//   3. For each sensor index i (0 to NUM_SENSORS-1):
//      a. Set reg = TEMP_REG (0x00) — the register we want to read.
//      b. Store this sensor’s I2C address in out->s[i].addr.
//      c. I2C transaction: write the register address (1 byte), then read
//         2 bytes from that register into buf (MSB first, typical for these
//         sensors).
//      d. Save the I2C return code in out->s[i].status.
//      e. If status == 0 (success):
//         - Combine buf[0], buf[1] into a 16-bit signed raw value (big-endian).
//         - These sensors put 12-bit temperature left-justified in 16 bits,
//           so we right-shift by 4 to get Q4 (degC * 16). Store in out->s[i].temp_q4.
//         - Increment the success count.
//      f. If status != 0: leave temp_q4 as 0 (or whatever we want for "no data").
//   4. Return the number of sensors that succeeded.
//
// SENSOR DATA FORMAT (TMP102/TMP75 style):
//   The 2-byte register is 12-bit signed, left-justified in 16 bits:
//   [15:4] = temperature (0.0625 degC per LSB); [3:0] = sub-LSB (often ignored).
//   So: raw_16bit >> 4 gives units of 0.0625 degC = 1/16 degC, i.e. Q4.
//
// THREADS / SCHEDULED TASKS (no main() required):
//   This function doesn't care who calls it. You can call it from a dedicated
//   "thermal" or "telemetry" thread that runs on a timer (e.g. every 500 ms).
//   The struct can live on that thread's stack, or in a shared buffer—caller
//   decides. After filling 'out', that thread typically:
//     - Puts the struct (or a copy) on a queue for a logger thread → eMMC, or
//     - Hands it to a CAN message builder for downlink, or
//     - Copies into a shared "latest telemetry" snapshot (use a mutex if other
//       threads read it). Only one thread should be doing I2C to the temp
//       sensors at a time (this function doesn't take a mutex; the caller's
//       design should ensure that).
//
static int temp_telemetry_read_all(const struct device *bus, struct temp_telemetry *out) {
	// Guard: don’t dereference null; return 0 successful reads
	if (out == NULL || bus == NULL) {
		return 0;
	}

	// When did we take this snapshot? (for ground station / logs)
	out->t_ms = (uint32_t)k_uptime_get_32();

	int ok = 0;  // Init count of sensors that read successfully to 0 at first

	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		uint8_t reg = TEMP_REG;   // Register 0x00 = temperature 
		uint8_t buf[2] = {0};     // MSB, LSB from sensor 

		// Remember which address we’re polling (for debugging / labels)
		// out->s[i].addr means: pointer out -> struct member s -> element [i] -> field addr
		out->s[i].addr = temp_addrs[i];

		// Single I2C transaction: write 1 byte (reg address), then read 2 bytes.
		// Device uses 7-bit address temp_addrs[i]; we get back the temp register.
		int ret = i2c_write_read(bus, temp_addrs[i], &reg, 1, buf, 2);
		out->s[i].status = ret;

		if (ret == 0) {
			// Sensor format: 12-bit signed temp, left-justified in 16 bits.
			// Big-endian: buf[0]=MSB, buf[1]=LSB. Cast to int16_t for sign.
			// Right-shift by 4 gives Q4 (value = degC * 16). This is the only
			// place that’s sensor-specific; different chip => change this block.
			int16_t raw = (int16_t)((buf[0] << 8) | buf[1]);
			out->s[i].temp_q4 = (int16_t)(raw >> 4);
			ok++;
		} else {
			// Read failed: leave temp at 0 so downstream knows we have no data
			out->s[i].temp_q4 = 0;
		}
	}

	return ok;
}


// Print one telemetry snapshot to console: timestamp then each sensor as
// "0x48:25C  0x49:26C  ..." or "0x48:ERR(-5)" on I2C failure.
static void temp_telemetry_print(const struct temp_telemetry *t) {
	printk("t=%lu ms | ", (unsigned long)t->t_ms);

	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		printk("0x%02X:", t->s[i].addr);

		if (t->s[i].status == 0) {
			print_temp_q4(t->s[i].temp_q4);
			printk("C");
		} else {
			printk("ERR(%d)", t->s[i].status);
		}

		if (i + 1 < NUM_SENSORS) {
			printk("  ");
		}
	}

	printk("\n");
}

// ===================== Main =====================
int main(void) {
	if (!device_is_ready(i2c_bus)) {
		printk("I2C bus not ready\n");
		return 0;
	}

	printk("Temp poller: reading %d sensors on i2c1 (0x48..0x4D)\n", NUM_SENSORS);

	struct temp_telemetry telem;

	while (1) {
		(void)temp_telemetry_read_all(i2c_bus, &telem);
		temp_telemetry_print(&telem);
		k_msleep(500);
	}

	return 0;
}

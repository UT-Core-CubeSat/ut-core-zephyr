/**
 * @file rfm26w.h
 * @brief Public API of the RFM26W radio driver (HopeRF RFM26W, Si446x-class)
 * @ingroup comms
 *
 * The driver moves fixed-size blocks. The block length comes from the devicetree
 * (`block-length`, 223 bytes). Framing above the block (CRC, Space Packet, security) is the
 * application's job. The radio is half-duplex and calls are serialized by the driver.
 *
 * Error codes used by the functions below: -EIO (SPI or chip error), -ETIMEDOUT, -EBUSY,
 * -EMSGSIZE (wrong block length), -ESHUTDOWN (radio shut down), -EINVAL, -ENOSYS (operation not
 * provided by the driver) and -ENOTSUP (not implemented yet).
 */
#ifndef UT_DRIVERS_RFM26W_H_
#define UT_DRIVERS_RFM26W_H_

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

/** Selects a generated property table (data rate, deviation, channel bandwidth). */
enum rfm26w_profile {
	RFM26W_PROFILE_GFSK_9K6,  /**< 2GFSK 9.6 kb/s, working value, undecided */
	RFM26W_PROFILE_GFSK_19K2, /**< 2GFSK 19.2 kb/s */
	RFM26W_PROFILE_GFSK_50K,  /**< 2GFSK 50 kb/s */
};

/** Radio settings applied by rfm26w_configure(). */
struct rfm26w_config {
	uint32_t frequency_hz;      /**< carrier frequency, limited by the module's band variant */
	int8_t tx_power_dbm;        /**< transmit power */
	enum rfm26w_profile profile; /**< data rate profile */
};

/** Identity and fixed settings, read from the chip and the devicetree. */
struct rfm26w_info {
	uint16_t part;      /**< PART_INFO part number */
	uint8_t chip_rev;   /**< PART_INFO chip revision */
	uint16_t block_len; /**< fixed block length in bytes */
	enum rfm26w_profile profile; /**< active profile */
};

/** Per-block receive information. */
struct rfm26w_rx_info {
	int16_t rssi_dbm10; /**< latched RSSI, tenths of dBm (conversion formula TBD) */
	uint32_t uptime_ms; /**< k_uptime_get_32() when the block completed */
};

/** Counters since start, for loss measurements. */
struct rfm26w_stats {
	uint32_t tx_blocks;    /**< blocks transmitted */
	uint32_t rx_blocks;    /**< blocks received */
	uint32_t rx_overruns;  /**< RX FIFO overflowed, block lost */
	uint32_t tx_underruns; /**< TX FIFO ran empty, block truncated */
	uint32_t cts_timeouts; /**< chip did not report clear-to-send in time */
	uint32_t spi_errors;   /**< SPI transfers that failed */
};

/**
 * @brief Receive callback, called from the driver's work context (not an ISR, do not block).
 * @param dev the radio
 * @param block received block, valid only during the call
 * @param len block length
 * @param info RSSI and timestamp
 * @param user_data pointer given to rfm26w_recv_async()
 */
typedef void (*rfm26w_rx_cb)(const struct device *dev, const uint8_t *block, size_t len,
			     const struct rfm26w_rx_info *info, void *user_data);

/** @cond INTERNAL_HIDDEN */
struct rfm26w_driver_api {
	int (*configure)(const struct device *dev, const struct rfm26w_config *cfg);
	int (*get_info)(const struct device *dev, struct rfm26w_info *info);
	int (*send)(const struct device *dev, const uint8_t *block, size_t len,
		    k_timeout_t timeout);
	int (*recv)(const struct device *dev, uint8_t *block, size_t len,
		    struct rfm26w_rx_info *info, k_timeout_t timeout);
	int (*recv_async)(const struct device *dev, rfm26w_rx_cb cb, void *user_data);
	int (*set_tx_power)(const struct device *dev, int8_t dbm);
	int (*read_rssi)(const struct device *dev, int16_t *rssi_dbm10);
	int (*get_stats)(const struct device *dev, struct rfm26w_stats *stats);
	int (*shutdown)(const struct device *dev);
	int (*start)(const struct device *dev);
	int (*test_cw)(const struct device *dev, uint32_t frequency_hz, int8_t dbm,
		       uint32_t duration_ms);
};
/** @endcond */

/**
 * @brief Apply frequency, TX power and profile. Call after rfm26w_start().
 *
 * If the radio is listening it is stopped and started again around the change, so a block that is
 * arriving at that moment is lost.
 * @retval 0 success
 * @retval -EBUSY a block is being transmitted
 * @retval -ESHUTDOWN the radio is shut down
 * @return another negative errno on failure
 */
static inline int rfm26w_configure(const struct device *dev, const struct rfm26w_config *cfg)
{
	const struct rfm26w_driver_api *api = (const struct rfm26w_driver_api *)dev->api;

	return (api->configure == NULL) ? -ENOSYS : api->configure(dev, cfg);
}

/**
 * @brief Read part number, revision, block length and profile.
 * @return 0 on success or a negative errno
 */
static inline int rfm26w_get_info(const struct device *dev, struct rfm26w_info *info)
{
	const struct rfm26w_driver_api *api = (const struct rfm26w_driver_api *)dev->api;

	return (api->get_info == NULL) ? -ENOSYS : api->get_info(dev, info);
}

/**
 * @brief Transmit one block, blocking until the packet is sent or the timeout expires.
 * @param dev the radio
 * @param block block to send
 * @param len must equal the block length, else -EMSGSIZE
 * @param timeout how long to wait in total
 * @retval 0 the block was sent
 * @retval -EBUSY a block is being received and did not finish within the timeout, or another
 *         rfm26w_send() is in progress and did not finish within the timeout
 * @retval -ETIMEDOUT the transmission did not finish within the timeout (it is aborted)
 * @retval -EIO SPI error, or the TX FIFO ran empty and the block was truncated
 * @retval -ESHUTDOWN the radio is shut down
 * @note The timeout must cover the whole transmission. The air time of a block is
 *       (preamble + sync + block length) * 8 / data rate, about 200 ms for 223 bytes at 9.6 kb/s.
 * @note If receive is armed, it resumes automatically after the transmission.
 * @note Do not call this from the receive callback: the callback runs in the thread that
 *       completes the transmission, so the call would wait on itself.
 */
static inline int rfm26w_send(const struct device *dev, const uint8_t *block, size_t len,
			      k_timeout_t timeout)
{
	const struct rfm26w_driver_api *api = (const struct rfm26w_driver_api *)dev->api;

	return (api->send == NULL) ? -ENOSYS : api->send(dev, block, len, timeout);
}

/**
 * @brief Receive one block, blocking until a block arrives or the timeout expires.
 *
 * The first call makes the radio listen, and it keeps listening afterwards, so call it again
 * promptly. One block is kept between calls. If a second block arrives before this is called
 * again, the second one is dropped and counted in rfm26w_stats.rx_overruns. Use one reader.
 * @param dev the radio
 * @param block buffer for the block
 * @param len buffer size, must be at least the block length
 * @param info receive information, may be NULL
 * @param timeout how long to wait
 * @retval 0 a block was received
 * @retval -EAGAIN timeout with no block
 * @retval -EBUSY asynchronous receive is armed (use one or the other)
 * @retval -EMSGSIZE the buffer is smaller than the block length
 * @retval -ESHUTDOWN the radio is shut down
 */
static inline int rfm26w_recv(const struct device *dev, uint8_t *block, size_t len,
			      struct rfm26w_rx_info *info, k_timeout_t timeout)
{
	const struct rfm26w_driver_api *api = (const struct rfm26w_driver_api *)dev->api;

	return (api->recv == NULL) ? -ENOSYS : api->recv(dev, block, len, info, timeout);
}

/**
 * @brief Arm continuous receive with a callback.
 *
 * The callback runs in the radio's own work queue thread (stack size
 * CONFIG_RFM26W_WORKQ_STACK_SIZE) while the next block may already be arriving, so it must return
 * quickly, copy what it needs, and hand the block to another thread. It must not call
 * rfm26w_send().
 * @param dev the radio
 * @param cb callback for each received block, NULL cancels receive (the radio stops listening)
 * @param user_data passed to the callback
 * @retval 0 success
 * @retval -EBUSY a callback is already armed, cancel it with NULL first
 * @retval -ESHUTDOWN the radio is shut down
 */
static inline int rfm26w_recv_async(const struct device *dev, rfm26w_rx_cb cb, void *user_data)
{
	const struct rfm26w_driver_api *api = (const struct rfm26w_driver_api *)dev->api;

	return (api->recv_async == NULL) ? -ENOSYS : api->recv_async(dev, cb, user_data);
}

/**
 * @brief Set TX power in dBm.
 * @retval -EINVAL the value is outside the module's range
 */
static inline int rfm26w_set_tx_power(const struct device *dev, int8_t dbm)
{
	const struct rfm26w_driver_api *api = (const struct rfm26w_driver_api *)dev->api;

	return (api->set_tx_power == NULL) ? -ENOSYS : api->set_tx_power(dev, dbm);
}

/**
 * @brief Instantaneous channel RSSI, tenths of dBm. Only meaningful while the radio is listening.
 *
 * The dBm conversion is unverified (PLANNED.md open item 23).
 * @return 0 on success or a negative errno
 */
static inline int rfm26w_read_rssi(const struct device *dev, int16_t *rssi_dbm10)
{
	const struct rfm26w_driver_api *api = (const struct rfm26w_driver_api *)dev->api;

	return (api->read_rssi == NULL) ? -ENOSYS : api->read_rssi(dev, rssi_dbm10);
}

/**
 * @brief Counters since the last start.
 * @return 0 on success or a negative errno
 */
static inline int rfm26w_get_stats(const struct device *dev, struct rfm26w_stats *stats)
{
	const struct rfm26w_driver_api *api = (const struct rfm26w_driver_api *)dev->api;

	return (api->get_stats == NULL) ? -ENOSYS : api->get_stats(dev, stats);
}

/**
 * @brief Shut the radio down (SDN high).
 *
 * Configuration is lost and pending calls return -ESHUTDOWN. Also the hook for UNP12-91
 * (cease transmission).
 * @return 0 on success or a negative errno
 */
static inline int rfm26w_shutdown(const struct device *dev)
{
	const struct rfm26w_driver_api *api = (const struct rfm26w_driver_api *)dev->api;

	return (api->shutdown == NULL) ? -ENOSYS : api->shutdown(dev);
}

/**
 * @brief Leave shutdown (SDN low), wait for the chip and load the property table.
 *
 * Powers the chip up with the reference clock from the devicetree (`xo-frequency-hz`, `xo-tcxo`),
 * checks it answers PART_INFO, loads the profile table and clears interrupts and FIFOs. Does nothing
 * if the radio is already started. On failure the radio is put back in shutdown.
 *
 * @warning The reference clock (30 MHz crystal) is an ASSUMPTION, not confirmed for the module. The
 *          chip cannot detect a wrong value, so the carrier frequency and data rate would simply be
 *          wrong. If it changes, change the devicetree and regenerate the profile (PLANNED.md 5.4).
 *
 * Call rfm26w_configure() afterwards.
 * @retval 0 started
 * @retval -ETIMEDOUT the chip never reported clear-to-send (absent, unpowered or wired wrong)
 * @retval -EIO SPI error, or a profile property did not read back
 * @retval -ENOTSUP no profile table exists for the default profile
 */
static inline int rfm26w_start(const struct device *dev)
{
	const struct rfm26w_driver_api *api = (const struct rfm26w_driver_api *)dev->api;

	return (api->start == NULL) ? -ENOSYS : api->start(dev);
}

/**
 * @brief Continuous-wave test transmission.
 * @param dev the radio
 * @param frequency_hz carrier frequency
 * @param dbm transmit power
 * @param duration_ms how long to transmit
 * @retval -ENOSYS the driver does not provide a test mode
 */
static inline int rfm26w_test_cw(const struct device *dev, uint32_t frequency_hz, int8_t dbm,
				 uint32_t duration_ms)
{
	const struct rfm26w_driver_api *api = (const struct rfm26w_driver_api *)dev->api;

	return (api->test_cw == NULL) ? -ENOSYS : api->test_cw(dev, frequency_hz, dbm, duration_ms);
}

#endif /* UT_DRIVERS_RFM26W_H_ */

/**
 * @file can_proto.c
 * @ingroup common
 * @brief Shared UT-CORE CAN bus helpers: frame decode, simple frame send,
 *        and common transceiver/controller bring-up.
 *
 * Plans to Implement the CAN handling logic previously duplicated across CDH,
 * EPS, GNSS, MOTOR, and SOLAR (frame decoding, single-opcode frame
 * construction, TCAN3403 wakeup, and CAN controller/filter setup), so a
 * protocol-level fix only needs to be made in one place.
 *
 * @todo move shared can source code to this file.
 */


#include "can_proto.h"

/**
 * @brief A test function that does nothing
 * This function is to make sure this file is building properly.
 *
 * @todo remove this function when actual code is added to this file.
 */
void test(void)
{
    (void)0;
}

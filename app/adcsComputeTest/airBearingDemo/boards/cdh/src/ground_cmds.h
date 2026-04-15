#pragma once
#include <stdint.h>
#include <zephyr/drivers/can.h>
#ifdef __cplusplus
extern "C" {
    #endif


    /* Decode a ground command in a CAN frame; return 0 if handled */
    int ground_cmd_handle(const struct can_frame *f);


    #ifdef __cplusplus
}
#endif
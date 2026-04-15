#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
    #endif


    typedef enum {
        EV_CAN_RX = 1,
        EV_GROUND_CMD,
        EV_SOH_TICK,
        EV_SCHED_TICK,
        EV_MODE_TICK,
        EV_WATCHDOG_TICK,
    } cdh_event_type_t;


    struct cdh_event {
        cdh_event_type_t type;
        uint32_t arg0;
        uint32_t arg1;
    };


    #ifdef __cplusplus
}
#endif
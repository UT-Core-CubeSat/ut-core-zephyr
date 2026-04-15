#pragma once
#ifdef __cplusplus
extern "C" {
    #endif


    enum cdh_mode {
        CDH_MODE_STANDBY = 0,
        CDH_MODE_LOW_POWER,
        CDH_MODE_SLEEP,
        CDH_MODE_DEEP_SLEEP,
        CDH_MODE_OPERATIONAL
    };


    #ifdef __cplusplus
}
#endif
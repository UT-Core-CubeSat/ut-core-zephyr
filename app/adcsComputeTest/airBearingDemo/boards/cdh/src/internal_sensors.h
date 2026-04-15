#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
    #endif


    struct internal_readings {
        int16_t mcu_temp_c10; /* 10x degC */
        uint16_t vbus_mv;
    };


    int internal_sensors_init(void);
    int internal_sensors_read(struct internal_readings *out);


    #ifdef __cplusplus
}
#endif
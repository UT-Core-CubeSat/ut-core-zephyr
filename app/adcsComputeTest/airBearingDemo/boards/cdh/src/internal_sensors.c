#include "internal_sensors.h"
int internal_sensors_init(void) { 
    return 0;
}
int internal_sensors_read(struct internal_readings *out) {
    if (!out) return -1;
    /* TODO: hook to real ADC/sensor drivers */
    out->mcu_temp_c10 = 250; /* 25.0 C */
    out->vbus_mv = 5000; /* 5.0 V */
    return 0;
}
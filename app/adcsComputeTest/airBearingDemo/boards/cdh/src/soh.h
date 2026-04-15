#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
    #endif


    struct soh_packet {
        /* add fields as needed */
        uint16_t bus_v_mv;
        int16_t mcu_temp_c10; /* 10x degC */
        uint16_t rx_count;
        uint16_t tx_count;
        uint8_t mode;
    };


    void soh_init(void);
    void soh_update_from_internal_sensors(struct soh_packet *p);
    void soh_update_comm_stats(struct soh_packet *p, uint16_t rx, uint16_t tx);


    #ifdef __cplusplus
}
#endif
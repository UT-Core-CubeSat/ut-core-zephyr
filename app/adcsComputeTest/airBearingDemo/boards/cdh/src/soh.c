#include "soh.h"

void soh_init(void) {
    /* nothing to do yet */
}

void soh_update_from_internal_sensors(struct soh_packet *p) {
    (void)p; /* fill from internal_sensors later */
}

void soh_update_comm_stats(struct soh_packet *p, uint16_t rx, uint16_t tx) {
    if (!p) return;
    p->rx_count = rx;
    p->tx_count = tx;
}
#include <zephyr/logging/log.h>
#include "cdh_can_proto.h"
#include "ground_cmds.h"
LOG_MODULE_REGISTER(ground_cmds, LOG_LEVEL_INF);


int ground_cmd_handle(const struct can_frame *f) {
    if (!f) 
        return -1;

    const uint8_t op = f->data[1];
    
    switch (op) {
        case OP_GROUND_CMD:
        LOG_INF("Ground command: p2=%u p3=%u", f->data[2], f->data[3]);
        /* TODO: decode and trigger actions */
        return 0;
        default:
        return -2;
    }
}
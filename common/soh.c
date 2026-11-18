/**
 * @file soh.c
  * @ingroup common

 * @brief Pack/unpack for the shared SOH CAN payloads.
 *
 * Byte order: BIG-endian - match can-proto.h, though 
 */
#include "soh.h"

/* ------------------------------------------------------------------ */
/* Byte-order helpers (private to this file)                           */
/* ------------------------------------------------------------------ */

/** Write a 16-bit value into p[0..1], most significant byte first. */
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v);
}

/** Write a 32-bit value into p[0..3], most significant byte first. */
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

/** Read a 16-bit value from p[0..1], most significant byte first. */
static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) |
                      ((uint16_t)p[1]));
}

/** Read a 32-bit value from p[0..3], most significant byte first. */
static uint32_t get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |
           ((uint32_t)p[3]);
}

/* ------------------------------------------------------------------ */
/* SOH Core                                                            */
/* ------------------------------------------------------------------ */

void soh_pack_core(const soh_core_t *in, uint8_t out[SOH_FRAME_LEN])
{
    put_u32(&out[0], in->uptime_s);          /* bytes 0-3 */
    put_u16(&out[4], in->vdd_mv);            /* bytes 4-5 */
    out[6] = (uint8_t)in->temp_c;            /* byte 6    */
    out[7] = (uint8_t)((SOH_SCHEMA_VERSION << 4) |
                       ((uint8_t)in->reset_cause & 0x0FU)); /* byte 7 */
}

bool soh_unpack_core(const uint8_t *in, size_t len, soh_core_t *out)
{
    if (in == NULL || out == NULL || len != SOH_FRAME_LEN) {
        return false;
    }

    /* Validate everything before writing to *out, so a rejected
     * frame never leaves *out half-updated. */
    uint8_t version = (uint8_t)(in[7] >> 4);
    uint8_t reset   = (uint8_t)(in[7] & 0x0FU);

    if (version != SOH_SCHEMA_VERSION) {
        return false;
    }
    if (reset >= (uint8_t)SOH_RESET_COUNT) {
        return false;
    }

    out->uptime_s    = get_u32(&in[0]);
    out->vdd_mv      = get_u16(&in[4]);
    out->temp_c      = (int8_t)in[6];        /* two's complement, e.g. 0xFB -> -5 */
    out->version     = version;
    out->reset_cause = (soh_reset_cause_t)reset;
    return true;
}

/* ------------------------------------------------------------------ */
/* SOH CAN Stats                                                       */
/* ------------------------------------------------------------------ */

void soh_pack_can(const soh_can_t *in, uint8_t out[SOH_FRAME_LEN])
{
    out[0] = in->tec;                        /* byte 0    */
    out[1] = in->rec;                        /* byte 1    */
    out[2] = in->can_flags;                  /* byte 2    */
    out[3] = in->bus_off_count;              /* byte 3    */
    put_u16(&out[4], in->error_irq_count);   /* bytes 4-5 */
    put_u16(&out[6], in->rx_overrun_count);  /* bytes 6-7 */
}

bool soh_unpack_can(const uint8_t *in, size_t len, soh_can_t *out)
{
    if (in == NULL || out == NULL || len != SOH_FRAME_LEN) {
        return false;
    }

    out->tec              = in[0];
    out->rec              = in[1];
    out->can_flags        = in[2];
    out->bus_off_count    = in[3];
    out->error_irq_count  = get_u16(&in[4]);
    out->rx_overrun_count = get_u16(&in[6]);
    return true;
}
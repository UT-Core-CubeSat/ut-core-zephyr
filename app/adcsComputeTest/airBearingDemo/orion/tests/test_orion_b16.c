/**
 * @file test_orion_b16.c
 * @brief Unit tests for the Orion B16 parser and command builder.
 *
 * These tests are PLATFORM-INDEPENDENT — they test the pure-C parser and
 * command builder without any Zephyr or hardware dependencies.
 *
 * Build on host:
 *   gcc -I../drivers/orion_b16 -o test_orion test_orion_b16.c \
 *       ../drivers/orion_b16/orion_b16_parser.c \
 *       ../drivers/orion_b16/orion_b16_commands.c -lm
 *   ./test_orion
 *
 * Or integrate with Zephyr's ztest framework for on-target testing.
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "orion_b16_protocol.h"
#include "orion_b16_messages.h"
#include "orion_b16_parser.h"
#include "orion_b16_commands.h"

/* ──────────────── Helper: Feed a frame into the parser ──────────── */

static int feed_frame(orion_parser_t *p, const uint8_t *data, size_t len,
                      orion_frame_t *out)
{
    for (size_t i = 0; i < len; i++) {
        orion_parse_result_t res = orion_parser_feed_byte(p, data[i], out);
        if (res == ORION_PARSE_FRAME_READY) {
            return 0;  /* success */
        }
        if (res == ORION_PARSE_ERROR) {
            return -1;
        }
    }
    return -2;  /* incomplete — no frame found */
}

/* ──────────────── Test: XOR Checksum ────────────────────────────── */

static void test_xor_checksum(void)
{
    printf("test_xor_checksum... ");

    uint8_t data1[] = { 0x02, 0x00 };  /* Query SW Version payload */
    assert(orion_xor_checksum(data1, 2) == 0x02);

    uint8_t data2[] = { 0x09, 0x02, 0x00 };  /* CFG Binary Mode */
    assert(orion_xor_checksum(data2, 3) == (0x09 ^ 0x02 ^ 0x00));

    uint8_t data3[] = { 0x83, 0x09 };  /* ACK for 0x09 */
    assert(orion_xor_checksum(data3, 2) == (0x83 ^ 0x09));

    printf("PASS\n");
}

/* ──────────────── Test: Command Builder ─────────────────────────── */

static void test_command_builder(void)
{
    printf("test_command_builder... ");

    uint8_t buf[64];

    /* Query SW Version: should produce [A0 A1 00 02 02 00 02 0D 0A] */
    int len = orion_cmd_query_sw_version(buf, sizeof(buf), 0x00);
    assert(len == 9);
    assert(buf[0] == 0xA0 && buf[1] == 0xA1);       /* SOF */
    assert(buf[2] == 0x00 && buf[3] == 0x02);        /* PL = 2 */
    assert(buf[4] == 0x02 && buf[5] == 0x00);        /* msg_id + body */
    assert(buf[6] == 0x02);                           /* CS = 0x02 ^ 0x00 */
    assert(buf[7] == 0x0D && buf[8] == 0x0A);        /* EOF */

    /* Configure Binary mode */
    len = orion_cmd_cfg_msg_type(buf, sizeof(buf), ORION_MSG_TYPE_BINARY, ORION_ATTR_SRAM_ONLY);
    assert(len == 10);
    assert(buf[4] == 0x09);                           /* msg_id */
    assert(buf[5] == 0x02);                           /* type = Binary */
    assert(buf[6] == 0x00);                           /* attr = SRAM */

    /* Query (no body) */
    len = orion_cmd_query_update_rate(buf, sizeof(buf));
    assert(len == 8);  /* 7 overhead + 1 payload (just msg_id) */
    assert(buf[4] == 0x10);

    /* Buffer overflow protection */
    len = orion_cmd_query_sw_version(buf, 5, 0x00);  /* too small */
    assert(len == -1);

    printf("PASS\n");
}

/* ──────────────── Test: Parser Round-Trip ────────────────────────── */

static void test_parser_roundtrip(void)
{
    printf("test_parser_roundtrip... ");

    orion_parser_t parser;
    orion_parser_init(&parser);

    /* Build a command frame, then parse it back */
    uint8_t cmd_buf[32];
    int cmd_len = orion_cmd_query_sw_version(cmd_buf, sizeof(cmd_buf), 0x00);
    assert(cmd_len > 0);

    orion_frame_t frame;
    int rc = feed_frame(&parser, cmd_buf, cmd_len, &frame);
    assert(rc == 0);
    assert(frame.msg_id == 0x02);
    assert(frame.body_len == 1);
    assert(frame.body[0] == 0x00);
    assert(parser.frames_ok == 1);
    assert(parser.frames_err == 0);

    printf("PASS\n");
}

/* ──────────────── Test: Navigation Data Decode ──────────────────── */

static void test_nav_data_decode(void)
{
    printf("test_nav_data_decode... ");

    /* Build a synthetic 0xA8 nav data frame */
    uint8_t body[ORION_NAV_DATA_BODY_LEN];
    memset(body, 0, sizeof(body));

    /* fix=3D, sv=12 */
    body[0] = 0x02;
    body[1] = 12;

    /* GNSS week = 2350 */
    orion_write_be16(&body[2], 2350);

    /* TOW = 30000000 cs = 300000.00 seconds */
    orion_write_be32(&body[4], 30000000);

    /* Lat = 30.2672000 deg => 302672000 */
    orion_write_be32(&body[8], (uint32_t)302672000);

    /* Lon = -97.7431000 deg => -977431000 */
    orion_write_be32(&body[12], (uint32_t)(int32_t)(-977431000));

    /* EllAlt = 408 km = 40800000 cm */
    orion_write_be32(&body[16], 40800000);

    /* MSL alt = same */
    orion_write_be32(&body[20], 40800000);

    /* DOP */
    orion_write_be16(&body[24], 150);
    orion_write_be16(&body[26], 120);
    orion_write_be16(&body[28], 80);
    orion_write_be16(&body[30], 90);
    orion_write_be16(&body[32], 70);

    /* ECEF */
    orion_write_be32(&body[34], (uint32_t)(int32_t)(-74186900));
    orion_write_be32(&body[38], (uint32_t)(int32_t)(-515389700));
    orion_write_be32(&body[42], (uint32_t)(int32_t)(319882500));

    /* ECEF vel */
    orion_write_be32(&body[46], (uint32_t)(int32_t)(760000));
    orion_write_be32(&body[50], (uint32_t)(int32_t)(10000));
    orion_write_be32(&body[54], (uint32_t)(int32_t)(0));

    /* Build a full frame with the 0xA8 message ID */
    uint8_t frame_buf[128];
    int frame_len = orion_cmd_build_raw(frame_buf, sizeof(frame_buf),
                                        ORION_RSP_NAV_DATA,
                                        body, sizeof(body));
    assert(frame_len > 0);

    /* Parse the frame */
    orion_parser_t parser;
    orion_parser_init(&parser);
    orion_frame_t frame;
    int rc = feed_frame(&parser, frame_buf, frame_len, &frame);
    assert(rc == 0);
    assert(frame.msg_id == ORION_RSP_NAV_DATA);

    /* Decode the message */
    orion_message_t msg;
    rc = orion_decode_message(&frame, &msg);
    assert(rc == 0);
    assert(msg.msg_id == ORION_RSP_NAV_DATA);

    orion_nav_data_t *nav = &msg.nav_data;
    assert(nav->valid == true);
    assert(nav->fix_mode == 0x02);
    assert(nav->sv_count == 12);
    assert(nav->gnss_week == 2350);
    assert(nav->tow_cs == 30000000);
    assert(nav->latitude_1e7 == 302672000);
    assert(nav->longitude_1e7 == -977431000);
    assert(nav->ellipsoid_alt_cm == 40800000);
    assert(nav->msl_alt_cm == 40800000);
    assert(nav->gdop == 150);
    assert(nav->pdop == 120);
    assert(nav->hdop == 80);
    assert(nav->vdop == 90);
    assert(nav->tdop == 70);
    assert(nav->ecef_x_cm == -74186900);
    assert(nav->ecef_y_cm == -515389700);
    assert(nav->ecef_z_cm == 319882500);
    assert(nav->ecef_vx_cms == 760000);
    assert(nav->ecef_vy_cms == 10000);
    assert(nav->ecef_vz_cms == 0);

    /* Verify floating-point conversions */
    double lat = ORION_DEG_FROM_1E7(nav->latitude_1e7);
    assert(fabs(lat - 30.2672) < 0.0001);

    double lon = ORION_DEG_FROM_1E7(nav->longitude_1e7);
    assert(fabs(lon - (-97.7431)) < 0.0001);

    double alt = ORION_M_FROM_CM(nav->msl_alt_cm);
    assert(fabs(alt - 408000.0) < 1.0);

    printf("PASS\n");
}

/* ──────────────── Test: ACK/NACK Decode ─────────────────────────── */

static void test_ack_nack_decode(void)
{
    printf("test_ack_nack_decode... ");

    orion_parser_t parser;
    orion_frame_t frame;
    orion_message_t msg;

    /* Build an ACK for command 0x09 */
    uint8_t ack_body[] = { 0x09 };
    uint8_t ack_buf[16];
    int ack_len = orion_cmd_build_raw(ack_buf, sizeof(ack_buf),
                                      ORION_RSP_ACK, ack_body, 1);
    assert(ack_len > 0);

    orion_parser_init(&parser);
    int rc = feed_frame(&parser, ack_buf, ack_len, &frame);
    assert(rc == 0);
    assert(frame.msg_id == ORION_RSP_ACK);

    rc = orion_decode_message(&frame, &msg);
    assert(rc == 0);
    assert(msg.ack.is_ack == true);
    assert(msg.ack.acked_msg_id == 0x09);

    /* Build a NACK for command 0x0E */
    uint8_t nack_body[] = { 0x0E };
    uint8_t nack_buf[16];
    int nack_len = orion_cmd_build_raw(nack_buf, sizeof(nack_buf),
                                       ORION_RSP_NACK, nack_body, 1);
    assert(nack_len > 0);

    orion_parser_init(&parser);
    rc = feed_frame(&parser, nack_buf, nack_len, &frame);
    assert(rc == 0);

    rc = orion_decode_message(&frame, &msg);
    assert(rc == 0);
    assert(msg.ack.is_ack == false);
    assert(msg.ack.acked_msg_id == 0x0E);

    printf("PASS\n");
}

/* ──────────────── Test: Parser Resync After Garbage ──────────────── */

static void test_parser_resync(void)
{
    printf("test_parser_resync... ");

    orion_parser_t parser;
    orion_parser_init(&parser);

    /* Feed some garbage, then a valid frame */
    uint8_t garbage[] = { 0xFF, 0x00, 0xA0, 0x55, 0xAA, 0xBB, 0xCC };
    orion_frame_t frame;

    /* Garbage should not produce a frame */
    for (size_t i = 0; i < sizeof(garbage); i++) {
        orion_parse_result_t r = orion_parser_feed_byte(&parser, garbage[i], &frame);
        assert(r != ORION_PARSE_FRAME_READY);
    }

    /* Now feed a valid frame — parser should resync */
    uint8_t cmd_buf[16];
    int cmd_len = orion_cmd_query_update_rate(cmd_buf, sizeof(cmd_buf));
    assert(cmd_len > 0);

    int rc = feed_frame(&parser, cmd_buf, cmd_len, &frame);
    assert(rc == 0);
    assert(frame.msg_id == 0x10);

    printf("PASS\n");
}

/* ──────────────── Test: Bad Checksum Rejection ──────────────────── */

static void test_bad_checksum(void)
{
    printf("test_bad_checksum... ");

    orion_parser_t parser;
    orion_parser_init(&parser);

    /* Build a valid frame, then corrupt the checksum */
    uint8_t buf[16];
    int len = orion_cmd_query_power_mode(buf, sizeof(buf));
    assert(len > 0);

    /* Checksum is at index (len - 3): before EOF0 EOF1 */
    buf[len - 3] ^= 0xFF;  /* flip all bits */

    orion_frame_t frame;
    int rc = feed_frame(&parser, buf, len, &frame);
    assert(rc == -1);  /* should be a parse error */
    assert(parser.frames_err > 0);
    assert(parser.frames_ok == 0);

    printf("PASS\n");
}

/* ──────────────── Test: Multiple Frames in Sequence ─────────────── */

static void test_multiple_frames(void)
{
    printf("test_multiple_frames... ");

    orion_parser_t parser;
    orion_parser_init(&parser);

    /* Build three different command frames back-to-back */
    uint8_t stream[128];
    size_t stream_len = 0;

    uint8_t tmp[32];
    int len;

    len = orion_cmd_query_sw_version(tmp, sizeof(tmp), 0x00);
    memcpy(&stream[stream_len], tmp, len); stream_len += len;

    len = orion_cmd_query_power_mode(tmp, sizeof(tmp));
    memcpy(&stream[stream_len], tmp, len); stream_len += len;

    len = orion_cmd_query_update_rate(tmp, sizeof(tmp));
    memcpy(&stream[stream_len], tmp, len); stream_len += len;

    /* Feed all bytes and count frames */
    int frame_count = 0;
    orion_frame_t frame;
    for (size_t i = 0; i < stream_len; i++) {
        orion_parse_result_t r = orion_parser_feed_byte(&parser, stream[i], &frame);
        if (r == ORION_PARSE_FRAME_READY) {
            frame_count++;
        }
    }

    assert(frame_count == 3);
    assert(parser.frames_ok == 3);
    assert(parser.frames_err == 0);

    printf("PASS\n");
}

/* ──────────────── Test: SW Version Decode ────────────────────────── */

static void test_sw_version_decode(void)
{
    printf("test_sw_version_decode... ");

    /* Build a fake SW version response */
    uint8_t body[ORION_SW_VERSION_BODY_LEN];
    body[0] = 0x00;  /* system code */
    body[1] = 0x01; body[2] = 0x02; body[3] = 0x03; body[4] = 0x04;  /* kernel */
    body[5] = 0x0A; body[6] = 0x0B; body[7] = 0x0C; body[8] = 0x0D;  /* ODM */
    body[9] = 0x14; body[10] = 0x06; body[11] = 0x01; body[12] = 0x0F; /* rev */

    uint8_t buf[32];
    int len = orion_cmd_build_raw(buf, sizeof(buf), ORION_RSP_SW_VERSION,
                                  body, sizeof(body));
    assert(len > 0);

    orion_parser_t parser;
    orion_parser_init(&parser);
    orion_frame_t frame;
    int rc = feed_frame(&parser, buf, len, &frame);
    assert(rc == 0);

    orion_message_t msg;
    rc = orion_decode_message(&frame, &msg);
    assert(rc == 0);
    assert(msg.sw_version.valid == true);
    assert(msg.sw_version.sw_type == 0x00);
    assert(msg.sw_version.kernel_ver[0] == 0x01);
    assert(msg.sw_version.kernel_ver[3] == 0x04);
    assert(msg.sw_version.odm_ver[0] == 0x0A);

    printf("PASS\n");
}

/* ──────────────── Test: Big-Endian Helpers ───────────────────────── */

static void test_endian_helpers(void)
{
    printf("test_endian_helpers... ");

    uint8_t buf[4];

    /* 16-bit */
    orion_write_be16(buf, 0x1234);
    assert(buf[0] == 0x12 && buf[1] == 0x34);
    assert(orion_read_be16(buf) == 0x1234);

    /* 32-bit */
    orion_write_be32(buf, 0xDEADBEEF);
    assert(buf[0] == 0xDE && buf[1] == 0xAD && buf[2] == 0xBE && buf[3] == 0xEF);
    assert(orion_read_be32(buf) == 0xDEADBEEF);

    /* Signed 32-bit */
    int32_t neg = -123456;
    orion_write_be32(buf, (uint32_t)neg);
    assert(orion_read_be32s(buf) == neg);

    /* Signed 16-bit */
    int16_t neg16 = -1234;
    orion_write_be16(buf, (uint16_t)neg16);
    assert(orion_read_be16s(buf) == neg16);

    printf("PASS\n");
}

/* ──────────────── Main ──────────────────────────────────────────── */

int main(void)
{
    printf("=== Orion B16 Unit Tests ===\n\n");

    test_endian_helpers();
    test_xor_checksum();
    test_command_builder();
    test_parser_roundtrip();
    test_parser_resync();
    test_bad_checksum();
    test_multiple_frames();
    test_ack_nack_decode();
    test_sw_version_decode();
    test_nav_data_decode();

    printf("\nAll tests passed!\n");
    return 0;
}

/**
 * @file orion_b16_messages.h
 * @brief Decoded message structures for all supported Orion B16 response types.
 *
 * Every struct here represents a *decoded* response message with native types,
 * ready for use throughout the CubeSat software.  All coordinate values are
 * stored in their natural integer representation from the protocol; helper
 * macros convert to floating-point where convenient.
 */

#ifndef ORION_B16_MESSAGES_H
#define ORION_B16_MESSAGES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────── Conversion Helpers ────────────────────────── */

/** Convert 1e-7 degree integer to double degrees. */
#define ORION_DEG_FROM_1E7(x)   ((double)(x) * 1e-7)

/** Convert centimeter integer to meters (double). */
#define ORION_M_FROM_CM(x)      ((double)(x) * 0.01)

/** Convert cm/s integer to m/s (double). */
#define ORION_MS_FROM_CMS(x)    ((double)(x) * 0.01)

/** Convert DOP value (hundredths) to double. */
#define ORION_DOP_FROM_100(x)   ((double)(x) * 0.01)

/* ──────────────── Navigation Data (0xA8) ────────────────────────── */

/**
 * @brief Decoded Navigation Data Message (PVT).
 *
 * This is the primary output — position, velocity, and time.
 * Raw integer fields use the protocol's natural representation.
 * Use the ORION_* macros above for floating-point conversions.
 *
 * Binary body layout (after msg_id 0xA8), 59 bytes:
 *   [0]      fix_mode          uint8   (0=none, 1=2D, 2=3D, 3=3D+DGPS)
 *   [1]      sv_count          uint8   Number of SVs in fix
 *   [2..3]   gnss_week         uint16  GPS week number
 *   [4..7]   tow_cs            uint32  Time-of-week in 1/100 sec
 *   [8..11]  latitude_1e7      int32   Latitude  × 1e7 (degrees)
 *   [12..15] longitude_1e7     int32   Longitude × 1e7 (degrees)
 *   [16..19] ellipsoid_alt_cm  uint32  Ellipsoid altitude in cm
 *   [20..23] msl_alt_cm        uint32  Mean-sea-level altitude in cm
 *   [24..25] gdop              uint16  GDOP × 100
 *   [26..27] pdop              uint16  PDOP × 100
 *   [28..29] hdop              uint16  HDOP × 100
 *   [30..31] vdop              uint16  VDOP × 100
 *   [32..33] tdop              uint16  TDOP × 100
 *   [34..37] ecef_x_cm         int32   ECEF X in cm
 *   [38..41] ecef_y_cm         int32   ECEF Y in cm
 *   [42..45] ecef_z_cm         int32   ECEF Z in cm
 *   [46..49] ecef_vx_cms       int32   ECEF VX in cm/s
 *   [50..53] ecef_vy_cms       int32   ECEF VY in cm/s
 *   [54..57] ecef_vz_cms       int32   ECEF VZ in cm/s
 */
typedef struct {
    /* Status */
    uint8_t  fix_mode;          /**< 0=none, 1=2D, 2=3D, 3=3D+DGPS         */
    uint8_t  sv_count;          /**< Satellites used in fix                  */

    /* Time */
    uint16_t gnss_week;         /**< GPS week number                        */
    uint32_t tow_cs;            /**< Time-of-week in centiseconds (1/100 s) */

    /* Geodetic Position */
    int32_t  latitude_1e7;      /**< Latitude  in 1e-7 degrees              */
    int32_t  longitude_1e7;     /**< Longitude in 1e-7 degrees              */
    uint32_t ellipsoid_alt_cm;  /**< Ellipsoid altitude in centimeters      */
    uint32_t msl_alt_cm;        /**< Mean sea level altitude in centimeters */

    /* Dilution of Precision (×100) */
    uint16_t gdop;
    uint16_t pdop;
    uint16_t hdop;
    uint16_t vdop;
    uint16_t tdop;

    /* ECEF Position (centimeters) */
    int32_t  ecef_x_cm;
    int32_t  ecef_y_cm;
    int32_t  ecef_z_cm;

    /* ECEF Velocity (centimeters/sec) */
    int32_t  ecef_vx_cms;
    int32_t  ecef_vy_cms;
    int32_t  ecef_vz_cms;

    /* Validity flag — set to true if decode succeeded */
    bool     valid;
} orion_nav_data_t;

/** Expected body length (bytes after msg_id) for nav data */
#define ORION_NAV_DATA_BODY_LEN  58u

/* ──────────────── Software Version (0x80) ───────────────────────── */

/**
 * @brief Decoded Software Version response.
 *
 * Body layout (after msg_id 0x80), 14 bytes:
 *   [0]      sw_type       uint8   0=system, 1=reserved
 *   [1..4]   kernel_ver    4×uint8 Kernel version bytes
 *   [5..8]   odm_ver       4×uint8 ODM version bytes
 *   [9..12]  revision      4×uint8 Revision date (YYYY/MM/DD encoded)
 */
typedef struct {
    uint8_t  sw_type;           /**< 0 = system code                */
    uint8_t  kernel_ver[4];     /**< Kernel version (4 bytes)       */
    uint8_t  odm_ver[4];       /**< ODM version (4 bytes)          */
    uint8_t  revision[4];       /**< Revision date (4 bytes)        */
    bool     valid;
} orion_sw_version_t;

#define ORION_SW_VERSION_BODY_LEN  13u

/* ──────────────── ACK / NACK (0x83 / 0x84) ──────────────────────── */

/**
 * @brief Decoded ACK or NACK response.
 * Body: [0] = message ID being acknowledged.
 */
typedef struct {
    uint8_t  acked_msg_id;      /**< The command message ID ACK'd/NACK'd */
    bool     is_ack;            /**< true = ACK (0x83), false = NACK (0x84) */
} orion_ack_t;

/* ──────────────── Position Update Rate (0x86) ───────────────────── */

typedef struct {
    uint8_t  rate_hz;           /**< Position update rate in Hz */
    bool     valid;
} orion_update_rate_t;

/* ──────────────── GNSS Message Type (0x8C) ──────────────────────── */

typedef struct {
    uint8_t  msg_type;          /**< 0x01=NMEA, 0x02=Binary */
    bool     valid;
} orion_msg_type_t;

/* ──────────────── NMEA Talker ID (0x93) ─────────────────────────── */

typedef struct {
    uint8_t  talker_mode;       /**< 0x00=GP, 0x01=GN, 0x02=Auto */
    bool     valid;
} orion_talker_id_t;

/* ──────────────── Power Mode Status (0xB9) ──────────────────────── */

typedef struct {
    uint8_t  power_mode;        /**< 0x00=Normal, 0x01=Power Save */
    bool     valid;
} orion_power_mode_t;

/* ──────────────── GNSS Constellation (0xB3) ─────────────────────── */

typedef struct {
    uint8_t  gps_enabled;       /**< GPS constellation enabled     */
    uint8_t  glonass_enabled;   /**< GLONASS constellation enabled */
    uint8_t  galileo_enabled;   /**< Galileo constellation enabled */
    uint8_t  beidou_enabled;    /**< BeiDou constellation enabled  */
    bool     valid;
} orion_gnss_constellation_t;

/* ──────────────── DOP Mask (0xAF) ───────────────────────────────── */

typedef struct {
    uint8_t  dop_mode;          /**< 0=disable, 1=auto, 2=PDOP, 3=HDOP, 4=GDOP */
    uint16_t pdop_val;          /**< PDOP threshold × 10    */
    uint16_t hdop_val;          /**< HDOP threshold × 10    */
    uint16_t gdop_val;          /**< GDOP threshold × 10    */
    bool     valid;
} orion_dop_mask_t;

/* ──────────────── Elevation & CNR Mask (0xB0) ───────────────────── */

typedef struct {
    uint8_t  elev_mask;         /**< Elevation mask in degrees   */
    uint8_t  cnr_mask;          /**< C/N0 mask in dB-Hz          */
    bool     valid;
} orion_elev_cnr_mask_t;

/* ──────────────── Generic union for callback routing ────────────── */

/**
 * @brief Union of all decoded Orion B16 message types.
 *
 * The `msg_id` discriminates which member is valid.
 */
typedef struct {
    uint8_t msg_id;             /**< The response message ID */
    union {
        orion_nav_data_t          nav_data;       /**< 0xA8 */
        orion_sw_version_t        sw_version;     /**< 0x80 */
        orion_ack_t               ack;            /**< 0x83 / 0x84 */
        orion_update_rate_t       update_rate;    /**< 0x86 */
        orion_msg_type_t          msg_type;       /**< 0x8C */
        orion_talker_id_t         talker_id;      /**< 0x93 */
        orion_power_mode_t        power_mode;     /**< 0xB9 */
        orion_gnss_constellation_t constellation; /**< 0xB3 */
        orion_dop_mask_t          dop_mask;       /**< 0xAF */
        orion_elev_cnr_mask_t     elev_cnr_mask;  /**< 0xB0 */
    };
} orion_message_t;


#ifdef __cplusplus
}
#endif

#endif /* ORION_B16_MESSAGES_H */

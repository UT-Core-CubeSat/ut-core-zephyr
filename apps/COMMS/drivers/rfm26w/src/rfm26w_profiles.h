/**
 * @file rfm26w_profiles.h
 * @brief Lookup of the WDS-generated radio profile tables (driver internal)
 * @ingroup comms
 */
#ifndef RFM26W_PROFILES_H_
#define RFM26W_PROFILES_H_

#include <stddef.h>
#include <stdint.h>

#include <ut/drivers/rfm26w.h>

/**
 * @brief Get the profile table in WDS array layout (see rfm26w_priv.h, rfm26w_tbl_next()).
 * @param profile which profile
 * @param tbl out: table start
 * @param len out: table size in bytes
 * @retval 0 success
 * @retval -ENOTSUP no table exists for this profile
 */
int rfm26w_profile_table(enum rfm26w_profile profile, const uint8_t **tbl, size_t *len);

#endif /* RFM26W_PROFILES_H_ */

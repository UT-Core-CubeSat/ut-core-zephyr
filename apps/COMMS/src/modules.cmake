# The one list of COMMS source modules (everything except main.c).
#
# Included by apps/COMMS/CMakeLists.txt (the firmware) and by apps/tests/commsTest/CMakeLists.txt (the host
# tests), so the two cannot drift apart. Add a module here when it is written, not in either CMakeLists.
#
# Modules that need Zephyr services a native_sim test build lacks must not be listed here: put them in
# COMMS_TARGET_ONLY_SOURCES instead (none yet).

set(COMMS_SRC_DIR ${CMAKE_CURRENT_LIST_DIR})

set(COMMS_MODULE_SOURCES
    # stage 1: link layer and CAN records
    ${COMMS_SRC_DIR}/link/crc16.c
    ${COMMS_SRC_DIR}/link/tc_frame.c
    ${COMMS_SRC_DIR}/link/space_packet.c
    ${COMMS_SRC_DIR}/link/dl_block.c
    ${COMMS_SRC_DIR}/can/can_record.c
)

set(COMMS_TARGET_ONLY_SOURCES
)

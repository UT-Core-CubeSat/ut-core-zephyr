"""Python port of common/can_proto.h and common/ops.h -- decodes UT-CORE CAN frames.

29-bit ID: prio[28:26] SRC[25:21] DST[20:16] OP[15:4] reserved[3:0]=0.
Payload is p0..p(dlc-1); dlc is fixed per opcode (op_len: 2, 4 or 8).
"""

CAN_SRC_SHIFT = 21
CAN_DST_SHIFT = 16
CAN_OP_SHIFT = 4

# Node IDs
CDH_ID, EPS_ID, COMMS_ID, ADCS_ID = 0x01, 0x02, 0x03, 0x04
MOTOR_ID, GNSS_ID, STAR_ID, SOLAR_ID = 0x05, 0x06, 0x07, 0x08
CAN_BROADCAST = 0x1F
BROADCAST = CAN_BROADCAST

NODE_NAMES = {
    CDH_ID: "CDH", EPS_ID: "EPS", COMMS_ID: "COMMS", ADCS_ID: "ADCS",
    MOTOR_ID: "MOTOR", GNSS_ID: "GNSS", STAR_ID: "STAR", SOLAR_ID: "SOLAR",
    CAN_BROADCAST: "BROADCAST",
}

OP_MAX = 0xFFF


def OP_RESP(cmd: int) -> int:
    return cmd + 0x100


# ---- 0x0xx system / heartbeat ----
OP_HEARTBEAT = 0x001
OP_PING = 0x002
OP_PONG = 0x003
OP_BUTTON = 0x004

# ---- 0x1xx commands ----
OP_SET_LED = 0x101
OP_SET_MODE = 0x102
OP_REBOOT = 0x103
OP_SET_PWR_STATE = 0x104
OP_GET_POS = 0x105
OP_SET_UPDATE_RATE = 0x106
OP_SET_WHEEL_RPM = 0x110
OP_SET_MAG_DIPOLE = 0x111
OP_SET_EPS_LOAD = 0x112

# ---- 0x2xx command responses ----
OP_RESP_SET_PWR_STATE = OP_RESP(OP_SET_PWR_STATE)
OP_RESP_GET_POS = OP_RESP(OP_GET_POS)
OP_RESP_SET_UPDATE_RATE = OP_RESP(OP_SET_UPDATE_RATE)
OP_RESP_SET_WHEEL_RPM = OP_RESP(OP_SET_WHEEL_RPM)
OP_RESP_SET_MAG_DIPOLE = OP_RESP(OP_SET_MAG_DIPOLE)
OP_RESP_SET_EPS_LOAD = OP_RESP(OP_SET_EPS_LOAD)

# ---- 0x3xx telemetry ----
OP_GNSS_POS = 0x301
OP_GNSS_VX = 0x302
OP_GNSS_VY = 0x303
OP_GNSS_VZ = 0x304
OP_GNSS_TIME = 0x305
OP_MOTOR_WHEEL_RPM = 0x310
OP_ADCS_WHEEL_RPM = 0x311
OP_ADCS_MTQ_DIPOLE = 0x312

# ---- 0x4xx state of health ----
OP_SOH_GNSS = 0x401
OP_SOH_SOLAR = 0x402
OP_SOH_MOTOR = 0x403
OP_ADCS_SOH_ATTITUDE = 0x404

OPCODE_NAMES = {
    OP_HEARTBEAT: "HEARTBEAT", OP_PING: "PING", OP_PONG: "PONG",
    OP_BUTTON: "BUTTON",
    OP_SET_LED: "SET_LED", OP_SET_MODE: "SET_MODE", OP_REBOOT: "REBOOT",
    OP_SET_PWR_STATE: "SET_PWR_STATE", OP_GET_POS: "GET_POS",
    OP_SET_UPDATE_RATE: "SET_UPDATE_RATE",
    OP_SET_WHEEL_RPM: "SET_WHEEL_RPM", OP_SET_MAG_DIPOLE: "SET_MAG_DIPOLE",
    OP_SET_EPS_LOAD: "SET_EPS_LOAD",
    OP_RESP_SET_PWR_STATE: "RESP_SET_PWR_STATE",
    OP_RESP_GET_POS: "RESP_GET_POS",
    OP_RESP_SET_UPDATE_RATE: "RESP_SET_UPDATE_RATE",
    OP_RESP_SET_WHEEL_RPM: "RESP_SET_WHEEL_RPM",
    OP_RESP_SET_MAG_DIPOLE: "RESP_SET_MAG_DIPOLE",
    OP_RESP_SET_EPS_LOAD: "RESP_SET_EPS_LOAD",
    OP_GNSS_POS: "GNSS_POS", OP_GNSS_VX: "GNSS_VX", OP_GNSS_VY: "GNSS_VY",
    OP_GNSS_VZ: "GNSS_VZ", OP_GNSS_TIME: "GNSS_TIME",
    OP_MOTOR_WHEEL_RPM: "MOTOR_WHEEL_RPM",
    OP_ADCS_WHEEL_RPM: "ADCS_WHEEL_RPM",
    OP_ADCS_MTQ_DIPOLE: "ADCS_MTQ_DIPOLE",
    OP_SOH_GNSS: "SOH_GNSS", OP_SOH_SOLAR: "SOH_SOLAR",
    OP_SOH_MOTOR: "SOH_MOTOR", OP_ADCS_SOH_ATTITUDE: "ADCS_SOH_ATTITUDE",
}

# Fixed DLC per opcode -- mirror of can_op_len() in common/ops.h.
OP_LEN = {}
for _op in (OP_HEARTBEAT, OP_PING, OP_PONG, OP_BUTTON, OP_SET_LED, OP_SET_MODE,
            OP_REBOOT, OP_SET_PWR_STATE, OP_GET_POS, OP_SET_UPDATE_RATE,
            OP_SET_EPS_LOAD, OP_RESP_GET_POS, OP_RESP_SET_UPDATE_RATE,
            OP_RESP_SET_WHEEL_RPM, OP_RESP_SET_MAG_DIPOLE, OP_RESP_SET_EPS_LOAD):
    OP_LEN[_op] = 2
for _op in (OP_SET_WHEEL_RPM, OP_RESP_SET_PWR_STATE, OP_MOTOR_WHEEL_RPM,
            OP_SOH_MOTOR):
    OP_LEN[_op] = 4
for _op in (OP_SET_MAG_DIPOLE, OP_GNSS_POS, OP_GNSS_VX, OP_GNSS_VY, OP_GNSS_VZ,
            OP_GNSS_TIME, OP_ADCS_WHEEL_RPM, OP_ADCS_MTQ_DIPOLE, OP_SOH_GNSS,
            OP_SOH_SOLAR, OP_ADCS_SOH_ATTITUDE):
    OP_LEN[_op] = 8
del _op


def op_len(op: int) -> int:
    """Expected DLC for an opcode; 0 if unknown."""
    return OP_LEN.get(op, 0)


# Category = op >> 8 (readability only, not part of the protocol logic).
CATEGORY_NAMES = {
    0: "system", 1: "command", 2: "response", 3: "telemetry", 4: "health",
}


def make_id(prio: int, src: int, dst: int, op: int) -> int:
    """Build a 29-bit ID (mirror of CAN_ID_FULL)."""
    return (((prio & 0x07) << 26) | ((src & 0x1F) << CAN_SRC_SHIFT) |
            ((dst & 0x1F) << CAN_DST_SHIFT) | ((op & 0xFFF) << CAN_OP_SHIFT))


def decode_id(can_id: int) -> dict:
    """Decode a 29-bit extended CAN ID per can_proto.h layout."""
    return {
        "priority": (can_id >> 26) & 0x07,
        "src":      (can_id >> CAN_SRC_SHIFT) & 0x1F,
        "dst":      (can_id >> CAN_DST_SHIFT) & 0x1F,
        "op":       (can_id >> CAN_OP_SHIFT) & 0xFFF,
    }


def name_node(node_id: int) -> str:
    return NODE_NAMES.get(node_id, f"0x{node_id:02X}")


def name_opcode(op: int) -> str:
    return OPCODE_NAMES.get(op, f"0x{op:03X}")


def name_category(op: int) -> str:
    cat = (op >> 8) & 0xF
    return CATEGORY_NAMES.get(cat, f"cat{cat}")

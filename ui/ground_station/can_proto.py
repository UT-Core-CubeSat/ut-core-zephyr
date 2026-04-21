"""Python port of common/can_proto.h -- decodes UT-CORE CAN frames."""

# Node IDs
CDH_ID, EPS_ID, COMMS_ID, ADCS_ID = 0x01, 0x02, 0x03, 0x04
MOTOR_ID, GNSS_ID, STAR_ID, SOLAR_ID = 0x05, 0x06, 0x07, 0x08
CAN_BROADCAST = 0xFF

NODE_NAMES = {
    CDH_ID: "CDH", EPS_ID: "EPS", COMMS_ID: "COMMS", ADCS_ID: "ADCS",
    MOTOR_ID: "MOTOR", GNSS_ID: "GNSS", STAR_ID: "STAR", SOLAR_ID: "SOLAR",
    CAN_BROADCAST: "BROADCAST",
}

# Message classes
CLS_HEARTBEAT, CLS_COMMAND, CLS_CMD_RESP = 0, 2, 3
CLS_TELEMETRY, CLS_HEALTH = 4, 10

CLASS_NAMES = {
    CLS_HEARTBEAT: "HEARTBEAT", CLS_COMMAND: "COMMAND",
    CLS_CMD_RESP: "CMD_RESP", CLS_TELEMETRY: "TELEMETRY",
    CLS_HEALTH: "HEALTH",
}

# Opcodes (payload byte 1)
OP_PING, OP_PONG = 0x01, 0x02
OP_BUTTON, OP_SET_LED = 0x10, 0x20
OP_HEARTBEAT, OP_SET_MODE, OP_REBOOT = 0x30, 0x40, 0x41

OPCODE_NAMES = {
    OP_PING: "PING", OP_PONG: "PONG", OP_BUTTON: "BUTTON",
    OP_SET_LED: "SET_LED", OP_HEARTBEAT: "HEARTBEAT",
    OP_SET_MODE: "SET_MODE", OP_REBOOT: "REBOOT",
}


def decode_id(can_id: int) -> dict:
    """Decode a 29-bit extended CAN ID per can_proto.h layout."""
    return {
        "priority": (can_id >> 26) & 0x07,
        "src":      (can_id >> 14) & 0xFF,
        "dst":      (can_id >>  6) & 0xFF,
        "cls":      can_id         & 0x3F,
    }


def name_node(node_id: int) -> str:
    return NODE_NAMES.get(node_id, f"0x{node_id:02X}")


def name_class(cls: int) -> str:
    return CLASS_NAMES.get(cls, f"CLS_{cls}")


def name_opcode(op: int) -> str:
    return OPCODE_NAMES.get(op, f"0x{op:02X}")

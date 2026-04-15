// testingCommunicationWithOrionB16.cpp
// Minimal-but-robust C++ harness to exercise SkyTraq/Sanav "Orion B16" GNSS over the
// SkyTraq Binary Protocol: frame builder, parser, and a few smoke tests
// (Query SW version, switch NMEA/Binary, query/update rate, query power mode, etc.)
//
// Drop-in transport abstraction makes it easy to port to STM32 HAL UART, POSIX termios,
// or any other byte-stream. See comments near the bottom for STM32/UART notes.
//
// NOTE (per vendor manual):
// Frame: 0xA0 0xA1  <PL_hi PL_lo>  <payload = [MessageID][Body...]>  <CS>  0x0D 0x0A
//   * PL counts bytes of [MessageID + Body]
//   * CS is XOR of every byte in payload only (MessageID..last body byte)
//   * Multi-byte fields in payload are BIG-ENDIAN
//
// Message IDs used here (host->receiver):
//   0x02 Query Software Version (body = [SoftwareType=0x00])
//   0x09 Configure Message Type      (body = [Type, Attr])  Type: 0x01=NMEA, 0x02=Binary
//   0x0E Configure Position Rate     (body = [Rate, Attr])  e.g., Rate=1 for 1 Hz
//   0x10 Query Position Update Rate  (no body)
//   0x15 Query Power Mode            (no body)
//   0x16 Query Message Type          (no body)
//   0x4F Query NMEA Talker ID        (no body)
// Receiver outputs we parse/print:
//   0x80 Software Version, 0x83 ACK, 0x84 NACK, 0x86 Position Update Rate,
//   0x8C GNSS Message Type, 0x93 NMEA Talker ID, 0xB9 Power Mode Status, 0xA8 Navigation Data
//
// *** This is a test harness, not a full binary decoder. Unknown messages are hex-dumped. ***

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <optional>
#include <chrono>
#include <thread>
#include <stdexcept>

// Transport Abstraction
struct ITransport {
    virtual ~ITransport() = default;
    // Write exactly 'n' bytes; return true on success
    virtual bool writeBytes(const uint8_t* data, size_t n) = 0;
    // Read one byte with timeout (ms). Return std::nullopt on timeout.
    virtual std::optional<uint8_t> readByteTimeout(uint32_t timeout_ms) = 0;
};

// Protocol Core 
namespace OrionB16 {

static constexpr uint8_t SOF0 = 0xA0;
static constexpr uint8_t SOF1 = 0xA1;
static constexpr uint8_t EOF0 = 0x0D;
static constexpr uint8_t EOF1 = 0x0A;

struct Frame {
    uint16_t payload_len = 0;            // bytes in [MessageID + Body]
    uint8_t  msg_id = 0;                 // first byte in payload
    std::vector<uint8_t> body;           // remaining payload
    uint8_t  checksum = 0;               // XOR over payload bytes
};

inline uint8_t xorChecksum(const std::vector<uint8_t>& payload) {
    uint8_t cs = 0;
    for (uint8_t b : payload) cs ^= b;
    return cs;
}

inline std::vector<uint8_t> encode(uint8_t msg_id, const std::vector<uint8_t>& body) {
    std::vector<uint8_t> payload;
    payload.reserve(1 + body.size());
    payload.push_back(msg_id);
    payload.insert(payload.end(), body.begin(), body.end());

    uint16_t PL = static_cast<uint16_t>(payload.size());
    std::vector<uint8_t> frame;
    frame.reserve(2 + 2 + PL + 1 + 2);

    frame.push_back(SOF0);
    frame.push_back(SOF1);
    frame.push_back(static_cast<uint8_t>((PL >> 8) & 0xFF)); // big-endian
    frame.push_back(static_cast<uint8_t>(PL & 0xFF));
    frame.insert(frame.end(), payload.begin(), payload.end());
    frame.push_back(xorChecksum(payload));
    frame.push_back(EOF0);
    frame.push_back(EOF1);
    return frame;
}

// Robust sync'd frame reader. Returns a parsed Frame or std::nullopt on timeout.
inline std::optional<Frame> readFrame(ITransport& io, uint32_t inter_byte_timeout_ms = 50, uint32_t overall_timeout_ms = 1000) {
    auto t_start = std::chrono::steady_clock::now();

    enum State { SEEK_SOF0, SEEK_SOF1, LEN_HI, LEN_LO, PAYLOAD, CS, EOF0_ST, EOF1_ST };
    State st = SEEK_SOF0;

    Frame f;
    std::vector<uint8_t> payload;

    while (true) {
        // overall timeout
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - t_start).count() > overall_timeout_ms) {
            return std::nullopt;
        }

        auto b_opt = io.readByteTimeout(inter_byte_timeout_ms);
        if (!b_opt.has_value()) {
            // keep looping until overall timeout
            continue;
        }
        uint8_t b = *b_opt;

        switch (st) {
            case SEEK_SOF0:
                if (b == SOF0) st = SEEK_SOF1; else st = SEEK_SOF0;
                break;
            case SEEK_SOF1:
                if (b == SOF1) st = LEN_HI; else st = SEEK_SOF0;
                break;
            case LEN_HI:
                f.payload_len = static_cast<uint16_t>(b) << 8;
                st = LEN_LO;
                break;
            case LEN_LO:
                f.payload_len |= b;
                payload.clear();
                payload.reserve(f.payload_len);
                st = (f.payload_len == 0) ? CS : PAYLOAD;
                break;
            case PAYLOAD:
                payload.push_back(b);
                if (payload.size() >= f.payload_len) st = CS;
                break;
            case CS:
                f.checksum = b;
                st = EOF0_ST;
                break;
            case EOF0_ST:
                if (b != EOF0) { st = SEEK_SOF0; break; }
                st = EOF1_ST;
                break;
            case EOF1_ST:
                if (b != EOF1) { st = SEEK_SOF0; break; }
                // verify checksum
                if (xorChecksum(payload) != f.checksum) {
                    // bad checksum; resync
                    st = SEEK_SOF0;
                    break;
                }
                if (payload.empty()) {
                    // invalid (no msg_id)
                    st = SEEK_SOF0;
                    break;
                }
                f.msg_id = payload[0];
                f.body.assign(payload.begin() + 1, payload.end());
                return f;
        }
    }
}

// Convenience builders
inline std::vector<uint8_t> buildQuerySwVersion() { return encode(0x02, {0x00}); }
inline std::vector<uint8_t> buildQueryMsgType()   { return encode(0x16, {}); }
inline std::vector<uint8_t> buildQueryUpdRate()   { return encode(0x10, {}); }
inline std::vector<uint8_t> buildQueryPowerMode() { return encode(0x15, {}); }
inline std::vector<uint8_t> buildQueryTalkerID()  { return encode(0x4F, {}); }

// type: 0x01=NMEA, 0x02=Binary; attr: 0x00=SRAM only, 0x01=SRAM+FLASH, 0x02=temp
inline std::vector<uint8_t> buildCfgMsgType(uint8_t type, uint8_t attr=0x00) {
    return encode(0x09, {type, attr});
}
// rate: allowed values often {1,2,4,5,8,10,20,25,40,50}; attr as above
inline std::vector<uint8_t> buildCfgUpdateRate(uint8_t rate, uint8_t attr=0x00) {
    return encode(0x0E, {rate, attr});
}

// Minimal decoders for a few response IDs
inline std::string toHex(const std::vector<uint8_t>& v) {
    static const char* hexd = "0123456789ABCDEF";
    std::string s; s.reserve(v.size()*2);
    for (uint8_t b : v) { s.push_back(hexd[b>>4]); s.push_back(hexd[b&0xF]); }
    return s;
}

inline void prettyPrintFrame(const Frame& f) {
    printf("< 0x%02X  len=%u  cs=0x%02X>\n", f.msg_id, (unsigned)(1+f.body.size()), f.checksum);
    auto dumpBody = [&]{ printf("    body: %s\n", toHex(f.body).c_str()); };

    switch (f.msg_id) {
        case 0x83: { // ACK
            printf("  ACK\n"); dumpBody();
            break; }
        case 0x84: { // NACK
            printf("  NACK\n"); dumpBody();
            break; }
        case 0x80: { // Software Version
            // Often ASCII within the body; print as best-effort string as well
            std::string ascii;
            for (uint8_t b : f.body) if (b >= 0x20 && b <= 0x7E) ascii.push_back((char)b);
            printf("  Software Version: %s\n", ascii.empty()? "(non-ASCII)" : ascii.c_str());
            dumpBody();
            break; }
        case 0x86: { // Position Update Rate
            if (!f.body.empty()) printf("  Position Update Rate: %u Hz (raw=0x%02X)\n", f.body[0], f.body[0]);
            dumpBody();
            break; }
        case 0x8C: { // GNSS Message Type
            if (!f.body.empty()) {
                const char* t = (f.body[0] == 0x01) ? "NMEA" : (f.body[0] == 0x02 ? "Binary" : "Unknown");
                printf("  Message Type: %s (0x%02X)\n", t, f.body[0]);
            }
            dumpBody();
            break; }
        case 0x93: { // NMEA Talker ID
            if (!f.body.empty()) printf("  NMEA Talker Mode: 0x%02X (00=GP, 01=GN, 02=Auto)\n", f.body[0]);
            dumpBody();
            break; }
        case 0xB9: { // Power Mode Status
            if (!f.body.empty()) printf("  Power Mode: %s (0x%02X)\n", (f.body[0]==0x00?"Normal":"Power Save"), f.body[0]);
            dumpBody();
            break; }
        case 0xA8: { // Navigation Data Message (binary PVT)
            printf("  Navigation Data Message (len=%zu)\n", f.body.size());
            dumpBody();
            break; }
        default:
            printf("  Unknown/Unhandled message.\n");
            dumpBody();
            break;
    }
}

// Send a command and wait for the next frame (or timeout). This is a simple helper
// for interactive smoke-tests (not robust transaction pairing).
inline bool sendAndPrint(ITransport& io, const std::vector<uint8_t>& cmd, const char* label, uint32_t read_timeout_ms=500) {
    printf("> %s\n", label);
    if (!io.writeBytes(cmd.data(), cmd.size())) {
        fprintf(stderr, "  write failed!\n");
        return false;
    }
    auto f = readFrame(io, /*inter_byte*/50, /*overall*/read_timeout_ms);
    if (!f.has_value()) {
        printf("  (no response)\n");
        return false;
    }
    prettyPrintFrame(*f);
    return true;
}

} // namespace OrionB16

// POSIX Serial (Optional)
#if defined(__unix__) || defined(__APPLE__)
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
class PosixSerial : public ITransport {
public:
    PosixSerial(const std::string& dev, int baud = B115200) {
        fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("open serial failed");
        struct termios tio{};
        tcgetattr(fd_, &tio);
        cfmakeraw(&tio);
        cfsetispeed(&tio, baud);
        cfsetospeed(&tio, baud);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~CSTOPB; // 1 stop
        tio.c_cflag &= ~PARENB; // no parity
        tio.c_cflag &= ~CRTSCTS; // no hw flow
        tcsetattr(fd_, TCSANOW, &tio);
        // non-blocking reads; we implement our own timeouts
    }
    ~PosixSerial() override { if (fd_>=0) ::close(fd_); }

    bool writeBytes(const uint8_t* data, size_t n) override {
        size_t off = 0; ssize_t w;
        while (off < n) {
            w = ::write(fd_, data+off, n-off);
            if (w < 0) { if (errno==EAGAIN || errno==EWOULDBLOCK) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; } return false; }
            off += (size_t)w;
        }
        return true;
    }

    std::optional<uint8_t> readByteTimeout(uint32_t timeout_ms) override {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        uint8_t b;
        while (std::chrono::steady_clock::now() < deadline) {
            ssize_t r = ::read(fd_, &b, 1);
            if (r == 1) return b;
            if (r < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) return std::nullopt;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return std::nullopt;
    }
private:
    int fd_ = -1;
};
#endif

// ------------------------------- Test Main ----------------------------------
static void interactiveSmoke(ITransport& io) {
    using namespace OrionB16;

    // 1) Query SW version
    sendAndPrint(io, buildQuerySwVersion(), "Query Software Version (0x02)", 1000);

    // 2) Query current message type (NMEA vs Binary)
    sendAndPrint(io, buildQueryMsgType(),    "Query Message Type (0x16)", 500);

    // 3) Force Binary output (SRAM only), then re-query
    sendAndPrint(io, buildCfgMsgType(0x02, 0x00), "Configure Message Type: Binary (0x09)", 500);
    sendAndPrint(io, buildQueryMsgType(),          "Query Message Type (0x16)", 500);

    // 4) Query and set update rate to 1 Hz (SRAM only), then re-query
    sendAndPrint(io, buildQueryUpdRate(),          "Query Update Rate (0x10)", 500);
    sendAndPrint(io, buildCfgUpdateRate(1, 0x00),  "Configure Update Rate: 1 Hz (0x0E)", 500);
    sendAndPrint(io, buildQueryUpdRate(),          "Query Update Rate (0x10)", 500);

    // 5) Query power mode and NMEA talker
    sendAndPrint(io, buildQueryPowerMode(),        "Query Power Mode (0x15)", 500);
    sendAndPrint(io, buildQueryTalkerID(),         "Query NMEA Talker ID (0x4F)", 500);

    // 6) Read a few more spontaneous frames (e.g., Navigation Data Message 0xA8)
    for (int i=0;i<5;i++) {
        auto f = readFrame(io, 50, 1000);
        if (!f) { printf("(idle)\n"); continue; }
        prettyPrintFrame(*f);
    }
}

int main(int argc, char** argv) {
#if defined(__unix__) || defined(__APPLE__)
    if (argc < 2) {
        fprintf(stderr, "Usage: %s /dev/ttyUSB0 [baud=115200]\n", argv[0]);
        return 1;
    }
    int baud = B115200;
    if (argc >= 3) {
        int b = atoi(argv[2]);
        switch (b) {
            case 4800: baud=B4800; break; case 9600: baud=B9600; break; case 19200: baud=B19200; break;
            case 38400: baud=B38400; break; case 57600: baud=B57600; break; case 115200: baud=B115200; break;
            case 230400: baud=B230400; break; case 460800: baud=B460800; break; case 921600: baud=B921600; break;
            default: fprintf(stderr, "Unsupported baud; using 115200\n"); baud=B115200; break;
        }
    }
    try {
        PosixSerial serial(argv[1], baud);
        interactiveSmoke(serial);
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        return 2;
    }
#else
    fprintf(stderr, "This example includes a POSIX serial transport.\n");
    fprintf(stderr, "On embedded/STM32: implement ITransport using HAL_UART_Transmit/Receive.\n");
    fprintf(stderr, "Then call interactiveSmoke(yourTransportInstance).\n");
#endif
    return 0;
}

/* --------------------------- STM32/HAL NOTES ---------------------------------

Implement your own transport like this:

class Stm32UartTransport : public ITransport {
public:
    Stm32UartTransport(UART_HandleTypeDef* huart) : huart_(huart) {}
    bool writeBytes(const uint8_t* data, size_t n) override {
        return HAL_UART_Transmit(huart_, const_cast<uint8_t*>(data), n, 100) == HAL_OK;
    }
    std::optional<uint8_t> readByteTimeout(uint32_t timeout_ms) override {
        uint8_t b; auto st = HAL_UART_Receive(huart_, &b, 1, timeout_ms);
        if (st == HAL_OK) return b; else return std::nullopt;
    }
private:
    UART_HandleTypeDef* huart_;
};

// Usage in your main/test task:
//   Stm32UartTransport io(&huartX);
//   OrionB16::interactiveSmoke(io);

------------------------------ TIPS -------------------------------------------
* After power-up many receivers default to NMEA output. The 0x09 Configure Message Type
  can switch to Binary; you can also keep NMEA and still parse binary replies to your
  solicited queries.
* If you increase update rate (0x0E) to >4 Hz, ensure your serial baud >= 38400.
* All payload multi-byte fields are BIG-ENDIAN. Build fields accordingly when you add
  more commands (e.g., Configure DOP Mask 0x2A, Elevation/CNR Mask 0x2B, etc.).
* Transaction pairing: for production code, match 0x83/0x84 ACK/NACK to the command ID
  you sent, and handle retries.
* Navigation Data Message (0xA8) contains binary PVT; parse as needed for your mission.
*/

#include <array>        // For std::array
#include <cmath>        // For std::sqrt
#include <cstdint>      // For uint8_t, uint16_t, int32_t
#include <cstring>      // For std::memcpy
#include <vector>       // For std::vector
#include <algorithm>    // For std::clamp
#include <arpa/inet.h>  // For htons

// Represents the header of a transmission packet for an AGV (Automated Guided Vehicle).
class AgvTxHeader {
public:
    // Enum for the different pages/modes that can be requested.
    enum class Page : uint8_t {
        NONE = 0,
        START = 1,
        PROCESS_1 = 10,
        PROCESS_2 = 11,
        PROCESS_3 = 12,
        PROCESS_4 = 13,
        AUTOMATIC = 20,
        MANUAL_JOG = 30,
        MANUAL_GUIDE = 31,
        STATE = 40,
        NAVIGATION_1 = 50,
        NAVIGATION_2 = 51,
        NAVIGATION_3 = 52,
        LAM_1 = 60,
        LAM_2 = 61
    };

    // Constructor to initialize default values.
    AgvTxHeader()
        : ip{0, 0, 0, 0}, rx_port(0), if_version(0), current_page(Page::NONE), numBytes(0) {}

    // Encodes the header into a byte array.
    std::vector<uint8_t> encode() const {
        std::vector<uint8_t> buf(40); // Allocate 40 bytes.
        uint8_t* ptr = buf.data();    // Pointer to the beginning of the buffer.
        std::memcpy(ptr, ip.data(), 4); // Copy IP address to buffer.
        ptr += 4; // Move pointer forward by 4 bytes.
        *reinterpret_cast<uint16_t*>(ptr) = htons(rx_port); // Copy port, converting to network byte order.
        ptr += 2; // Move pointer forward by 2 bytes.
        *ptr++ = if_version; // Copy interface version.
        *ptr++ = static_cast<uint8_t>(current_page); // Copy current page.
        *reinterpret_cast<uint16_t*>(ptr) = htons(numBytes); // Copy number of bytes, converting to network byte order.
        return buf; // Return the encoded buffer.
    }

    // Setters for the header attributes.
    void setIP(const std::array<uint8_t, 4>& new_ip) { ip = new_ip; }
    void setPort(uint16_t new_port) { rx_port = new_port; }
    void setVersion(uint8_t version) { if_version = version; }
    void setPage(Page page) { current_page = page; }
    void setNumBytes(uint16_t bytes) { numBytes = bytes; }

private:
    // Attributes of the header.
    std::array<uint8_t, 4> ip;
    uint16_t rx_port;
    uint8_t if_version;
    Page current_page;
    uint16_t numBytes;
};

// Represents the body of an AGV transmission.
class AgvTxBody {
public:
    static constexpr size_t BODY_SIZE = 32; // Body size in bytes.

    // Encodes the body into a byte array.
    std::vector<uint8_t> encode() const {
        return std::vector<uint8_t>(BODY_SIZE); // Return a vector with BODY_SIZE bytes.
    }
};

// Represents a message to be transmitted by the AGV.
class AgvTxMsg {
public:
    // Constructor that initializes the header with the given page.
    AgvTxMsg(AgvTxHeader::Page page)
        : header(), body() {
        header.setVersion(2); // Set interface version to 2.
        header.setPage(page); // Set the current page.
        header.setNumBytes(static_cast<uint16_t>(AgvTxBody::BODY_SIZE)); // Set the number of bytes to the body size.
    }

    // Set the IP address for the message.
    void setIP(const std::array<uint8_t, 4>& ip) {
        header.setIP(ip);
    }

    // Set the port number for the message.
    void setPort(uint16_t port) {
        header.setPort(port);
    }

    // Encodes the message (header + body) into a byte array.
    std::vector<uint8_t> encode() const {
        auto buf = header.encode(); // Encode the header.
        auto bodyBuf = body.encode(); // Encode the body.
        buf.insert(buf.end(), bodyBuf.begin(), bodyBuf.end()); // Append the body to the header.
        return buf; // Return the complete encoded message.
    }

protected:
    // Header and body of the message.
    AgvTxHeader header;
    AgvTxBody body;
};

// Represents a start message to establish a connection with the AGV.
class StartTxMsg : public AgvTxMsg {
public:
    // Constructor that sets the page to START.
    StartTxMsg()
        : AgvTxMsg(AgvTxHeader::Page::START) {}
};

// Represents a manual jog transmission to control the movement of the AGV.
class ManualJogTxMsg : public AgvTxMsg {
public:
    // Enum for the speed modes of the AGV.
    enum class SpeedMode : uint8_t {
        CREEP = 0,
        RAPID = 1
    };

    // Constructor that initializes the default values.
    ManualJogTxMsg()
        : AgvTxMsg(AgvTxHeader::Page::MANUAL_JOG), speed_mode(SpeedMode::CREEP), speed(100), x(0), y(0) {}

    // Set the speed mode of the AGV.
    void setSpeedMode(SpeedMode mode) {
        speed_mode = mode;
    }

    // Set the speed of the AGV.
    void setSpeed(float new_speed) {
        speed = std::clamp(new_speed, 0.0f, 100.0f); // Clamp the speed between 0 and 100.
    }

    // Set the direction the AGV should move in.
    void setDirection(float x_dir, float y_dir) {
        float magnitude = std::sqrt(x_dir * x_dir + y_dir * y_dir); // Calculate the magnitude.
        if (magnitude == 0) {
            x = 0.0; // Avoid division by zero.
            y = 0.0;
        } else {
            x = x_dir / magnitude; // Normalize the x direction.
            y = y_dir / magnitude; // Normalize the y direction.
        }
    }

    // Encodes the jog message into a byte array.
    std::vector<uint8_t> encode() const {
        auto buf = AgvTxMsg::encode(); // Encode the base message.
        int x_val = static_cast<int>(x * speed); // Calculate the x value based on the speed.
        int y_val = static_cast<int>(y * speed); // Calculate the y value based on the speed.

        // std::cout << "x_val: " << x_val << std::endl;
        // std::cout << "y_val: " << y_val << std::endl;
        // std::cout << "norm: " << std::sqrt(x_val * x_val + y_val * y_val) << std::endl;
        
        uint8_t* ptr = buf.data() + BODY_OFFSET; // Pointer to the body offset in the buffer.
        *ptr++ = static_cast<uint8_t>(speed_mode); // Copy the speed mode.
        *reinterpret_cast<int32_t*>(ptr) = htonl(x_val); // Copy the x value, converting to network byte order.
        ptr += 4;
        *reinterpret_cast<int32_t*>(ptr) = htonl(y_val); // Copy the y value, converting to network byte order.
        ptr += 4;
        *reinterpret_cast<int32_t*>(ptr) = htonl(static_cast<int32_t>(speed)); // Copy the speed, converting to network byte order.
        return buf; // Return the encoded buffer.
    }

private:
    static constexpr size_t BODY_OFFSET = 40; // Offset value for the body in the buffer.
    SpeedMode speed_mode; // Speed mode of the AGV.
    float speed; // Speed of the AGV.
    float x; // Normalized x direction.
    float y; // Normalized y direction.
};

// Represents a monitor state message to request data from the AGV.
class MonitorTxMsg : public AgvTxMsg {
public:
    // Constructor that sets the page to STATE.
    MonitorTxMsg()
        : AgvTxMsg(AgvTxHeader::Page::STATE) {}
};


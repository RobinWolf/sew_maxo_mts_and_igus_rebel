#ifndef AGV_RX_HEADER_HPP
#define AGV_RX_HEADER_HPP

#include <cstdint>  // Include for fixed-size integer types like uint8_t and uint16_t
#include <array>    // Include for std::array
#include <stdexcept>// Include for standard exceptions
#include <iostream> // Include for input-output stream (for debugging, if needed)

// Class representing the header of an AGV (Automated Guided Vehicle) message received.
class AgvRxHeader {
public:
    // Enumeration for the state of the AGV.
    enum class State : uint8_t {
        OFF = 0,
        INIT = 1,
        AUTO = 2,
        LOCAL = 3,
        STANDBY = 4,
        CHARGING = 5
    };

    // Enumeration for the colors.
    enum class Color : uint8_t {
        GRAY = 48,
        DARK_GREEN = 49,
        GREEN = 50,
        DARK_RED = 51,
        RED = 52,
        YELLOW = 53,
        ORANGE = 54,
        BLUE = 55,
        PURPLE = 56
    };

    // Enumeration for the current page/mode of the AGV.
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

    State state;             // State of the AGV.
    Color color;             // Color of the AGV.
    Page current_page;       // Current page/mode of the AGV.
    bool error;              // Error flag indicating if there's an error.
    uint16_t error_code;     // Error code of the AGV.

    // Constructor to initialize the attributes.
    AgvRxHeader() 
        : state(State::OFF), color(Color::GRAY), current_page(Page::NONE), error(false), error_code(0) {}

    // Method to decode the header from a given byte array (buffer).
    void decode(const std::array<uint8_t, 14>& buf) {
        state = static_cast<State>(buf[2]);                      // Decode the state from the buffer.
        color = static_cast<Color>(buf[3]);                      // Decode the color from the buffer.
        current_page = static_cast<Page>(buf[7]);                // Decode the current page from the buffer.
        error = static_cast<bool>(buf[10]);                      // Decode the error flag from the buffer.
        error_code = (buf[11] << 8) | buf[12];                   // Decode the error code from the buffer.
    }
};



// Class representing a message received from the monitor.
class MonitorRxMsg {
public:
    static constexpr size_t BODY_OFFSET = 40;   // Offset of the message body in the buffer.

    uint16_t part_data;               // Part data.
    uint16_t in_station;              // Station indicator.
    uint16_t in_station_state;        // State of the station.
    uint16_t transponder;             // Transponder data.
    uint16_t transponder_distance;    // Distance to the transponder.
    uint16_t v_track;                 // V-track data.
    uint16_t v_track_distance;        // Distance on the V-track.
    uint16_t actual_speed;            // Actual speed of the AGV.
    uint16_t target_speed;            // Target speed of the AGV.
    uint16_t speed_limit;             // Speed limit.
    uint16_t charging_state;          // Charging state of the AGV.
    uint16_t power;                   // Power level.

    // Constructor to initialize the attributes.
    MonitorRxMsg()
        : part_data(0), in_station(0), in_station_state(0), transponder(0), transponder_distance(0),
          v_track(0), v_track_distance(0), actual_speed(0), target_speed(0), speed_limit(0),
          charging_state(0), power(0) {}

    // Method to decode the message from a given byte array.
    void decode(const std::array<uint8_t, 54>& buf) {
        size_t offset = BODY_OFFSET;   // Start at the body offset.
        part_data = (buf[offset] << 8) | buf[offset + 1];                  // Decode part data.
        in_station = (buf[offset + 2] << 8) | buf[offset + 3];             // Decode station indicator.
        in_station_state = (buf[offset + 4] << 8) | buf[offset + 5];       // Decode station state.
        transponder = (buf[offset + 6] << 8) | buf[offset + 7];            // Decode transponder data.
        transponder_distance = (buf[offset + 8] << 8) | buf[offset + 9];   // Decode distance to transponder.
        v_track = (buf[offset + 10] << 8) | buf[offset + 11];              // Decode V-track data.
        v_track_distance = (buf[offset + 12] << 8) | buf[offset + 13];     // Decode V-track distance.
        actual_speed = (buf[offset + 14] << 8) | buf[offset + 15];         // Decode actual speed.
        target_speed = (buf[offset + 16] << 8) | buf[offset + 17];         // Decode target speed.
        speed_limit = (buf[offset + 18] << 8) | buf[offset + 19];          // Decode speed limit.
        charging_state = (buf[offset + 20] << 8) | buf[offset + 21];       // Decode charging state.
        power = (buf[offset + 22] << 8) | buf[offset + 23];                // Decode power level.
    }
};

#endif // AGV_RX_HEADER_HPP

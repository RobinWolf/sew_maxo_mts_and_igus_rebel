#ifndef AGV_ENDPOINT_HPP
#define AGV_ENDPOINT_HPP

#include <array>
#include <vector>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "tx.hpp"
#include "rx.hpp"

// Class representing the AGV endpoint for communication
class AgvEndpoint {
public:
    // Constructor initializes member variables
    AgvEndpoint() : udpTx_(-1), udpRx_(-1), connected_(false), stopRequested_(false) {}

    // Destructor ensures sockets are closed upon object destruction
    ~AgvEndpoint() {
        disconnect();
    }

    // Connect to the AGV with provided IP addresses and ports
    bool connect(const std::string& agv_ip, int agv_port, const std::string& local_ip, int local_port) {
        // Disconnect any existing connection
        disconnect();

        // Store the connection details
        this->agv_ip_ = agv_ip;
        this->agv_port_ = agv_port;
        this->local_ip_ = local_ip;
        this->local_port_ = local_port;

        // std::cout << "(AGVEndpoint) AGV IP: " << agv_ip_ << std::endl;
        // std::cout << "(AGVEndpoint) AGV Port: " << agv_port_ << std::endl;
        // std::cout << "(AGVEndpoint) Local IP: " << local_ip_ << std::endl;
        // std::cout << "(AGVEndpoint) Local Port: " << local_port_ << std::endl;

        // Create UDP sockets for transmission and reception
        udpTx_ = socket(AF_INET, SOCK_DGRAM, 0);
        udpRx_ = socket(AF_INET, SOCK_DGRAM, 0);

        // Check if socket creation was successful
        if (udpTx_ < 0 || udpRx_ < 0) {
            std::cerr << "(AGVEndpoint) Failed to create sockets" << std::endl;
            return false;
        }

        // Add timeout for receiving data to avoid blocking the program
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 200000; // Timeout in microseconds
        setsockopt(udpRx_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        // Set up the local address structure
        sockaddr_in localAddr {};
        localAddr.sin_family = AF_INET;
        localAddr.sin_addr.s_addr = inet_addr(local_ip_.c_str());
        localAddr.sin_port = htons(local_port_);

        std::cout << "(AGVEndpoint) Bind the receiving socket to the local address" << std::endl;
        // Bind the receiving socket to the local address
        if (bind(udpRx_, reinterpret_cast<struct sockaddr*>(&localAddr), sizeof(localAddr)) < 0) {
            perror("(AGVEndpoint) Failed to bind the socket");
            return false;
        }


        // Start the connection thread
        stopRequested_ = false;
        connectionThread_ = std::thread(&AgvEndpoint::connectionLoop, this);

        connected_ = true;
        return true;
    }

    // Disconnect from the AGV and close the sockets
    void disconnect() {
        std::cout << "(AGVEndpoint) disconnect()" << std::endl;
        stopRequested_ = true;
        if (connectionThread_.joinable()) {
            connectionThread_.join();
        }
        if (udpTx_ >= 0) {
            close(udpTx_);
            udpTx_ = -1;
        }
        if (udpRx_ >= 0) {
            close(udpRx_);
            udpRx_ = -1;
        }
        connected_ = false;
    }

    bool isConnected() const {
        return connected_;
    }

    // Write comands to the AGV tx buffer
    bool writeAgvTxBuffer(float speed, float x, float y, ManualJogTxMsg::SpeedMode speed_mode = ManualJogTxMsg::SpeedMode::CREEP) {
        if (!connected_) {
            std::cerr << "(AGVEndpoint) AGV is not connected" << std::endl;
            return false;
        }
        // Create and set up the control message
        ManualJogTxMsg msg;
        msg.setSpeedMode(speed_mode);
        msg.setSpeed(speed);
        msg.setDirection(x, y);

        // Encode the message
        std::vector<uint8_t> encoded_msg = msg.encode();

        // Lock the txBuffer_ and set the new data
        {
            std::lock_guard<std::mutex> lock(txMutex_);
            txBuffer_ = encoded_msg;
        }

        return true;
    }

    // read the AGV rx buffer
    bool readAgvRxBuffer() {
        if (!connected_) {
            std::cerr << "(AGVEndpoint) AGV is not connected" << std::endl;
            return false;
        }

        // Lock the rxBuffer_ and print its content
        std::array<uint8_t, 54> localRxBuffer;
        {
            std::lock_guard<std::mutex> lock(rxMutex_);
            localRxBuffer = rxBuffer_;
            std::fill(rxBuffer_.begin(), rxBuffer_.end(), 0);

            std::cout << "(readAgvRxBuffer) Received Buffer content: ";
                for (const auto& byte : localRxBuffer) {
                    std::cout << static_cast<int>(byte) << " ";
                }
                std::cout << std::endl;

        }

        // Check if localRxBuffer contains only zeros
        bool allZeros = std::all_of(localRxBuffer.begin(), localRxBuffer.end(), [](uint8_t byte) { return byte == 0; });

        if (allZeros) {
            std::cout << "(AGVEndpoint) localRxBuffer contains only zeros" << std::endl;
            return false;
        } else {     
            // Decode and print the header and message data
            AgvRxHeader header;
            std::array<uint8_t, 14> header_buf;
            std::copy(localRxBuffer.begin(), localRxBuffer.begin() + 14, header_buf.begin());
            header.decode(header_buf);

            MonitorRxMsg msg;
            msg.decode(localRxBuffer);

            // Print AGV status
            std::cout << "(AGVEndpoint) AGV Status:" << std::endl;
            std::cout << "(AGVEndpoint) State: " << static_cast<int>(header.state) << std::endl;
            std::cout << "(AGVEndpoint) Color: " << static_cast<int>(header.color) << std::endl;
            std::cout << "(AGVEndpoint) Current Page: " << static_cast<int>(header.current_page) << std::endl;
            std::cout << "(AGVEndpoint) Error: " << header.error << std::endl;
            std::cout << "(AGVEndpoint) Error Code: " << header.error_code << std::endl;
            std::cout << "(AGVEndpoint) Part Data: " << msg.part_data << std::endl;
            std::cout << "(AGVEndpoint) In Station: " << msg.in_station << std::endl;
            std::cout << "(AGVEndpoint) In Station State: " << msg.in_station_state << std::endl;
            std::cout << "(AGVEndpoint) Transponder: " << msg.transponder << std::endl;
            std::cout << "(AGVEndpoint) Transponder Distance: " << msg.transponder_distance << std::endl;
            std::cout << "(AGVEndpoint) V-Track: " << msg.v_track << std::endl;
            std::cout << "(AGVEndpoint) V-Track Distance: " << msg.v_track_distance << std::endl;
            std::cout << "(AGVEndpoint) Actual Speed: " << msg.actual_speed << std::endl;
            std::cout << "(AGVEndpoint) Target Speed: " << msg.target_speed << std::endl;
            std::cout << "(AGVEndpoint) Speed Limit: " << msg.speed_limit << std::endl;
            std::cout << "(AGVEndpoint) Charging State: " << msg.charging_state << std::endl;
            std::cout << "(AGVEndpoint) Power: " << msg.power << std::endl;


            std::fill(localRxBuffer.begin(), localRxBuffer.end(), 0);
            return true;
        }
    }

private:
    // Member variables for AGV connection details
    std::string agv_ip_;  // AGV IP address
    int agv_port_;        // AGV port
    std::string local_ip_; // Local IP address
    int local_port_;      // Local port
    int udpTx_;           // UDP socket for transmitting data
    int udpRx_;           // UDP socket for receiving data
    bool connected_;      // Connection status flag

    // Thread and synchronization
    std::thread connectionThread_;
    std::condition_variable cv_;
    std::atomic<bool> stopRequested_; // Flag to signal the thread to stop

    // Buffers for incoming and outgoing data
    std::array<uint8_t, 54> rxBuffer_;
    std::vector<uint8_t> txBuffer_;
    std::mutex txMutex_;
    std::mutex rxMutex_;

    // Function that runs in a separate thread for continuous communication
    void connectionLoop() {
        std::cout << "(AGVEndpoint) Connection loop started" << std::endl;

        /* // Read AGV status one time --> works on pc, but causes problems on raspi5
        // Create a start message to send to the AGV and recive AGV status
        StartTxMsg startMsg;
        startMsg.setIP(parseIp(local_ip_));
        startMsg.setPort(local_port_);

        // Encode the start message
        std::vector<uint8_t> temp_buf = startMsg.encode();

        // Send start message until a response is received
        bool recivedStatus = false;
        while (!recivedStatus) {
            sendDataToAgv(temp_buf);  // Send start message
            temp_buf.resize(1024);
            sockaddr_in senderAddr {};
            socklen_t addrLen = sizeof(senderAddr);
            
            // Receive a response
            int len = recvfrom(udpRx_, temp_buf.data(), temp_buf.size(), 0, reinterpret_cast<struct sockaddr*>(&senderAddr), &addrLen);
            if (len > 0) {
                recivedStatus = true;

                // copy the received data to the rxBuffer_
                {
                std::lock_guard<std::mutex> lock(rxMutex_);
                std::copy(temp_buf.begin(), temp_buf.end(), rxBuffer_.begin());
                }

                readAgvRxBuffer();      // Print AGV Status

            } else {
                std::cout << "(AGVEndpoint) not connected" << std::endl;
            }
        }*/

        while (!stopRequested_) {
            // Send data to AGV if txBuffer_ is not empty
            std::vector<uint8_t> localTxBuffer;
            {
                std::lock_guard<std::mutex> lock(txMutex_);
                localTxBuffer = txBuffer_;
                txBuffer_.clear();
            }

            if (!localTxBuffer.empty()) {
                // Send the data to the AGV
                sendDataToAgv(localTxBuffer);
                localTxBuffer.clear();
            } else {
                // Send only the header if txBuffer_ is empty
                ManualJogTxMsg msg;
                msg.setSpeedMode(ManualJogTxMsg::SpeedMode::CREEP); // Set default header values if needed
                localTxBuffer = msg.encode();
                sendDataToAgv(localTxBuffer);
                localTxBuffer.clear();
            }

            // Receive data from the AGV
            std::array<uint8_t, 54> buf;
            sockaddr_in senderAddr {};
            socklen_t addrLen = sizeof(senderAddr);
  
            int len = recvfrom(udpRx_, buf.data(), buf.size(), 0, reinterpret_cast<struct sockaddr*>(&senderAddr), &addrLen);
            if (len > 0) {
                std::lock_guard<std::mutex> lock(rxMutex_);
                std::copy(buf.begin(), buf.end(), rxBuffer_.begin());
                for (const auto& byte : buf) {
                    std::cout << static_cast<int>(byte) << " ";
                }
                std::cout << std::endl;
            }
            else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("(AGVEndpoint) recvfrom error");
            }

            // Wait for a while before sending the next message
            //std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "(AGVEndpoint) Connection loop stopped" << std::endl;
    }

    // Helper function to send data to the AGV
    void sendDataToAgv(const std::vector<uint8_t>& buf) {
        struct sockaddr_in agvAddr {};
        agvAddr.sin_family = AF_INET;
        agvAddr.sin_addr.s_addr = inet_addr(agv_ip_.c_str());
        agvAddr.sin_port = htons(agv_port_);
        ssize_t sent_bytes = sendto(udpTx_, buf.data(), buf.size(), 0, reinterpret_cast<struct sockaddr*>(&agvAddr), sizeof(agvAddr));
        if (sent_bytes < 0) {
            perror("(AGVEndpoint) Error sending to AGV");
        } 
    }

    // Helper function to parse IP address into an array of bytes
    std::array<uint8_t, 4> parseIp(const std::string& ip) {
        std::array<uint8_t, 4> arr;
        sscanf(ip.c_str(), "%hhu.%hhu.%hhu.%hhu", &arr[0], &arr[1], &arr[2], &arr[3]);
        return arr;
    }
};

#endif // AGV_ENDPOINT_HPP

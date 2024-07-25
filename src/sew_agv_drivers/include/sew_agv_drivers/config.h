#ifndef DIFFDRIVE_CONFIG_H
#define DIFFDRIVE_CONFIG_H

#include <string>


struct Config
{
  std::string agv_ip;    // The IP address or hostname of the AGV.
  int agv_port;          // The port number of the AGV.
  std::string local_ip;  // The local IP address to bind the UDP socket.
  int local_port;        // The local port number to bind the UDP socket.
};

#endif // DIFFDRIVE_CONFIG_H
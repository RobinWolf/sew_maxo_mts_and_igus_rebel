#include "sew_agv_drivers/agv_endpoint.hpp"
#include "sew_agv_drivers/tx.hpp"
#include "sew_agv_drivers/rx.hpp"
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

namespace sew_agv_drivers
{
AgvEndpoint::AgvEndpoint(const std::string & agv_host, int agv_port, const std::string & local_ip, int local_port)
: agv_host_(agv_host), agv_port_(agv_port), local_ip_(local_ip), local_port_(local_port), running_(false)
{
  sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  memset(&agv_addr_, 0, sizeof(agv_addr_));
  agv_addr_.sin_family = AF_INET;
  agv_addr_.sin_port = htons(agv_port_);
  inet_pton(AF_INET, agv_host_.c_str(), &agv_addr_.sin_addr);
}

void AgvEndpoint::start()
{
  running_ = true;
  thread_ = std::thread(&AgvEndpoint::run, this);
}

void AgvEndpoint::stop()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void AgvEndpoint::set_msg(const AgvTxMsg & msg)
{
  std::lock_guard<std::mutex> lock(lock_);
  msg_ = msg;
}

void AgvEndpoint::run()
{
  while (running_) {
    AgvTxMsg msg;
    {
      std::lock_guard<std::mutex> lock(lock_);
      msg = msg_;
    }
    std::vector<uint8_t> data = msg.serialize();
    sendto(sock_, data.data(), data.size(), 0, (struct sockaddr *)&agv_addr_, sizeof(agv_addr_));
    usleep(100000);  // 10 Hz
  }
  close(sock_);
}
}  // namespace sew_agv_drivers

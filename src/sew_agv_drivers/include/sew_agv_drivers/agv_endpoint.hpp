#ifndef SEW_AGV_DRIVERS__AGV_ENDPOINT_HPP_
#define SEW_AGV_DRIVERS__AGV_ENDPOINT_HPP_

#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include "sew_agv/tx.hpp"
#include "sew_agv/rx.hpp"

namespace sew_agv_drivers
{
class AgvEndpoint
{
public:
  AgvEndpoint(const std::string & agv_host, int agv_port, const std::string & local_ip, int local_port);
  void start();
  void stop();
  void set_msg(const AgvTxMsg & msg);

private:
  void run();

  std::string agv_host_;
  int agv_port_;
  std::string local_ip_;
  int local_port_;

  std::thread thread_;
  std::atomic_bool running_;
  std::mutex lock_;
  AgvTxMsg msg_;
  int sock_;
  struct sockaddr_in agv_addr_;
  };
}  // namespace sew_agv_drivers

#endif  // SEW_AGV_DRIVERS__AGV_ENDPOINT_HPP_

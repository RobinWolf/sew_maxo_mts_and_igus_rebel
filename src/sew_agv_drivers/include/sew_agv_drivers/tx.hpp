
#ifndef SEW_AGV_DRIVERS__TX_HPP_
#define SEW_AGV_DRIVERS__TX_HPP_

#include <vector>
#include <cstdint>

namespace sew_agv_drivers
{
class AgvTxMsg
{
public:
  AgvTxMsg() : speed_(0.0), direction_x_(0.0), direction_y_(0.0) {}
  void set_speed(double speed);
  void set_direction(double x, double y);
  std::vector<uint8_t> serialize() const;

private:
  double speed_;
  double direction_x_;
  double direction_y_;
};
}  // namespace sew_agv_drivers

#endif  // SEW_AGV_DRIVERS__TX_HPP_

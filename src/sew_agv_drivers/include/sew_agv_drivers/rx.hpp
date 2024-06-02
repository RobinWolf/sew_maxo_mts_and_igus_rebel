#ifndef SEW_AGV_DRIVERS__RX_HPP_
#define SEW_AGV_DRIVERS__RX_HPP_

#include <vector>
#include <cstdint>

namespace sew_agv_drivers
{
class AgvRxMsg
{
public:
  AgvRxMsg();
  bool deserialize(const std::vector<uint8_t>& data);

  double get_speed() const;
  double get_direction_x() const;
  double get_direction_y() const;

private:
  double speed_;
  double direction_x_;
  double direction_y_;
};
}  // namespace sew_agv_drivers

#endif  // SEW_AGV_DRIVERS__RX_HPP_

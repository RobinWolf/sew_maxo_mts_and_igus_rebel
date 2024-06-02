#include "sew_agv_drivers/rx.hpp"
#include <cstring>

namespace sew_agv_drivers
{
AgvRxMsg::AgvRxMsg() : speed_(0.0), direction_x_(0.0), direction_y_(0.0) {}

bool AgvRxMsg::deserialize(const std::vector<uint8_t>& data)
{
  if (data.size() != sizeof(double) * 3) {
    return false;
  }
  std::memcpy(&speed_, data.data(), sizeof(double));
  std::memcpy(&direction_x_, data.data() + sizeof(double), sizeof(double));
  std::memcpy(&direction_y_, data.data() + sizeof(double) * 2, sizeof(double));
  return true;
}

double AgvRxMsg::get_speed() const
{
  return speed_;
}

double AgvRxMsg::get_direction_x() const
{
  return direction_x_;
}

double AgvRxMsg::get_direction_y() const
{
  return direction_y_;
}
}  // namespace sew_agv_drivers

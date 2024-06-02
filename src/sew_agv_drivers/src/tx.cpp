#include "sew_agv_drivers/tx.hpp"
#include <cstring>

namespace sew_agv_drivers
{
void AgvTxMsg::set_speed(double speed)
{
  speed_ = speed;
}

void AgvTxMsg::set_direction(double x, double y)
{
  direction_x_ = x;
  direction_y_ = y;
}

std::vector<uint8_t> AgvTxMsg::serialize() const
{
  std::vector<uint8_t> data(sizeof(double) * 3);
  std::memcpy(data.data(), &speed_, sizeof(double));
  std::memcpy(data.data() + sizeof(double), &direction_x_, sizeof(double));
  std::memcpy(data.data() + sizeof(double) * 2, &direction_y_, sizeof(double));
  return data;
}
}  // namespace sew_agv_drivers

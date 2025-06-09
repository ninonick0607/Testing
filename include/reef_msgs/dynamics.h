#ifndef REEF_MSGS_DYNAMICS_H
#define REEF_MSGS_DYNAMICS_H

#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>

namespace reef_msgs {
inline double get_yaw(const geometry_msgs::msg::Quaternion &q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
}

#endif  // REEF_MSGS_DYNAMICS_H

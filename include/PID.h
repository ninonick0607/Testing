#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <simple_pid.h>
#include <math.h>
#include <algorithm>
#include <reef_msgs/dynamics.h>
#include "controller.h"

namespace reef_control
{
  class PIDController : public Controller
  {
  public:
    PIDController();

  private:
    reef_control::SimplePID d_; //simple pid object for Down
    reef_control::SimplePID yaw_; //simple pid object for Yaw
    reef_control::SimplePID u_; //simple pid object for velocity
    reef_control::SimplePID v_; //simple pid object for velocity
    reef_control::SimplePID w_; //simple pid object for velocity
    reef_control::SimplePID r_; //simple pid object for Yaw rate

    double threshold_;
    double speed_;
    double thrust_to_hover_bias;
    double ax, ay, az;
    double current_yaw;
    double kp;
    double deadzone;
    double vel_max;
    double x_0;
    double alpha;
    double sigma;
    double gaussian_offset_;
    double theta;
    bool face_target_;
    bool fly_fixed_wing_;

    rclcpp::Publisher<reef_msgs::msg::DesiredState>::SharedPtr desired_state_pub_;


    void lookupTable(reef_msgs::msg::DesiredState& desired_state ,const nav_msgs::msg::Odometry& current_state);

    void computeCommand(const nav_msgs::msg::Odometry current_state,
                        reef_msgs::msg::DesiredState& desired_state,
                        double dt);

  };
}
#endif
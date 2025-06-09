#include <rclcpp/rclcpp.hpp>
#include "controller.h"

namespace reef_control
{
  Controller::Controller() :
    node_(std::make_shared<rclcpp::Node>("reef_controller")),
    armed_(false),
    initialized_(false),
    is_flying_(false)
  {

    // Get Parameters
    node_->declare_parameter("gravity", 9.80665);
    gravity_ = node_->get_parameter("gravity").as_double();

    node_->declare_parameter("max_roll", 0.0);
    node_->declare_parameter("max_pitch", 0.0);
    node_->declare_parameter("max_yaw_rate", 0.0);
    max_roll_ = node_->get_parameter("max_roll").as_double();
    max_pitch_ = node_->get_parameter("max_pitch").as_double();
    max_yaw_rate_ = node_->get_parameter("max_yaw_rate").as_double();

    command_publisher_       = node_->create_publisher<rosflight_msgs::msg::Command>("command", 1);

    desired_state_subcriber_ = node_->create_subscription<reef_msgs::msg::DesiredState>("desired_state",1,std::bind(&Controller::desiredStateCallback,this,std::placeholders::_1));
    status_subscriber_       = node_->create_subscription<rosflight_msgs::msg::Status>("status",1,std::bind(&Controller::statusCallback,this,std::placeholders::_1));
    is_flying_subcriber_     = node_->create_subscription<std_msgs::msg::Bool>("is_flying",1,std::bind(&Controller::isflyingCallback,this,std::placeholders::_1));
    current_state_subcriber_ = node_->create_subscription<reef_msgs::msg::XYZEstimate>("xyz_estimate",1,std::bind(&Controller::currentStateCallback,this,std::placeholders::_1));
    rc_in_subcriber_         = node_->create_subscription<rosflight_msgs::msg::RCRaw>("rc_raw",1,std::bind(&Controller::RCInCallback,this,std::placeholders::_1));
    pose_subcriber_          = node_->create_subscription<geometry_msgs::msg::PoseStamped>("pose_stamped",1,std::bind(&Controller::poseCallback,this,std::placeholders::_1));

    time_of_previous_control_ = node_->get_clock()->now();

  }

  void Controller::desiredStateCallback(const reef_msgs::msg::DesiredState& msg)
  {
    desired_state_ = msg;
  }

  void Controller::currentStateCallback(const reef_msgs::msg::XYZEstimate& msg)
  {
    current_state_.header = msg.header;
    current_state_.twist.twist.linear.x = msg.xy_plus.x_dot;
    current_state_.twist.twist.linear.y = msg.xy_plus.y_dot;
    current_state_.twist.twist.linear.z = msg.z_plus.z_dot;
    current_state_.pose.pose.position.z = msg.z_plus.z;
    computeCommand();
  }

  void Controller::poseCallback(const geometry_msgs::msg::PoseStamped& msg)
  {
    current_state_.pose.pose.position.x = msg.pose.position.x;
    current_state_.pose.pose.position.y = msg.pose.position.y;
    current_state_.pose.pose.orientation = msg.pose.orientation;

  }

  void Controller::statusCallback(const rosflight_msgs::msg::Status &msg)
  {
    armed_ = msg.armed;
    initialized_ = armed_;
  }

  void Controller::isflyingCallback(const std_msgs::msg::Bool &msg)
  {
    is_flying_ = msg.data;
    initialized_ = is_flying_ && armed_;
  }

  void Controller::RCInCallback(const rosflight_msgs::msg::RCRaw &msg)
  {

  }

  void Controller::computeCommand()
  {
    // Time calculation
    dt = (rclcpp::Time(current_state_.header.stamp) - time_of_previous_control_).seconds();
    time_of_previous_control_ = rclcpp::Time(current_state_.header.stamp);
    if(dt <= 0.0000001)
    {
      // Don't do anything if dt is really close (or equal to) zero
      return;
    }

    computeCommand(current_state_ ,desired_state_,dt);

    phi_desired = desired_state_.acceleration.y;
    theta_desired = -desired_state_.acceleration.x;
    thrust = -desired_state_.acceleration.z;

    /*
    accel_out = Eigen::Vector3d(desired_state_.acceleration.x, desired_state_.acceleration.y, desired_state_.acceleration.z );
    total_accel = sqrt( pow(accel_out.x(),2) + pow(accel_out.y(),2) + pow((1 - accel_out.z()),2) );
    thrust = total_accel * hover_throttle_ ;

    if(thrust > 0.001)
    {
      phi_desired = asin(accel_out.y() / total_accel);
      theta_desired = -1.0 * asin(accel_out.x() / total_accel);
    }
    else
    {
        phi_desired = 0;
        theta_desired = 0;
    }
    */

    command.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command.F = std::min(std::max(thrust, 0.0), 1.0);
    //desired_state_.altitude_only = true;
    if(!desired_state_.attitude_valid && !desired_state_.altitude_only) {
      command.ignore = 0x00;
      command.x = std::min(std::max(phi_desired, -1.0 * max_roll_), max_roll_);
      command.y = std::min(std::max(theta_desired, -1.0 * max_pitch_), max_pitch_);
      command.z = std::min(std::max(desired_state_.velocity.yaw, -1.0 * max_yaw_rate_), max_yaw_rate_);
    }else if(desired_state_.altitude_only)
      command.ignore = 0x07;
    else
    {
      command.ignore = 0x00;
      command.x = desired_state_.attitude.x;
      command.y = desired_state_.attitude.y;
      command.z = desired_state_.attitude.yaw;
    }

    command_publisher_.publish(command);
  }

} //namespace

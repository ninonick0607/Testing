#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <rclcpp/rclcpp.hpp>

#include <rosflight_msgs/Command.h>
#include <rosflight_msgs/Status.h>
#include <rosflight_msgs/RCRaw.h>

#include <math.h>
#include <eigen3/Eigen/Core>

#include <reef_msgs/msg/xyz_estimate.hpp>
#include <reef_msgs/msg/desired_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <simple_pid.h>


namespace reef_control
{
  class Controller
  {
  public:
    Controller();
    ~Controller(){}

    bool initialized_;
    rclcpp::Node::SharedPtr node_;

   private:
    bool is_flying_;                       // Set by is_flying callback
    bool armed_;
    bool xy_control_flag;

    nav_msgs::msg::Odometry current_state_;
    reef_msgs::msg::DesiredState desired_state_;

    rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr command_publisher_;

    rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_subscriber_;
    rclcpp::Subscription<reef_msgs::msg::XYZEstimate>::SharedPtr current_state_subcriber_;
    rclcpp::Subscription<reef_msgs::msg::DesiredState>::SharedPtr desired_state_subcriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_flying_subcriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subcriber_;
    rclcpp::Subscription<rosflight_msgs::msg::RCRaw>::SharedPtr rc_in_subcriber_;

    rclcpp::Time time_of_previous_control_;
    rosflight_msgs::Command command;

    double mass_;
    double gravity_;
    double max_roll_, max_pitch_, max_yaw_rate_;
    double max_thrust_, min_thrust_;
    double hover_throttle_;
    double max_u_, max_v_, max_w_;
    double dt;
    double total_accel;
    double thrust;
    double phi_desired;
    double theta_desired;

    Eigen::Vector3d accel_out;

    void currentStateCallback(const reef_msgs::msg::XYZEstimate& msg);
    void desiredStateCallback(const reef_msgs::msg::DesiredState& msg);
    void poseCallback(const geometry_msgs::msg::PoseStamped& msg);
    void isflyingCallback(const std_msgs::msg::Bool& msg);
    void statusCallback(const rosflight_msgs::msg::Status &msg);
    void RCInCallback(const rosflight_msgs::msg::RCRaw &msg);
    void computeCommand();  // Computes and sends command message

    // Virtual Function
    virtual void computeCommand(const nav_msgs::msg::Odometry current_state,
                  reef_msgs::msg::DesiredState& desired_state,
                  double dt) = 0;

  };
}
#endif
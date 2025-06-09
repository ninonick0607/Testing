#include <rclcpp/rclcpp.hpp>
#include "controller.h"
#include "PID.h"

namespace reef_control
{
  PIDController::PIDController() : Controller()
  {
    desired_state_pub_ = node_->create_publisher<reef_msgs::msg::DesiredState>("controller_state", 1);
    node_->declare_parameter("face_target", false);
    node_->declare_parameter("fly_fixed_wing", false);
    face_target_ = node_->get_parameter("face_target").as_bool();
    fly_fixed_wing_ = node_->get_parameter("fly_fixed_wing").as_bool();
  }


  void PIDController::computeCommand(const nav_msgs::msg::Odometry current_state_,
                                     reef_msgs::msg::DesiredState& desired_state,
                                     double dt)  {
    if(!initialized_) {
      d_.clearIntegrator();
      yaw_.clearIntegrator();
      u_.clearIntegrator();
      v_.clearIntegrator();
      w_.clearIntegrator();
    }

    current_yaw = reef_msgs::get_yaw(current_state_.pose.pose.orientation);
    desired_state.velocity.z = d_.computePID(desired_state.pose.z, current_state_.pose.pose.position.z, dt);
    desired_state.acceleration.z = w_.computePID(desired_state.velocity.z, current_state_.twist.twist.linear.z, dt);

    if(desired_state.position_valid)
    {
      if(face_target_) {
      	desired_state.pose.yaw = theta + current_yaw;
      }
      desired_state.velocity.yaw = yaw_.computePID(desired_state.pose.yaw, current_yaw, dt);
      lookupTable(desired_state,current_state_);
      desired_state.velocity_valid = true;
    }
    
    if(desired_state.velocity_valid)
    {
      desired_state.acceleration.x = u_.computePID(desired_state.velocity.x, current_state_.twist.twist.linear.x, dt);
      desired_state.acceleration.y = v_.computePID(desired_state.velocity.y, current_state_.twist.twist.linear.y, dt);
    }
    desired_state_pub_.publish(desired_state);
  }

  void PIDController::lookupTable(reef_msgs::msg::DesiredState& desired_state ,const nav_msgs::msg::Odometry& current_state_)
  {
    double velocity_request;
    double euclidian_distance;
    double x_error;
    double y_error;
    double multiplier;

    x_error = desired_state.pose.x - current_state_.pose.pose.position.x;
    y_error = desired_state.pose.y - current_state_.pose.pose.position.y;
    euclidian_distance = sqrt(x_error * x_error + y_error * y_error);
    if (euclidian_distance < deadzone)
      velocity_request = 0;
    else if (desired_state.velocity_valid) 
    	velocity_request = sqrt(desired_state.velocity.x*desired_state.velocity.x + desired_state.velocity.y*desired_state.velocity.y);
    else 
    	velocity_request = 1.*vel_max * 1 / ( 1 + kp * exp( - (euclidian_distance - x_0)/alpha ) );
    if (false and desired_state.velocity_valid) theta = atan2(desired_state.velocity.y, desired_state.velocity.x) - current_yaw; //Theoretically more correct?
    else theta = atan2(y_error,x_error) - current_yaw; // This is desired_heading_in_body_frame = desired_heading_in_world_frame - body_heading_in_world_frame

    if(fly_fixed_wing_){
      multiplier = exp( - ( desired_state.velocity.yaw * desired_state.velocity.yaw ) / (2 * sigma * sigma) );
      multiplier = std::min(multiplier + 0.1 , 1.0);
    }
    else
      multiplier = 1;

    desired_state.velocity.x = multiplier  * velocity_request * cos(theta);
    desired_state.velocity.y = multiplier * velocity_request * sin(theta);

  }


}

#include <cstdio>
#include <cmath>

#include <chibarobo2024main/time_operator.hpp>

#include <rclcpp/rclcpp.hpp>
#include <chibarobo2024main/shooter.hpp>
#include <sensor_msgs/msg/joy.hpp>

void shooter::pose2d_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
  (void) msg;
  RCLCPP_INFO(this->get_logger(), "I heard:");
  pose2d = *msg;
}

void shooter::timer_callback()
{
  (void) pose2d;
  RCLCPP_INFO(this->get_logger(), "I heard:");
  switch(robotstate){
  case RobotState::STOPstartpose :
  {
    robot_target_vel.set__x(0.0f).set__y(0.0f).set__theta(0.0f);
    robot_vel_publisher_->publish(robot_target_vel);
    break;
  }
  case RobotState::RUNtoSHOOTpose :
  {
    if(std::abs(shoot_pose2d.x - pose2d.x) < 0.1f && std::abs(shoot_pose2d.y - pose2d.y) < 0.1f && std::abs(shoot_pose2d.theta - pose2d.theta) < 0.1f)
    {
      robotstate = RobotState::SHOOTING;
      break;
    }
    kp_ = 1.0f;
    robot_target_vel.set__x(kp_*(shoot_pose2d.x - pose2d.x)).set__y(kp_*(shoot_pose2d.y - pose2d.y)).set__theta(kp_*(shoot_pose2d.theta - pose2d.theta));
    robot_vel_publisher_->publish(robot_target_vel);
    break;
  }
  case RobotState::SHOOTING :
  {
    
    break;
  }
  case RobotState::RUNtoSTARTpose :
  {
    if(std::abs(start_pose2d.x - pose2d.x) < 0.1f && std::abs(start_pose2d.y - pose2d.y) < 0.1f && std::abs(start_pose2d.theta - pose2d.theta) < 0.1f)
    {
      robotstate = RobotState::STOPstartpose;
      break;
    }
    kp_ = 1.0f;
    robot_target_vel.set__x(kp_*(start_pose2d.x - pose2d.x)).set__y(kp_*(start_pose2d.y - pose2d.y)).set__theta(kp_*(start_pose2d.theta - pose2d.theta));
    robot_vel_publisher_->publish(robot_target_vel);
    break;
  }
  }
}

void shooter::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  (void) msg;
  RCLCPP_INFO(this->get_logger(), "I heard:");
  if(msg->buttons[6] == 1)
  {
    robotstate = RobotState::RUNtoSHOOTpose;
  }
  if(msg->buttons[7] == 1)
  {
    robomas_frame_publisher_->publish(robomas::get_vel_frame(4, true));
    robomas_frame_publisher_->publish(robomas::get_stablepos_frame(5, true, 1.0f));
    roler_state = false;
  }else if(msg->buttons[0] == 1)
  {
    robomas_frame_publisher_->publish(robomas::get_dis_frame(4, false));
    robomas_frame_publisher_->publish(robomas::get_dis_frame(5, false));
  };
  if(msg->buttons[2] == 1)
  {
    if(pushed == false)
    {
      if(roler_state)
      {
        robomas_target4_publisher_->publish(robomas::get_target(0.0f));
        roler_state = false;
      }else{
        robomas_target4_publisher_->publish(robomas::get_target(300.0f));
        roler_state = true;
      }
      pushed = true;
    }
  }
  if(msg->buttons[1] == 1)
  {
    if(count==0){
      robomas_target5_publisher_->publish(robomas::get_target(M_PI/6));
      timestamp = msg->header.stamp;
      timestamp.sec += 1;
      count++;
    }else{
      if(msg->header.stamp > timestamp){
        robomas_frame_publisher_->publish(robomas::get_stablepos_frame(5, true, 1.0f));
        count = 0;
      }
    }
  }
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world chibarobo2024main package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<shooter>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <robomas_plugins/msg/robomas_frame.hpp>
#include <robomas_plugins/msg/robomas_target.hpp>
#include <robomas_plugins/robomas_utils.hpp>

using namespace std::chrono_literals;

enum class RobotState{
    STOPstartpose,
    RUNtoSHOOTpose,
    STOPshootpose,
    SHOOTING,
    RUNtoSTARTpose,
};


class shooter : public rclcpp::Node
{
    private:
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void pose2d_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
        void timer_callback();
        rclcpp::Publisher<robomas_plugins::msg::RobomasFrame>::SharedPtr robomas_frame_publisher_;
        rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr robomas_target4_publisher_;
        rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr robomas_target5_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr robot_vel_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose2d_subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
        int count = 0;
        builtin_interfaces::msg::Time timestamp;
        bool roler_state = false;
        bool pushed = false;
        geometry_msgs::msg::Pose2D pose2d;
        geometry_msgs::msg::Pose2D robot_target_vel;
        geometry_msgs::msg::Pose2D start_pose2d;
        geometry_msgs::msg::Pose2D shoot_pose2d;
        RobotState robotstate = RobotState::STOPstartpose;
        float kp_ = 1.0f;
        
    public:
        shooter() : Node("shooter") 
            {
                robomas_frame_publisher_ = this->create_publisher<robomas_plugins::msg::RobomasFrame>("robomas_frame", 10);
                robomas_target4_publisher_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target4", 10);
                robomas_target5_publisher_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target5", 10);
                robot_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("robot_vel", 10);
                joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&shooter::joy_callback, this, std::placeholders::_1));
                pose2d_subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>("pose2d", 10, std::bind(&shooter::pose2d_callback, this, std::placeholders::_1));
                timer_ = this->create_wall_timer(100ms, std::bind(&shooter::timer_callback, this));
                start_pose2d.set__x(0.35f).set__y(0.35f).set__theta(0.0f);
                shoot_pose2d.set__x(1.0f).set__y(0.30f).set__theta(66.57f);
            }
};
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class TurnController : public rclcpp::Node {
public:
    // Constructor with scene selection capability
    TurnController(int scene_number) 
        : Node("turn_controller_node"), scene_number_(scene_number) {
        
        // PID gains for turn controller - TUNED FOR OPTIMAL PERFORMANCE
        kp_turn_ = 2.0;    // Proportional gain - aggressive response to angle error
        ki_turn_ = 0.1;    // Integral gain - eliminate steady-state error
        kd_turn_ = 0.5;    // Derivative gain - smooth approach to target
        
        // Initialize PID variables
        integral_turn_ = 0.0;
        previous_error_turn_ = 0.0;
        previous_time_ = this->get_clock()->now();
        
        // Initialize current state
        current_yaw_ = 0.0;
        current_waypoint_index_ = 0;
        
        // Load waypoints based on scene
        SelectWaypoints();
        
        // Publishers and Subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rosbot_xl_base_controller/odom", 10,
            std::bind(&TurnController::odomCallback, this, std::placeholders::_1));
        
        // Control loop timer (40Hz for smooth control)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(25),
            std::bind(&TurnController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Turn Controller started with %zu waypoints", waypoints_.size());
        RCLCPP_INFO(this->get_logger(), "PID Gains - Kp: %.2f, Ki: %.2f, Kd: %.2f", kp_turn_, ki_turn_, kd_turn_);
    }

private:
    // Scene selection and waypoint loading
    void SelectWaypoints() {
        switch (scene_number_) {
            case 1:
                waypoints_ = {
                    {0.0, 0.0, -0.532},   // initial point -> WP1: Turn 24.8 degrees clockwise
                    {0.0, 0.0, +0.432},   // WP1 -> WP2: Turn 19 degrees counter-clockwise
                    {0.0, 0.0, +0.464},   // WP2 -> WP3: Turn 20.8 degrees counter-clockwise
                    {0.0, 0.0, -0.364}   // WP3 -> initial point 15.1 degrees clockwise
                };
                RCLCPP_INFO(this->get_logger(), "Loaded simulation waypoints");
                break;
            
            case 2:
                waypoints_ = {
                    {0.0, 0.0, -0.955},   // initial point -> WP1: Turn 54.7 degrees clockwise
                    {0.0, 0.0, -0.315},   // WP1 -> WP2: Turn 18 degrees clockwise  
                    {0.0, 0.0, +1.100}    // WP2 -> initial point: Turn 63 degrees counter-clockwise
                };
                RCLCPP_INFO(this->get_logger(), "Loaded real robot waypoints");
                break;
            
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid Scene Number: %d", scene_number_);
                waypoints_ = {{0.0, 0.0, 0.0}};

        }
        
    }
    // Odometry callback - extract current yaw angle
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Convert quaternion to yaw angle
        tf2::Quaternion quat(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        
        tf2::Matrix3x3 matrix(quat);
        double roll, pitch;
        matrix.getRPY(roll, pitch, current_yaw_);
    }
    
    // Main control loop
    void controlLoop() {
        if (current_waypoint_index_ >= waypoints_.size()) {
            // All waypoints completed
            stopRobot();
            RCLCPP_INFO(this->get_logger(), "All turn waypoints completed! Robot stopped.");
            rclcpp::shutdown();
            return;
        }
        
        // Get current waypoint turn angle (relative angle to turn)
        double target_turn_angle = waypoints_[current_waypoint_index_][2];
        
        // Calculate angle error (how much more to turn)
        static double accumulated_angle = 0.0;  // Track total rotation for this waypoint
        static double target_absolute_angle = current_yaw_;  // Target absolute orientation
        static bool waypoint_started = false;
        
        // Initialize target for new waypoint
        if (!waypoint_started) {
            target_absolute_angle = current_yaw_ + target_turn_angle;
            accumulated_angle = 0.0;
            waypoint_started = true;
            RCLCPP_INFO(this->get_logger(), 
                       "Starting waypoint%zu: Turn %.1f° (%.3f rad) from current orientation",
                       current_waypoint_index_ + 1, 
                       target_turn_angle * 180.0 / M_PI, target_turn_angle);
        }
        
        // Normalize target angle to [-π, π]
        target_absolute_angle = normalizeAngle(target_absolute_angle);
        
        // Calculate angle error
        double angle_error = normalizeAngle(target_absolute_angle - current_yaw_);
        
        // PID Controller for turning
        auto current_time = this->get_clock()->now();
        double dt = (current_time - previous_time_).seconds();
        
        if (dt > 0.0) {
            // Integral term (with windup protection)
            integral_turn_ += angle_error * dt;
            integral_turn_ = std::max(-1.0, std::min(1.0, integral_turn_)); // Clamp integral
            
            // Derivative term
            double derivative_turn = (angle_error - previous_error_turn_) / dt;
            
            // PID control signal
            double control_signal = kp_turn_ * angle_error + 
                                  ki_turn_ * integral_turn_ + 
                                  kd_turn_ * derivative_turn;
            
            // Apply velocity limits for safety
            control_signal = std::max(-2.0, std::min(2.0, control_signal));
            
            // Check if waypoint is reached
            if (std::abs(angle_error) < 0.05) { // 3 degree tolerance
                stopRobot();
                
                RCLCPP_INFO(this->get_logger(), 
                           "Waypoint %zu completed! Turned %.1f° (target: %.1f°)",
                           current_waypoint_index_ + 1,
                           accumulated_angle * 180.0 / M_PI,
                           target_turn_angle * 180.0 / M_PI);
                
                // Wait at waypoint for 2 seconds before proceeding
                static auto stop_time = this->get_clock()->now();
                static bool stopping = false;
                
                if (!stopping) {
                    stop_time = this->get_clock()->now();
                    stopping = true;
                }
                
                if ((this->get_clock()->now() - stop_time).seconds() > 2.0) {
                    current_waypoint_index_++;
                    waypoint_started = false;
                    stopping = false;
                    
                    // Reset PID for next waypoint
                    integral_turn_ = 0.0;
                    previous_error_turn_ = 0.0;
                }
            } else {
                // Send turn command - TURN IN PLACE ONLY
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;   // No forward/backward movement
                cmd_vel.linear.y = 0.0;   // No sideways movement  
                cmd_vel.angular.z = control_signal;  // Pure rotation
                
                cmd_vel_pub_->publish(cmd_vel);
                
                // Debug output every second
                static int debug_counter = 0;
                if (debug_counter++ % 40 == 0) {
                    RCLCPP_INFO(this->get_logger(), 
                               "Waypoint %zu: angle_err=%.1f°, ctrl=%.3f, current_yaw=%.1f°, target=%.1f°",
                               current_waypoint_index_ + 1, 
                               angle_error * 180.0 / M_PI,
                               control_signal,
                               current_yaw_ * 180.0 / M_PI,
                               target_absolute_angle * 180.0 / M_PI);
                }
            }
            
            // Update for next iteration
            previous_error_turn_ = angle_error;
            previous_time_ = current_time;
        }
    }
    
    // Stop robot completely
    void stopRobot() {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    // Normalize angle to [-π, π] range
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    // Member variables
    int scene_number_;
    
    // PID gains for turn controller
    double kp_turn_, ki_turn_, kd_turn_;
    
    // PID state variables
    double integral_turn_;
    double previous_error_turn_;
    rclcpp::Time previous_time_;
    
    // Robot state
    double current_yaw_;
    size_t current_waypoint_index_;
    
    // Waypoints [x, y, turn_angle_in_radians]
    std::vector<std::vector<double>> waypoints_;
    
    // ROS2 interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // Check if a scene number argument is provided
    int scene_number = 1; // Default to simulation
    if (argc > 1) {
        scene_number = std::atoi(argv[1]);
    }
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Turn Controller (Scene: %d)", scene_number);
    
    auto node = std::make_shared<TurnController>(scene_number);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
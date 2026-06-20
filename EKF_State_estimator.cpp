#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Include our pure C++ math library
#include "EKFCore.hpp"

using std::placeholders::_1;

class StateEstimatorNode : public rclcpp::Node {
public:
    StateEstimatorNode() : Node("state_estimator_node"), ekf_(0.05, 0.1) {
        // Declare hardware-agnostic frame parameters
        odom_frame_ = this->declare_parameter("odom_frame", "odom");
        base_frame_ = this->declare_parameter("base_frame", "base_link");
        publish_tf_ = this->declare_parameter("publish_tf", true);

        // State Tracking variables
        gx_ = 0.0; gy_ = 0.0; gyaw_ = 0.0;
        last_time_ = this->now();
        has_meas_ = false;

        // Imu LPF variables
        y_raw_ = Eigen::Vector3d::Zero();

        // Standardized Subscriptions
        reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "~/reset", 10, [this](std_msgs::msg::Empty::SharedPtr) {
                gx_ = 0.0; gy_ = 0.0; gyaw_ = 0.0;
                ekf_.reset();
                RCLCPP_INFO(this->get_logger(), "EKF: State and Covariance Reset.");
            });

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                // Low-Pass Filter for raw hardware vibration
                y_raw_(0) = (0.8 * y_raw_(0)) + (0.2 * msg->linear_acceleration.x);
                y_raw_(1) = (0.8 * y_raw_(1)) + (0.2 * msg->linear_acceleration.y);
                y_raw_(2) = (0.8 * y_raw_(2)) + (0.2 * msg->angular_velocity.z); 
            });

        // Subscribes to standard vehicle velocity instead of simulator-specific joint names
        vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "vehicle/velocity", rclcpp::SensorDataQoS(), [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                latest_vx_meas_ = msg->twist.linear.x;
                has_meas_ = true;
            });

        state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Run EKF clock at 100Hz (10ms)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&StateEstimatorNode::runEKF, this));
        
        RCLCPP_INFO(this->get_logger(), "Hardware-Agnostic EKF State Estimator Online.");
    }

private:
    EKFCore ekf_; // Pure math engine

    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;

    Eigen::Vector3d y_raw_;
    double gx_, gy_, gyaw_;
    double latest_vx_meas_;
    bool has_meas_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void runEKF() {
        double dt = (this->now() - last_time_).seconds();
        if (dt <= 0 || dt > 0.1) { last_time_ = this->now(); return; }
        last_time_ = this->now();

        // 1. Predict Step
        ekf_.predict(y_raw_(0), y_raw_(1), y_raw_(2), dt);

        // 2. Update Step
        if (has_meas_) {
            ekf_.update(latest_vx_meas_);
            has_meas_ = false;
        }

        // 3. Integrate 2D Pose
        Eigen::Vector3d state = ekf_.getState();
        double vx = state(0);
        double vy = state(1);
        double r  = state(2);

        gyaw_ += r * dt;
        gyaw_ = std::atan2(std::sin(gyaw_), std::cos(gyaw_));

        gx_ += (vx * std::cos(gyaw_) - vy * std::sin(gyaw_)) * dt;
        gy_ += (vx * std::sin(gyaw_) + vy * std::cos(gyaw_)) * dt;

        // 4. Publish Results
        publishOdometry(state);
        
        if (publish_tf_) {
            publishTF();
        }
    }

    void publishOdometry(const Eigen::Vector3d& state) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        
        odom.pose.pose.position.x = gx_;
        odom.pose.pose.position.y = gy_;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, gyaw_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = state(0);
        odom.twist.twist.linear.y = state(1);
        odom.twist.twist.angular.z = state(2);

        // Note: You can populate odom.pose.covariance from ekf_.getCovariance() here!

        state_pub_->publish(odom);
    }

    void publishTF() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = odom_frame_;
        t.child_frame_id = base_frame_; 
        
        t.transform.translation.x = gx_;
        t.transform.translation.y = gy_;
        t.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, gyaw_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(t);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}

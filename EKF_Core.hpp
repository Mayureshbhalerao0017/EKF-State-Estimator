#pragma once

#include <eigen3/Eigen/Dense>
#include <cmath>

class EKFCore {
public:
    EKFCore(double process_noise_std = 0.05, double meas_noise_std = 0.1) {
        Q_ = Eigen::Matrix3d::Identity() * process_noise_std;
        
        // R matrix for [vx, vy] update (Non-Holonomic Constraint)
        R_ = Eigen::Matrix2d::Zero();
        R_(0, 0) = meas_noise_std; // Forward velocity variance
        R_(1, 1) = 0.01;           // Lateral velocity variance (slip allowance)
        
        reset();
    }

    void reset() {
        x_hat_ = Eigen::Vector3d::Zero(); // [vx, vy, yaw_rate]
        P_ = Eigen::Matrix3d::Identity() * 0.1;
    }

    // Step 1: Predict state forward in time using IMU data
    void predict(double ax, double ay, double yaw_rate, double dt) {
        if (dt <= 0.0) return;

        // Jacobian Matrix (F) for the dynamic model
        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0, 1) = yaw_rate * dt;
        F(1, 0) = -yaw_rate * dt;

        // Predict State
        x_hat_(0) += (ax + yaw_rate * x_hat_(1)) * dt; 
        x_hat_(1) += (ay - yaw_rate * x_hat_(0)) * dt;
        x_hat_(2) = yaw_rate; // Direct feedthrough for yaw rate

        // Predict Covariance
        P_ = F * P_ * F.transpose() + Q_;
    }

    // Step 2: Correct state using longitudinal velocity measurement
    void update(double vx_meas) {
        // H Matrix maps state [vx, vy, r] to measurements [vx, vy]
        Eigen::Matrix<double, 2, 3> H;
        H << 1.0, 0.0, 0.0,   
             0.0, 1.0, 0.0;   
        
        // Innovation: z(0) is measured wheel speed, z(1) is 0.0 (Non-Holonomic constraint)
        Eigen::Vector2d z(vx_meas, 0.0);
        Eigen::Vector2d y = z - H * x_hat_; 
        
        // Innovation Covariance (S)
        Eigen::Matrix2d S = H * P_ * H.transpose() + R_;
        
        // Kalman Gain (K)
        Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();
        
        // Update State and Covariance
        x_hat_ = x_hat_ + K * y;
        P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
    }

    // Getters
    Eigen::Vector3d getState() const { return x_hat_; }
    Eigen::Matrix3d getCovariance() const { return P_; }

private:
    Eigen::Vector3d x_hat_; // State vector: [vx, vy, yaw_rate]
    Eigen::Matrix3d P_;     // State covariance
    Eigen::Matrix3d Q_;     // Process noise covariance
    Eigen::Matrix2d R_;     // Measurement noise covariance
};

# 🚀 EKF State Estimator

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![C++](https://img.shields.io/badge/C++-17-orange)
![Eigen](https://img.shields.io/badge/Eigen-3.4-green)

A professional-grade Extended Kalman Filter (EKF) for 2D autonomous vehicle state estimation. This package fuses IMU data (acceleration and yaw rate) with hardware velocity encoders to provide a smooth, continuous, zero-latency odometry estimate.

## 🏗️ Software Architecture: The Wrapper Pattern

This repository is strictly designed using the **Wrapper Pattern** to ensure maximum portability, testability, and hardware agnosticism. It is split into two distinct layers:

### 1. The Pure Math Core (`include/EKFCore.hpp`)
A pure, vanilla C++ library powered by `Eigen3`. 
* **Zero Middleware:** Contains absolutely no ROS 2 dependencies. 
* **Portability:** Can be compiled directly into proprietary middleware, embedded systems, or bare-metal microcontrollers tomorrow.
* **Math:** Implements a dynamic non-holonomic kinematic bicycle model, predicting state via IMU integration and applying measurement updates via high-frequency velocity data.

### 2. The ROS 2 Wrapper (`src/ekf_node.cpp`)
A lightweight ROS 2 interface that handles the TF trees, ROS parameter servers, and message serialization.
* **Hardware Agnostic:** Subscribes to standard `geometry_msgs::msg::TwistStamped` rather than simulator-specific joint states. This node can be dropped into any robot (drones, rovers, autonomous racecars) that publishes standard ROS velocity topics.
* **Configurable:** Uses ROS parameters to define dynamic TF frames (`odom` -> `base_link`).

---

## 📡 Node Interfaces

### Subscribed Topics
* `imu/data` ([sensor_msgs/Imu](http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)): Raw IMU data. The node internally passes this through a Low-Pass Filter (LPF) to eliminate physical hardware vibration and chassis jitter.
* `vehicle/velocity` ([geometry_msgs/TwistStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/TwistStamped.html)): The measured forward velocity from wheel encoders or external sensors.
* `~/reset` ([std_msgs/Empty](http://docs.ros.org/en/api/std_msgs/html/msg/Empty.html)): Resets the EKF covariance and state matrices back to origin.

### Published Topics
* `odometry/filtered` ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html)): The fused, high-frequency state estimate.
* `tf`: Broadcasts the transform from `odom_frame` to `base_frame`.

### ROS 2 Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `odom_frame` | `string` | `"odom"` | The parent coordinate frame. |
| `base_frame` | `string` | `"base_link"` | The child coordinate frame attached to the robot. |
| `publish_tf` | `bool` | `true` | Whether the node should actively broadcast to `/tf`. |

---

## ⚙️ Prerequisites

* **OS:** Ubuntu 22.04
* **Middleware:** [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
* **Math Library:** `Eigen3`

To install Eigen locally:
```bash
sudo apt-get update
sudo apt-get install libeigen-dev

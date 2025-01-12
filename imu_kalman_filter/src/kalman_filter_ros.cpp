/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

  @section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holder nor the names of its
        contributors may be used to endorse or promote products derived from
        this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "imu_kalman_filter/kalman_filter_ros.h"
#include "imu_kalman_filter/imu.h"
#include "imu_kalman_filter/tic_toc.h"

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

namespace imu_tools {

KalmanFilterROS::KalmanFilterROS(const ros::NodeHandle &nh,
                                 const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), initialized_filter_(false) {
    ROS_INFO("Starting KalmanFilterROS");
    initializeParams();

    last_ros_time_ = ros::Time::now().toSec();

    int queue_size = 5;

    // Register publishers:
    imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("/data", queue_size);

    if (publish_debug_topics_) {
        rpy_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
            ros::names::resolve("imu") + "/rpy/filtered", queue_size);

        // if (filter_.getDoBiasEstimation()) {
        //     state_publisher_ = nh_.advertise<std_msgs::Bool>(
        //         ros::names::resolve("imu") + "/steady_state", queue_size);
        // }
    }

    // Register IMU raw data subscriber.
    imu_subscriber_.reset(new ImuSubscriber(nh_, "/camera/imu", queue_size));

    // Register magnetic data subscriber.
    imu_subscriber_->registerCallback(&KalmanFilterROS::imuCallback, this);
}

KalmanFilterROS::~KalmanFilterROS() { ROS_INFO("Destroying KalmanFilterROS"); }

void KalmanFilterROS::initializeParams() {
    double gain_acc;
    double gain_mag;
    bool do_bias_estimation;
    double bias_alpha;
    bool do_adaptive_gain;

    if (!nh_private_.getParam("fixed_frame", fixed_frame_))
        fixed_frame_ = "odom";
    if (!nh_private_.getParam("publish_tf", publish_tf_))
        publish_tf_ = true;
    if (!nh_private_.getParam("reverse_tf", reverse_tf_))
        reverse_tf_ = false;
    if (!nh_private_.getParam("publish_debug_topics", publish_debug_topics_))
        publish_debug_topics_ = false;
    if (!nh_private_.getParam("gain_acc", gain_acc))
        gain_acc = 0.01;
    if (!nh_private_.getParam("gain_mag", gain_mag))
        gain_mag = 0.01;
    if (!nh_private_.getParam("do_bias_estimation", do_bias_estimation))
        do_bias_estimation = true;
    if (!nh_private_.getParam("bias_alpha", bias_alpha))
        bias_alpha = 0.01;
    if (!nh_private_.getParam("do_adaptive_gain", do_adaptive_gain))
        do_adaptive_gain = true;

    double orientation_stddev;
    if (!nh_private_.getParam("orientation_stddev", orientation_stddev))
        orientation_stddev = 0.0;

    orientation_variance_ = orientation_stddev * orientation_stddev;
}

void KalmanFilterROS::imuCallback(const ImuMsg::ConstPtr &imu_msg_raw) {

    static double prev_time = 0;
    if (imu_msg_raw->header.stamp.toSec() < prev_time ||
        std::abs(imu_msg_raw->header.stamp.toSec() - prev_time) < 1e-6) {
        std::cout << __LINE__ << " : imu time error" << std::endl;
    }

    Eigen::Vector3d acce{imu_msg_raw->linear_acceleration.x,
                         imu_msg_raw->linear_acceleration.y,
                         imu_msg_raw->linear_acceleration.z};
    Eigen::Vector3d gyro{imu_msg_raw->angular_velocity.x,
                         imu_msg_raw->angular_velocity.y,
                         imu_msg_raw->angular_velocity.z};
    const double &time = imu_msg_raw->header.stamp.toSec();

    // IMU imu_data{time, gyro, acce};

    static bool is_first = true;
    static Eigen::Vector3d first_acc_data = acce;
    if (is_first) {
        // 第一次不用卡尔曼滤波，直接发布出去，
        sensor_msgs::Imu M;
        M.linear_acceleration.x = acce.x();
        M.linear_acceleration.y = acce.y();
        M.linear_acceleration.z = acce.z();
        M.header = imu_msg_raw->header;
        imu_publisher_.publish(M);
        is_first = false;
        std::cout << "first farm" << std::endl;
    } else {
        TicToc kalman_use_time;
        Eigen::Vector3d x_new = kalman_filter_.predictAndupdate(
            first_acc_data, acce); // z当前时刻的量测值，X
        std::cout << "kalman_use_time " << kalman_use_time.toc() << " ms"
                  << std::endl;
        first_acc_data = x_new;

        sensor_msgs::Imu M;
        M.linear_acceleration.x = x_new(0, 0);
        M.linear_acceleration.y = x_new(1, 0);
        M.linear_acceleration.z = x_new(2, 0);
        M.header = imu_msg_raw->header;
        imu_publisher_.publish(M);
    }

    // // Initialize.
    // if (!initialized_filter_) {
    //     time_prev_ = time;
    //     initialized_filter_ = true;
    //     return;
    // }

    // // determine dt: either constant, or from IMU timestamp
    // double dt = time - time_prev_;

    // time_prev_ = time;

    // // Update the filter.
    // // filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);

    // // Publish state.
    // publish(imu_msg_raw);
}

tf::Quaternion KalmanFilterROS::hamiltonToTFQuaternion(double q0, double q1,
                                                       double q2,
                                                       double q3) const {
    // ROS uses the Hamilton quaternion convention (q0 is the scalar). However,
    // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
    return tf::Quaternion(q1, q2, q3, q0);
}

void KalmanFilterROS::publish(const sensor_msgs::Imu::ConstPtr &imu_msg_raw) {
    // // Get the orientation:
    // double q0, q1, q2, q3;
    // filter_.getOrientation(q0, q1, q2, q3);
    // tf::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

    // // Create and publish fitlered IMU message.
    // boost::shared_ptr<sensor_msgs::Imu> imu_msg =
    //     boost::make_shared<sensor_msgs::Imu>(*imu_msg_raw);
    // tf::quaternionTFToMsg(q, imu_msg->orientation);

    // imu_msg->orientation_covariance[0] = orientation_variance_;
    // imu_msg->orientation_covariance[1] = 0.0;
    // imu_msg->orientation_covariance[2] = 0.0;
    // imu_msg->orientation_covariance[3] = 0.0;
    // imu_msg->orientation_covariance[4] = orientation_variance_;
    // imu_msg->orientation_covariance[5] = 0.0;
    // imu_msg->orientation_covariance[6] = 0.0;
    // imu_msg->orientation_covariance[7] = 0.0;
    // imu_msg->orientation_covariance[8] = orientation_variance_;

    // // Account for biases.
    // if (filter_.getDoBiasEstimation()) {
    //     imu_msg->angular_velocity.x -= filter_.getAngularVelocityBiasX();
    //     imu_msg->angular_velocity.y -= filter_.getAngularVelocityBiasY();
    //     imu_msg->angular_velocity.z -= filter_.getAngularVelocityBiasZ();
    // }

    // imu_publisher_.publish(imu_msg);

    // if (publish_debug_topics_) {
    //     // Create and publish roll, pitch, yaw angles
    //     geometry_msgs::Vector3Stamped rpy;
    //     rpy.header = imu_msg_raw->header;

    //     tf::Matrix3x3 M;
    //     M.setRotation(q);
    //     M.getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
    //     rpy_publisher_.publish(rpy);

    //     // Publish whether we are in the steady state, when doing bias
    //     // estimation
    //     if (filter_.getDoBiasEstimation()) {
    //         std_msgs::Bool state_msg;
    //         state_msg.data = filter_.getSteadyState();
    //         state_publisher_.publish(state_msg);
    //     }
    // }

    // if (publish_tf_) {
    //     // Create and publish the ROS tf.
    //     tf::Transform transform;
    //     transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    //     transform.setRotation(q);

    //     if (reverse_tf_) {
    //         tf_broadcaster_.sendTransform(tf::StampedTransform(
    //             transform.inverse(), imu_msg_raw->header.stamp,
    //             imu_msg_raw->header.frame_id, fixed_frame_));
    //     } else {
    //         tf_broadcaster_.sendTransform(tf::StampedTransform(
    //             transform, imu_msg_raw->header.stamp, fixed_frame_,
    //             imu_msg_raw->header.frame_id));
    //     }
    // }
}

void KalmanFilterROS::reset() {
    time_prev_ = {};
    last_ros_time_ = ros::Time::now().toSec();
    initialized_filter_ = false;
}

} // namespace imu_tools

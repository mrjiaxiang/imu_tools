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

#pragma once

#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "imu_kalman_filter/kalman_filter.h"

namespace imu_tools {

class KalmanFilterROS {
  public:
    KalmanFilterROS(const ros::NodeHandle &nh,
                    const ros::NodeHandle &nh_private);
    virtual ~KalmanFilterROS();

    // Reset the filter to the initial state.
    void reset();

  private:
    // Convenience typedefs
    typedef sensor_msgs::Imu ImuMsg;
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;

    // ROS-related variables.
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    boost::shared_ptr<ImuSubscriber> imu_subscriber_;

    ros::Publisher imu_publisher_;
    ros::Publisher rpy_publisher_;
    ros::Publisher state_publisher_;
    tf::TransformBroadcaster tf_broadcaster_;

    // Parameters:
    bool use_mag_;
    bool publish_tf_;
    bool reverse_tf_;
    bool publish_debug_topics_;
    std::string fixed_frame_;
    double orientation_variance_;

    KalmanFilter kalman_filter_;

    // State variables:
    double time_prev_;
    double last_ros_time_;
    bool initialized_filter_;

    void initializeParams();
    void imuCallback(const ImuMsg::ConstPtr &imu_msg_raw);
    void publish(const sensor_msgs::Imu::ConstPtr &imu_msg_raw);

    tf::Quaternion hamiltonToTFQuaternion(double q0, double q1, double q2,
                                          double q3) const;
};

} // namespace imu_tools

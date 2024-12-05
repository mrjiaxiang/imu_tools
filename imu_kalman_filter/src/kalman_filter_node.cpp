#include <ros/ros.h>

#include "imu_kalman_filter/kalman_filter_ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "KalmanFilterROS");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    imu_tools::KalmanFilterROS filter(nh, nh_private);
    ros::spin();
    return 0;
}

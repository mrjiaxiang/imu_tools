#include "imu_kalman_filter/kalman_filter.h"
#include <iostream>

using namespace std;

Eigen::Vector3d KalmanFilter::predictAndupdate(Eigen::Vector3d x,
                                               Eigen::Vector3d z) {
    if (!isinitized_) {
        P = Eigen::Matrix3d::Identity(); // 协方差的初始化
        isinitized_ = true;
    }
    x = A * x;                       // 状态一步预测方程
    P = A * P * (A.transpose()) + Q; // 一步预测协方差阵
    Eigen::MatrixXd K = P * (H.transpose()) *
                        ((H * P * (H.transpose()) + R).inverse()); // kalman增益
    x = x + K * (z - H * x); // 状态更新：
    int x_size = x.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P = (I - K * H) * P; // 协方差阵更新：
    return x;
}

KalmanFilter::KalmanFilter() {
    // 参数初始化设置
    // 系统状态矩阵
    A = Eigen::Matrix3d::Identity();
    H = Eigen::Matrix3d::Identity();        // 测量矩阵
    Q = 0.01 * Eigen::Matrix3d::Identity(); // （预测）过程噪音
    R = 3.65 * Eigen::Matrix3d::Identity(); // 真实传感器噪音
}

KalmanFilter::~KalmanFilter() {}

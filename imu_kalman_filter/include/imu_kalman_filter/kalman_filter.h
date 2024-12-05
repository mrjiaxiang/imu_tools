#pragma once

#include <Eigen/Dense>

class KalmanFilter {
  public:
    KalmanFilter();
    Eigen::Vector3d predictAndupdate(Eigen::Vector3d x, Eigen::Vector3d z);
    ~KalmanFilter();

  private:
    Eigen::MatrixX3d A; // 系统状态矩阵
    Eigen::MatrixX3d P; // 协方差
    Eigen::MatrixX3d Q; // 测量过程噪音（预测）
    Eigen::MatrixX3d R; // 真实传感器噪音
    Eigen::MatrixX3d H; // 测量矩阵

    bool isinitized_{false};
};
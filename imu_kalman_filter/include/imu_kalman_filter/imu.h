//
// Created by xiang on 2021/7/19.
//

#pragma once

#include <memory>
#include <Eigen/Core>

/// IMU 读数
struct IMU {
    IMU() = default;
    IMU(double t, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acce)
        : timestamp_(t), gyro_(gyro), acce_(acce) {}

    double timestamp_ = 0.0;
    Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acce_ = Eigen::Vector3d::Zero();
};

using IMUPtr = std::shared_ptr<IMU>;

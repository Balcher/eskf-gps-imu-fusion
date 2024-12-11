/*
 * @Description:
 * @Author: BiChunkai 321521004@qq.com
 * @Date: 2024-12-10 10:43:45
 * @FilePath: /eskf-gps-imu-fusion/include/imu_data.h
 *
 * Copyright (c) 2024 by 无锡捷普迅科技有限公司, All Rights Reserved.
 */
//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_IMU_DATA_H
#define GPS_IMU_FUSION_IMU_DATA_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class IMUData
{
public:
    IMUData() = default;

    double time = 0.0;                                        // 记录IMU采集的时间戳
    Eigen::Vector3d linear_accel = Eigen::Vector3d::Zero();   // 测量到的线性加速度（加速度计输出）
    Eigen::Vector3d angle_velocity = Eigen::Vector3d::Zero(); // 测量到的角速度（陀螺仪输出）

    Eigen::Vector3d true_linear_accel = Eigen::Vector3d::Zero();   // 真实的线性加速度
    Eigen::Vector3d true_angle_velocity = Eigen::Vector3d::Zero(); // 真实的角速度

    Eigen::Quaterniond true_q_enu = Eigen::Quaterniond::Identity(); // 真实的四元数姿态（ENU坐标系下的旋转）
    Eigen::Vector3d true_t_enu = Eigen::Vector3d::Zero();           // 真实的位移（ENU坐标系下的平移量）
};

#endif // GPS_IMU_FUSION_IMU_DATA_H

//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_GPS_TOOL_H
#define GPS_IMU_FUSION_GPS_TOOL_H

#include "gps_data.h"
#include "Geocentric/LocalCartesian.hpp"

#include <deque>
#include <vector>

/**
 * @brief 用于处理与GPS数据相关的工具类
 */
class GPSTool
{
public:
    GPSTool() = delete; // 禁用默认构造函数

    /**
     * @brief Construct a new GPSTool object
     *
     * @param lon 经度
     * @param lat 纬度
     * @param altitude 高度
     */
    GPSTool(double lon, double lat, double altitude);

    /**
     * 将 GPS 数据从经纬度高度（LLA）坐标系转换为本地东北天（NED）坐标系
     * @param gps_data 包含 GPS 数据的结构体，包括LLA 坐标和转换后的 NED 坐标
     */
    void LLAToLocalNED(GPSData &gps_data);

    /**
     * 将给定的三维向量从本地东北天（NED）坐标系转换为经纬度高度（LLA）坐标系
     * @param ned 输入的三维向量，在本地东北天（NED）坐标系下
     * @return 转换后的三维向量，在经纬度高度（LLA）坐标系下
     */
    Eigen::Vector3d LLAToLocalNED(const Eigen::Vector3d &lla);

    /**
     * @brief 读取GPS数据文件，并将其转换为本地东北天（NED）坐标系下的数据，然后存储在一个向量中
     * @param path 数据文件所在的路径
     * @param gps_data_vec 存储GPS数据的向量
     * @param skip_rows 跳过的行数
     */
    void ReadGPSData(const std::string &path, std::vector<GPSData> &gps_data_vec, int skip_rows = 1);

    /**
     * @brief 读取GPS数据文件，并将其转换为本地东北天（NED）坐标系下的数据，然后存储在一个双端队列中
     * @param path 数据文件所在的路径
     * @param gps_data_vec 存储GPS数据的双端队列
     * @param skip_rows 跳过的行数
     */
    void ReadGPSData(const std::string &path, std::deque<GPSData> &gps_data_vec, int skip_rows = 1);

private:
    GeographicLib::LocalCartesian geo_converter_; // only support ENU，东-北-天坐标系（ENU）
};

#endif // GPS_IMU_FUSION_GPS_TOOL_H

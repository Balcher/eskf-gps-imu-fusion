//
// Created by Zhang Zhimeng on 23-8-16.
//

#ifndef GPS_IMU_FUSION_CONFIG_PARAMETERS_H
#define GPS_IMU_FUSION_CONFIG_PARAMETERS_H

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

/**
 * @brief 加载配置参数的类
 */
class ConfigParameters
{
public:
    ConfigParameters() = default; // 默认构造函数

    void LoadParameters(const std::string &config_file_path)
    {
        try
        {
            YAML::Node config_node = YAML::LoadFile(config_file_path);

            earth_rotation_speed_ = config_node["earth_rotation_speed"].as<double>();
            earth_gravity_ = config_node["earth_gravity"].as<double>();
            ref_longitude_ = config_node["ref_longitude"].as<double>();
            ref_latitude_ = config_node["ref_latitude"].as<double>();
            ref_altitude_ = config_node["ref_altitude"].as<double>();
            position_error_prior_std_ = config_node["position_error_prior_std"].as<double>();
            velocity_error_prior_std_ = config_node["velocity_error_prior_std"].as<double>();
            rotation_error_prior_std_ = config_node["rotation_error_prior_std"].as<double>();
            accelerometer_bias_error_prior_std_ = config_node["accelerometer_bias_error_prior_std"].as<double>();
            gyro_bias_error_prior_std_ = config_node["gyro_bias_error_prior_std"].as<double>();
            gyro_noise_std_ = config_node["gyro_noise_std"].as<double>();
            accelerometer_noise_std_ = config_node["accelerometer_noise_std"].as<double>();
            gps_position_x_std_ = config_node["gps_position_x_std"].as<double>();
            gps_position_y_std_ = config_node["gps_position_y_std"].as<double>();
            gps_position_z_std_ = config_node["gps_position_z_std"].as<double>();
            only_prediction_ = config_node["only_prediction"].as<bool>();
            use_earth_model_ = config_node["use_earth_model"].as<bool>();

            LOG(INFO) << "####### Config Parameters #######";
            LOG(INFO) << "earth_rotation_speed: " << earth_rotation_speed_;
            LOG(INFO) << "earth_gravity: " << earth_gravity_;
            LOG(INFO) << "ref_longitude: " << ref_longitude_;
            LOG(INFO) << "ref_latitude: " << ref_latitude_;
            LOG(INFO) << "ref_altitude: " << ref_altitude_;
            LOG(INFO) << "position_error_prior_std: " << position_error_prior_std_;
            LOG(INFO) << "velocity_error_prior_std: " << velocity_error_prior_std_;
            LOG(INFO) << "rotation_error_prior_std: " << rotation_error_prior_std_;
            LOG(INFO) << "accelerometer_bias_error_prior_std: " << accelerometer_bias_error_prior_std_;
            LOG(INFO) << "gyro_bias_error_prior_std: " << gyro_bias_error_prior_std_;
            LOG(INFO) << "gyro_noise_std: " << gyro_noise_std_;
            LOG(INFO) << "accelerometer_noise_std: " << accelerometer_noise_std_;
            LOG(INFO) << "gps_position_x_std: " << gps_position_x_std_;
            LOG(INFO) << "gps_position_y_std: " << gps_position_y_std_;
            LOG(INFO) << "gps_position_z_std: " << gps_position_z_std_;
            LOG(INFO) << "only_prediction: " << only_prediction_;
            LOG(INFO) << "use_earth_model: " << use_earth_model_;
            LOG(INFO) << std::endl;
        }
        catch (...)
        {
            LOG(FATAL) << "Load config parameters failure, path: " << config_file_path;
        }
    }

public:
    // 初始化方法使用的C++11的列表初始化方法
    // earth
    double earth_rotation_speed_{}; // 地球自转角速度
    double earth_gravity_{};        // 地球重力加速度

    // reference point
    double ref_longitude_{}; // 参考经度
    double ref_latitude_{};  // 参考纬度
    double ref_altitude_{};  // 参考高度

    // kalman prediction process std
    double position_error_prior_std_{};           // 位置误差先验标准差
    double velocity_error_prior_std_{};           // 速度误差先验标准差
    double rotation_error_prior_std_{};           // 旋转误差先验标准差
    double accelerometer_bias_error_prior_std_{}; // 加速度计偏置误差先验标准差
    double gyro_bias_error_prior_std_{};          // 陀螺仪偏置误差先验标准差

    // imu sensor noise
    double gyro_noise_std_{};          // 陀螺仪噪声标准差
    double accelerometer_noise_std_{}; // 加速度计噪声标准差

    // kalman measurement process std
    double gps_position_x_std_{}; // GPS位置测量标准差
    double gps_position_y_std_{}; // GPS位置测量标准差
    double gps_position_z_std_{}; // GPS位置测量标准差

    // only using IMU to integration
    bool only_prediction_{}; // 仅使用IMU进行积分

    bool use_earth_model_{}; // 使用地球模型
};

#endif // GPS_IMU_FUSION_CONFIG_PARAMETERS_H

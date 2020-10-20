#include "localization_wrapper.h"

#include <iomanip>

#include <glog/logging.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "imu_gps_localizer/base_type.h"

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);

    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    const Eigen::Vector3d I_p_Gps(x, y, z);

    std::string log_folder = "/home";
    ros::param::get("log_folder", log_folder);

    // Log.
    file_state_.open(log_folder + "/state.csv");
    file_gps_.open(log_folder +"/gps.csv");

    // Initialization imu gps localizer.
    imu_gps_localizer_ptr_ = 
        std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                              acc_bias_noise, gyro_bias_noise,
                                                              I_p_Gps);

    // Subscribe topics.
    imu_sub_ = nh.subscribe("/imu/data", 100,  &LocalizationWrapper::ImuCallback, this);
    gps_position_sub_ = nh.subscribe("/nmea_sentence", 100,  &LocalizationWrapper::GpsPositionCallback, this);

    state_pub_ = nh.advertise<nav_msgs::Path>("fused_path", 10);
}

LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    file_gps_.close();
}

void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;
    
    ImuGpsLocalization::State fused_state;
    const bool ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
    if (!ok) {
        return;
    }

    // Publish fused state.
    ConvertStateToRosTopic(fused_state);
    state_pub_.publish(ros_path_);

    std::cout << "current state is: " << fused_state.G_p_I.transpose() << std::endl;

    // Log fused state.
    LogState(fused_state);
}

void LocalizationWrapper::GpsPositionCallback(const nmea_msgs::SentenceConstPtr& gps_msg_ptr) {
    // Check the gps_status.
//    if (gps_msg_ptr->status.status != 2) {
//        LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
//        return;
//    }

    float t, latitude, longitude, altitude, precision_factor;
    std::string msg(gps_msg_ptr->sentence.c_str()), type;
    char NONE;
    Eigen::Vector3d bd_position;

//    ROS_INFO("I heard: [%s]", beidou_msg->sentence.c_str());

    for(int i = 0; i < msg.size(); i++){
        if(msg[i] == ',') msg[i] = ' ';
    }
    std::istringstream is(msg);
    is >> type;

    is.setf(std::ios::fixed, std::ios::floatfield);
    is.precision(8);

    if(type == "$GPGGA"){
        is >> setiosflags(std::ios::fixed) >> std::setprecision(10) >> t
           >> setiosflags(std::ios::fixed) >> std::setprecision(10) >> latitude >> NONE
           >> setiosflags(std::ios::fixed) >> std::setprecision(10) >> longitude >> NONE >> NONE >> NONE >> NONE >> precision_factor
           >> setiosflags(std::ios::fixed) >> std::setprecision(10) >> altitude;
    }
    else
        return;

    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << latitude/100.0, longitude/100.0, altitude;
//    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());
    gps_data_ptr->cov << 0.01, 0, 0,
                         0, 0.01, 0,
                         0, 0, 0.01;
    imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);

    LogGps(gps_data_ptr);
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << G_q_I.x() << "," << G_q_I.y() << "," << G_q_I.z() << "," << G_q_I.w() << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data) {
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
}

void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = state.G_p_I[0];
    pose.pose.position.y = state.G_p_I[1];
    pose.pose.position.z = state.G_p_I[2];

    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    ros_path_.poses.push_back(pose);
}
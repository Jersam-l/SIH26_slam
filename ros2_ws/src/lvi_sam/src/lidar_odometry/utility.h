#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


#ifndef BOOST_BIND_GLOBAL_PLACEHOLDERS
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#endif
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;


class ParamServer
{
public:

    rclcpp::Node::SharedPtr node_;

    std::string PROJECT_NAME;

    std::string robot_id;

    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Velodyne Sensor Configuration: Velodyne
    int N_SCAN;
    int Horizon_SCAN;
    string timeField;
    int downsampleRate;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool loopClosureEnableFlag;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    explicit ParamServer(const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        node_->declare_parameter<std::string>("PROJECT_NAME", "sam");
        PROJECT_NAME = node_->get_parameter("PROJECT_NAME").as_string();

        node_->declare_parameter<std::string>("robot_id", "roboat");
        robot_id = node_->get_parameter("robot_id").as_string();

        node_->declare_parameter<std::string>(PROJECT_NAME + ".pointCloudTopic", "points_raw");
        pointCloudTopic = node_->get_parameter(PROJECT_NAME + ".pointCloudTopic").as_string();
        node_->declare_parameter<std::string>(PROJECT_NAME + ".imuTopic", "imu_correct");
        imuTopic = node_->get_parameter(PROJECT_NAME + ".imuTopic").as_string();
        node_->declare_parameter<std::string>(PROJECT_NAME + ".odomTopic", "odometry/imu");
        odomTopic = node_->get_parameter(PROJECT_NAME + ".odomTopic").as_string();
        node_->declare_parameter<std::string>(PROJECT_NAME + ".gpsTopic", "odometry/gps");
        gpsTopic = node_->get_parameter(PROJECT_NAME + ".gpsTopic").as_string();

        node_->declare_parameter<bool>(PROJECT_NAME + ".useImuHeadingInitialization", false);
        useImuHeadingInitialization = node_->get_parameter(PROJECT_NAME + ".useImuHeadingInitialization").as_bool();
        node_->declare_parameter<bool>(PROJECT_NAME + ".useGpsElevation", false);
        useGpsElevation = node_->get_parameter(PROJECT_NAME + ".useGpsElevation").as_bool();
        node_->declare_parameter<double>(PROJECT_NAME + ".gpsCovThreshold", 2.0);
        gpsCovThreshold = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".gpsCovThreshold").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".poseCovThreshold", 25.0);
        poseCovThreshold = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".poseCovThreshold").as_double());

        node_->declare_parameter<bool>(PROJECT_NAME + ".savePCD", false);
        savePCD = node_->get_parameter(PROJECT_NAME + ".savePCD").as_bool();
        node_->declare_parameter<std::string>(PROJECT_NAME + ".savePCDDirectory", "/tmp/loam/");
        savePCDDirectory = node_->get_parameter(PROJECT_NAME + ".savePCDDirectory").as_string();

        node_->declare_parameter<int>(PROJECT_NAME + ".N_SCAN", 16);
        N_SCAN = node_->get_parameter(PROJECT_NAME + ".N_SCAN").as_int();
        node_->declare_parameter<int>(PROJECT_NAME + ".Horizon_SCAN", 1800);
        Horizon_SCAN = node_->get_parameter(PROJECT_NAME + ".Horizon_SCAN").as_int();
        node_->declare_parameter<std::string>(PROJECT_NAME + ".timeField", "time");
        timeField = node_->get_parameter(PROJECT_NAME + ".timeField").as_string();
        node_->declare_parameter<int>(PROJECT_NAME + ".downsampleRate", 1);
        downsampleRate = node_->get_parameter(PROJECT_NAME + ".downsampleRate").as_int();

        node_->declare_parameter<double>(PROJECT_NAME + ".imuAccNoise", 0.01);
        imuAccNoise = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".imuAccNoise").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".imuGyrNoise", 0.001);
        imuGyrNoise = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".imuGyrNoise").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".imuAccBiasN", 0.0002);
        imuAccBiasN = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".imuAccBiasN").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".imuGyrBiasN", 0.00003);
        imuGyrBiasN = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".imuGyrBiasN").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".imuGravity", 9.80511);
        imuGravity = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".imuGravity").as_double());
        node_->declare_parameter<std::vector<double>>(
            PROJECT_NAME + ".extrinsicRot",
            std::vector<double>{1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0});
        extRotV = node_->get_parameter(PROJECT_NAME + ".extrinsicRot").as_double_array();

        node_->declare_parameter<std::vector<double>>(
            PROJECT_NAME + ".extrinsicRPY",
            std::vector<double>{1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 1.0});
        extRPYV = node_->get_parameter(PROJECT_NAME + ".extrinsicRPY").as_double_array();

        node_->declare_parameter<std::vector<double>>(
            PROJECT_NAME + ".extrinsicTrans",
            std::vector<double>{0.0, 0.0, 0.0});
        extTransV = node_->get_parameter(PROJECT_NAME + ".extrinsicTrans").as_double_array();
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        node_->declare_parameter<double>(PROJECT_NAME + ".edgeThreshold", 0.1);
        edgeThreshold = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".edgeThreshold").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".surfThreshold", 0.1);
        surfThreshold = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".surfThreshold").as_double());
        node_->declare_parameter<int>(PROJECT_NAME + ".edgeFeatureMinValidNum", 10);
        edgeFeatureMinValidNum = node_->get_parameter(PROJECT_NAME + ".edgeFeatureMinValidNum").as_int();
        node_->declare_parameter<int>(PROJECT_NAME + ".surfFeatureMinValidNum", 100);
        surfFeatureMinValidNum = node_->get_parameter(PROJECT_NAME + ".surfFeatureMinValidNum").as_int();

        node_->declare_parameter<double>(PROJECT_NAME + ".odometrySurfLeafSize", 0.2);
        odometrySurfLeafSize = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".odometrySurfLeafSize").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".mappingCornerLeafSize", 0.2);
        mappingCornerLeafSize = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".mappingCornerLeafSize").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".mappingSurfLeafSize", 0.2);
        mappingSurfLeafSize = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".mappingSurfLeafSize").as_double());

        node_->declare_parameter<double>(PROJECT_NAME + ".z_tollerance", FLT_MAX);
        z_tollerance = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".z_tollerance").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".rotation_tollerance", FLT_MAX);
        rotation_tollerance = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".rotation_tollerance").as_double());

        node_->declare_parameter<int>(PROJECT_NAME + ".numberOfCores", 2);
        numberOfCores = node_->get_parameter(PROJECT_NAME + ".numberOfCores").as_int();
        node_->declare_parameter<double>(PROJECT_NAME + ".mappingProcessInterval", 0.15);
        mappingProcessInterval = node_->get_parameter(PROJECT_NAME + ".mappingProcessInterval").as_double();

        node_->declare_parameter<double>(PROJECT_NAME + ".surroundingkeyframeAddingDistThreshold", 1.0);
        surroundingkeyframeAddingDistThreshold = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".surroundingkeyframeAddingDistThreshold").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".surroundingkeyframeAddingAngleThreshold", 0.2);
        surroundingkeyframeAddingAngleThreshold = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".surroundingkeyframeAddingAngleThreshold").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".surroundingKeyframeDensity", 1.0);
        surroundingKeyframeDensity = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".surroundingKeyframeDensity").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".surroundingKeyframeSearchRadius", 50.0);
        surroundingKeyframeSearchRadius = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".surroundingKeyframeSearchRadius").as_double());

        node_->declare_parameter<bool>(PROJECT_NAME + ".loopClosureEnableFlag", false);
        loopClosureEnableFlag = node_->get_parameter(PROJECT_NAME + ".loopClosureEnableFlag").as_bool();
        node_->declare_parameter<int>(PROJECT_NAME + ".surroundingKeyframeSize", 50);
        surroundingKeyframeSize = node_->get_parameter(PROJECT_NAME + ".surroundingKeyframeSize").as_int();
        node_->declare_parameter<double>(PROJECT_NAME + ".historyKeyframeSearchRadius", 10.0);
        historyKeyframeSearchRadius = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".historyKeyframeSearchRadius").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".historyKeyframeSearchTimeDiff", 30.0);
        historyKeyframeSearchTimeDiff = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".historyKeyframeSearchTimeDiff").as_double());
        node_->declare_parameter<int>(PROJECT_NAME + ".historyKeyframeSearchNum", 25);
        historyKeyframeSearchNum = node_->get_parameter(PROJECT_NAME + ".historyKeyframeSearchNum").as_int();
        node_->declare_parameter<double>(PROJECT_NAME + ".historyKeyframeFitnessScore", 0.3);
        historyKeyframeFitnessScore = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".historyKeyframeFitnessScore").as_double());

        node_->declare_parameter<double>(PROJECT_NAME + ".globalMapVisualizationSearchRadius", 1e3);
        globalMapVisualizationSearchRadius = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".globalMapVisualizationSearchRadius").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".globalMapVisualizationPoseDensity", 10.0);
        globalMapVisualizationPoseDensity = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".globalMapVisualizationPoseDensity").as_double());
        node_->declare_parameter<double>(PROJECT_NAME + ".globalMapVisualizationLeafSize", 1.0);
        globalMapVisualizationLeafSize = static_cast<float>(node_->get_parameter(PROJECT_NAME + ".globalMapVisualizationLeafSize").as_double());

        usleep(100);
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            RCLCPP_ERROR(node_->get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }

        return imu_out;
    }
};

template<typename T>
sensor_msgs::msg::PointCloud2 publishCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& thisPub, T thisCloud, const rclcpp::Time& thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return rclcpp::Time(msg->header.stamp).seconds();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif
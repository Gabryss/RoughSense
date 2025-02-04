/**
 * @file ROSWrapper.hpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2024-09-20
 * 
 * @copyright Gabriel Garcia | 2024
 * @brief Implementation file for Class ROSWrapper.
 * @details This is a ROS wrapper that subscribe to a pointcloud and publish a roughness costmap.
 */
#include <memory>
#include "iostream"
#include <fstream>
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <functional>
#include <boost/make_shared.hpp>
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h" 

//ROS
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


// OPENCV
#include <opencv2/opencv.hpp>


// Custom library
#include "Roughness.hpp"
#include "Dsp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace cv;

class ROSWrapper : public rclcpp::Node
{
    public:
        ROSWrapper();
        ~ROSWrapper();

        // ===========================
        // Attributes
        // ===========================
        vector<double> filteredData;
        vector<double> rawData;

        // ===========================
        // Methods
        // ===========================
        void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu &msg);
        void publish_roughness_map(const Mat &image, float resolution, float size);
        void simulate_sinusoid_signal();
        void save_filtered_data();



    protected:
        // ===========================
        // Attributes
        // ===========================
        // ROS 2
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_roughness_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped transform_stamped;

        // Roughness
        Roughness roughness;

        // Notch bandstop filter;
        std::shared_ptr<Dsp> DSP_;

        // Initialize PCL pointcloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;        

        // Config file reader
        rapidjson::Document p; 

        
        // ===========================
        // Methods
        // ===========================
        void get_parameters(std::string parameters_path);
        void lookupTransform();
};
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
#include "Filter.hpp"
#include "Dsp.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace cv;

class ROSWrapper : public rclcpp::Node
{
    public:
        ROSWrapper();
        ~ROSWrapper();
        void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu &msg);

        void publish_roughness_map(const Mat &image, float resolution, float size);
        // std::vector<double> b = {0.95654368, -7.29004346, 24.59047794, -48.75218652, 60.51090319, -48.75218652, 24.59047794, -7.29004346, 0.95654368};
        // std::vector<double> a = {1.0, -7.512191, 25.66782538, -49.0144187, 60.48563302, -48.47511596, 23.5183952, -6.97042531, 0.80048889};
        void simulate_sinusoid_signal();
        vector<double> filteredData;
        vector<double> rawData;
        void save_filtered_data();



    protected:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        geometry_msgs::msg::TransformStamped transform_stamped;
        Roughness roughness;
        // BandStopFilter filter;
        std::shared_ptr<BandStopFilter> filter_;
        std::shared_ptr<Dsp> DSP_;

        // Initialize PCL pointcloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_roughness_;
        
        rapidjson::Document p; 

        // Number of pass
        double filtered_data;
        


        void get_parameters(std::string parameters_path);
        void lookupTransform();
};
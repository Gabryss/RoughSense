/**
 * @file ROSWrapper.cpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2024-09-20
 * 
 * @copyright Gabriel Garcia | 2024
 * @brief Implementation file for Class ROSWrapper.
 * @details This is a ROS wrapper that subscribe to a pointcloud and publish a roughness costmap.
 */
#include "ROSWrapper.hpp"


ROSWrapper::ROSWrapper(): Node("roughness_node")
{
    // Get parameters
    this->declare_parameter("param_path", "");
    std::string path_parameters = this->get_parameter("param_path").as_string();
    get_parameters(path_parameters);

    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // Get transform of the robot
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // Call every 100 ms
      std::bind(&ROSWrapper::lookupTransform, this)
    );


    // Subscribe to the point cloud topic    
    sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
     p["input_topic"].GetString(), 10, std::bind(&ROSWrapper::pc_callback, this, _1));
    

    // Create publishers
    pub_roughness_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(p["roughness_topic"].GetString(), 10);

    // Set roughness parameters
    roughness.resolution = p["map_resolution"].GetFloat();
    roughness.size = p["map_size"].GetFloat();
    roughness.stylized=p["map_unknown_transparency"].GetBool();
    roughness.ransac_iterations=p["ransac_iterations"].GetInt();
    roughness.roughness_threshold=p["roughness_threshold"].GetFloat();
    roughness.roughness_shift=p["roughness_shift"].GetFloat();
    roughness.height=p["height"].GetInt();
    roughness.low_grid_resolution=p["map_low_resolution_division_factor"].GetInt();
};

ROSWrapper::~ROSWrapper() 
{

};

void ROSWrapper::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    cloud->points.clear();
    pcl::fromROSMsg(*msg, *cloud);
    RCLCPP_INFO(this->get_logger(), "New map received");
};

void ROSWrapper::imu_callback(const sensor_msgs::msg::Imu &msg)
{

};

void ROSWrapper::lookupTransform()
{
    try {
      // Lookup transform from 'odom' to 'base_link'
      transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      roughness.pose = {transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z};

      if(cloud->size()>0)
      {
        roughness.CalculateRoughness(cloud);
        Mat image_roughness = roughness.image_roughness;

        float resolution = roughness.resolution;
        float size = roughness.size;
        publish_roughness_map(image_roughness, resolution, size);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Point cloud is empty!");
      }
    } 
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform 'odom' to 'base_link': %s", ex.what());
    }
  }


void ROSWrapper::publish_roughness_map(const Mat &image, float resolution, float size)
{
    int nb_cells = size / resolution;

    // Create a vector to hold the data from the Mat (int8_t means signed char)
    std::vector<int8_t> static_map_cell_values(image.rows * image.cols);

    // Copy the data from cv::Mat to std::vector<int8_t>
    for (int i = 0; i < image.rows * image.cols; ++i) {
        static_map_cell_values[i] = static_cast<int8_t>(image.data[i]);
    }
    size_t num_elements = image.total() * image.elemSize();


    nav_msgs::msg::OccupancyGrid occupancy_grid;
    nav_msgs::msg::MapMetaData map_meta_data;
    map_meta_data.map_load_time = this->now();
    map_meta_data.resolution = resolution;
    map_meta_data.width =  nb_cells;
    map_meta_data.height = nb_cells;
    map_meta_data.origin.position.x = (-size/2) + transform_stamped.transform.translation.x;
    map_meta_data.origin.position.y = (-size/2) + transform_stamped.transform.translation.y;
    map_meta_data.origin.position.z = transform_stamped.transform.translation.z;
    map_meta_data.origin.orientation.x= 0.7071068;
    map_meta_data.origin.orientation.y= 0.7071068;
    map_meta_data.origin.orientation.z= 0.0;
    map_meta_data.origin.orientation.w= 0.0;


    
    occupancy_grid.header.stamp = this->now();
    occupancy_grid.header.frame_id = p["roughness_frame_id"].GetString();
    occupancy_grid.info = map_meta_data;
    occupancy_grid.data.resize(image.rows * image.cols);
    occupancy_grid.data = static_map_cell_values;

    pub_roughness_->publish(occupancy_grid);
};


// Utility method to get parameters from the config file
void ROSWrapper::get_parameters(std::string parameters_path)
{
    // Open the file for reading 
    FILE* fp = fopen(parameters_path.c_str(), "r");
    
    // Use a FileReadStream to 
    // read the data from the file 
    char readBuffer[65536]; 
    rapidjson::FileReadStream is(fp, readBuffer, 
                                 sizeof(readBuffer)); 
  
    // Parse the JSON data  
    // using a Document object 
    // rapidjson::Document d; 
    p.ParseStream(is); 
  
    // Close the file 
    fclose(fp); 
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSWrapper>());
  rclcpp::shutdown();
  return 0;
}
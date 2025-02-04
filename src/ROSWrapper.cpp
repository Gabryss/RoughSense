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

    // timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(100),  // Call every 100 ms
    //   std::bind(&ROSWrapper::lookupTransform, this)
    // );

    // Point cloud

    // Subscribe to the point cloud topic    
    sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
     p["pc_topic"].GetString(), 10, std::bind(&ROSWrapper::pc_callback, this, _1));
    

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



    // IMU
    DSP_ = std::make_shared<Dsp>();
    DSP_->create_filter(27.5f, 5.0f, 200);

    // this->simulate_sinusoid_signal();
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
     p["imu_topic"].GetString(), 10, std::bind(&ROSWrapper::imu_callback, this, _1));

};

ROSWrapper::~ROSWrapper() 
{
  save_filtered_data();
};

void ROSWrapper::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    cloud->points.clear();
    pcl::fromROSMsg(*msg, *cloud);
    RCLCPP_INFO(this->get_logger(), "New map received");
};

void ROSWrapper::imu_callback(const sensor_msgs::msg::Imu &msg)
{

    // Calculate the norm of the linear acceleration vector
    double norm = roughness.calculateNorm(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    this->rawData.push_back(norm);

    this->filtered_data = norm;

    complex<float> input(norm, 0.0f);
    complex<float> output;


      
    // RCLCPP_INFO(rclcpp::get_logger("roughness_node"), "FILTERING");
    // Filter the norm to exclude the rover's resonnancy frequencies
    // this->filtered_data = DSP_->processSample(this->filtered_data);

    // Execute one filtering iteration.
    // DSP_->iirfilt_crcf_execute(DSP_->_filter, input, &output);
    DSP_->process_sample(input, &output);
    

    // Log the filtered output (here we print the real part).
    RCLCPP_INFO(this->get_logger(), "Filtered sample: %f", output.real());

    this->filteredData.push_back(output.real());
    

    // Update window
    DSP_->std_update(this->filtered_data);

    // STD
    double roughness = DSP_->get_std();

    // Publish data (development only)
    RCLCPP_INFO(this->get_logger(), "Norm: %.3f Filtered data: %.3f, Roughness: %.3f", norm, filtered_data, roughness);


};


void ROSWrapper::simulate_sinusoid_signal()
{
  RCLCPP_INFO(this->get_logger(), "Entering fake data");
  DSP_->generate_simulated_signal();
  RCLCPP_INFO(this->get_logger(), "Size of generated data: %.3ld", DSP_->sinusoid.size());

  for(int i=0; i<DSP_->sinusoid.size(); i++)
  {
    complex<float> input(DSP_->sinusoid[i], 0.0f);
    complex<float> output;

    DSP_->process_sample(input, &output);

    // double filtered_data = DSP_->processSample(DSP_->sinusoid[i]);
    this->filteredData.push_back(output.real());
    RCLCPP_INFO(this->get_logger(), "Data: %.3f Filtered data: %.3f", DSP_->sinusoid[i], output.real());
  }

  // Open an output file stream (CSV file).
  std::ofstream outFile("data.csv");
  if (!outFile.is_open()) 
  {
      std::cerr << "Error opening file for writing!" << std::endl;
  }

  // Write a header row (optional).
  outFile << "Index,Raw,Filtered\n";

  // Write data row by row.
  for (size_t i = 0; i < DSP_->sinusoid.size(); i++) 
  {
      outFile << i << "," << DSP_->sinusoid[i] << "," << this->filteredData[i] << "\n";
  }

  outFile.close();
  std::cout << "Data exported to data.csv" << std::endl;
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
      RCLCPP_WARN(this->get_logger(), "Is the TF topic properly published|filled ?");

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


// Save the filtered data
void ROSWrapper::save_filtered_data()
{
  // Open an output file stream (CSV file).
  std::ofstream outFile("data_exp.csv");
  if (!outFile.is_open()) 
  {
      RCLCPP_ERROR(this->get_logger(), "Error opening file for writing.");
      return;
  }

  // Write a header row (optional).
  outFile << "Index,Raw,Filtered\n";

  // Write data row by row.
  for (size_t i = 0; i < this->rawData.size(); i++) 
  {
      outFile << i << "," << this->rawData[i] << "," << this->filteredData[i] << "\n";
  }

  outFile.close();
  RCLCPP_INFO(this->get_logger(), "Filtered data saved to filtered_data.csv");
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSWrapper>());
  rclcpp::shutdown();
  return 0;
}
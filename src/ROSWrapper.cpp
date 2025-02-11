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
    // =====================================================
    // GET CONFIG PARAMETERS
    // =====================================================
    this->declare_parameter("param_path", "");
    std::string path_parameters = this->get_parameter("param_path").as_string();
    get_parameters(path_parameters);
    debug_info = p["debug_info"].GetBool();
    debug_time_s = p["debug_time_s"].GetInt();
    global_map_size = p["global_map_size"].GetFloat();
    resolution = p["map_resolution"].GetFloat();


    // =====================================================
    // Initialize global map
    // =====================================================
    this->create_global_map();

    // =====================================================
    // TRANSFORM
    // =====================================================
    // Get transform of the robot
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),  // Call every 200 ms
      std::bind(&ROSWrapper::lookupTransform, this)
    );

    // =====================================================
    // SPEED
    // =====================================================
    // Subscribe to the point cloud topic    
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
     p["cmd_vel_topic"].GetString(), 10, std::bind(&ROSWrapper::cmd_vel_callback, this, _1));
    


    // =====================================================
    // POINT CLOUD
    // =====================================================
    cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    // Subscribe to the point cloud topic    
    sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
     p["pc_topic"].GetString(), 10, std::bind(&ROSWrapper::pc_callback, this, _1));
    

    // Create publishers
    pub_roughness_local_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(p["roughness_topic_local"].GetString(), 10);

    pub_roughness_global_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(p["roughness_topic_global"].GetString(), 10);


    // Set roughness parameters
    roughness.resolution = resolution;
    roughness.globalGrid.resolution = p["map_resolution"].GetFloat();
    roughness.local_size = p["local_map_size"].GetFloat();
    roughness.stylized=p["map_unknown_transparency"].GetBool();
    roughness.ransac_iterations=p["ransac_iterations"].GetInt();
    roughness.roughness_threshold=p["roughness_threshold"].GetFloat();
    roughness.roughness_shift=p["roughness_shift"].GetFloat();
    roughness.height=p["height"].GetInt();
    roughness.low_grid_resolution=p["map_low_resolution_division_factor"].GetInt();


    // =====================================================
    // IMU
    // =====================================================
    imu_sampling_frequency = p["imu_sampling_frequency"].GetFloat();
    imu_filter_frequency = p["imu_filter_frequency"].GetFloat();
    imu_filter_bandwidth = p["imu_filter_bandwidth"].GetFloat();
    roughness.TestGrid();
    DSP_ = std::make_shared<Dsp>();
    DSP_->create_filter(imu_filter_frequency, imu_filter_bandwidth, imu_sampling_frequency);

    // this->simulate_sinusoid_signal();    // Debug only
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
     p["imu_topic"].GetString(), 10, std::bind(&ROSWrapper::imu_callback, this, _1));
};


ROSWrapper::~ROSWrapper() 
{
  if(debug_info)
  {
    save_filtered_data();
    save_roughness_data();
  }
};




// =====================================================
// CALLBACKS
// =====================================================

void ROSWrapper::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    cloud->points.clear();
    pcl::fromROSMsg(*msg, *cloud);
    RCLCPP_INFO(this->get_logger(), "New map received");
};


void ROSWrapper::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double norm_linear = roughness.calculateNorm(msg->linear.x, msg->linear.y, msg->linear.z);
    double norm_angular = roughness.calculateNorm(msg->angular.x, msg->angular.y, msg->angular.z);
    velocity_norm = norm_linear + norm_angular;

    // RCLCPP_INFO(this->get_logger(), "Test: %.3f", norm);
};


void ROSWrapper::imu_callback(const sensor_msgs::msg::Imu &msg)
{
    
    // Calculate the norm of the linear acceleration vector
    double norm = roughness.calculateNorm(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    this->rawData.push_back(norm);

    // Initialize filtered variable
    complex<float> input(norm, 0.0f);
    complex<float> filtered_data;
      
    // Execute one filtering iteration.
    DSP_->process_sample(input, &filtered_data);

    this->filteredData.push_back(filtered_data.real());

    // Update window
    this->update_window_imu(filtered_data.real());

    // Publish data (debug only)
    if (debug_info)
    {
      RCLCPP_INFO(this->get_logger(), "Norm: %.3f Filtered data: %.3f", norm, filtered_data.real());
    }
};



// =====================================================
// TRANSFORM
// =====================================================

void ROSWrapper::lookupTransform()
{
    try {
      // Lookup transform from 'odom' to 'base_link'
      transform_stamped = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      roughness.pose = {transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z};

      coordinates_grid current_cell = compute_offset();


      // Calculate the IMU roughness
      if (current_cell != previous_cell)
      {
        vector<double> window_imu_vector = convert_deque_vector(window_imu);
        double std = roughness.CalculateStd(window_imu_vector);
        double roughness_imu = roughness.roughness_normalization(std, roughness.roughness_threshold);
        vector<long unsigned int> robot_imu_coordinates = {previous_cell[0] + (roughness.TGridLocal.size()/2), previous_cell[1] + (roughness.TGridLocal.size()/2)};
        
        // Check if the computed global cell is within the bounds of the global grid.&
        if (robot_imu_coordinates[0] < 0 || robot_imu_coordinates[0] >= static_cast<int>(global_grid.size()) ||
            robot_imu_coordinates[1] < 0 || robot_imu_coordinates[1] >= static_cast<int>(global_grid.size()))
        {
          // Skip cells that fall outside the global grid.
          RCLCPP_ERROR(this->get_logger(), "Error: Local grid outside the global grid.");
        }
        else
        {
          global_grid[robot_imu_coordinates[0]][robot_imu_coordinates[1]][1] = roughness_imu;
          roughness_vector_imu.push_back(std);
          // roughness_vector_lidar.push_back(global_grid[robot_imu_coordinates[0]][robot_imu_coordinates[1]][0]);
          roughness_vector_lidar.push_back(global_grid[robot_imu_coordinates[0]][robot_imu_coordinates[1]][0]);

          velocity_vector_imu.push_back(velocity_norm);
          RCLCPP_ERROR(this->get_logger(), "LiDAR: %.3f, IMU: %.3f", global_grid[robot_imu_coordinates[0]][robot_imu_coordinates[1]][0], std);
        }
      }

      // Calculate the point cloud roughness
      if(cloud->size()>0)
      {
        roughness.CalculatePCRoughness(cloud);
        coordinates_local = {roughness.pose[0], roughness.pose[1]};
        update_global_map(current_cell);
        publish_roughness_map(roughness.TGridLocal, true);
        publish_roughness_map(global_grid, false);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Point cloud is empty!");
      }
    } 
    catch (const tf2::TransformException & ex) {
      if ((temp_timer>=debug_time_s*5) && debug_info)
      {
        temp_timer = 0;
        RCLCPP_WARN(this->get_logger(), "Could not transform 'odom' to 'base_link': %s", ex.what());
        RCLCPP_WARN(this->get_logger(), "Is the TF topic properly published|filled ?");
      }
      else
      {
        temp_timer +=1;
      }
    }
  };



// =====================================================
// PUBLISHER
// =====================================================

void ROSWrapper::publish_roughness_map(const TerrainGrid &grid, bool is_local)
{
    nav_msgs::msg::OccupancyGrid occupancy_grid;
    nav_msgs::msg::MapMetaData map_meta_data;
    map_meta_data.map_load_time = this->now();
    map_meta_data.resolution = resolution;
    map_meta_data.width =  grid.size();
    map_meta_data.height = grid[0].size();

    if (is_local)
    {
      map_meta_data.origin.position.x = (-(grid.size()*resolution)/2) + coordinates_local[0];
      map_meta_data.origin.position.y = (-(grid[0].size()*resolution)/2) + coordinates_local[1];
    }
    else
    {
      map_meta_data.origin.position.x = -(grid.size()*resolution)/2;
      map_meta_data.origin.position.y = -(grid[0].size()*resolution)/2;
    }
    map_meta_data.origin.position.z = transform_stamped.transform.translation.z;

    map_meta_data.origin.orientation.x= 0.7071068;
    map_meta_data.origin.orientation.y= 0.7071068;

    map_meta_data.origin.orientation.z= 0.0;
    map_meta_data.origin.orientation.w= 0.0;

    
    occupancy_grid.header.stamp = this->now();
    occupancy_grid.header.frame_id = p["roughness_frame_id"].GetString();
    occupancy_grid.info = map_meta_data;
    occupancy_grid.data.resize(map_meta_data.width * map_meta_data.height);

    // Fill the occupancy_grid with data
    // Fill the OccupancyGrid message.
    for (unsigned int y = 0; y < map_meta_data.height; ++y) 
    {
      for (unsigned int x = 0; x < map_meta_data.width; ++x) 
      {
        unsigned int index = y * map_meta_data.width + x;
        occupancy_grid.data[index] = grid[y][x][0];                    // Fill data
      }
    }

    if(is_local)
    {
      pub_roughness_local_->publish(occupancy_grid);
    }
    else
    {
      pub_roughness_global_->publish(occupancy_grid);
    }
};



// =====================================================
// Update prediction weights
// =====================================================


double ROSWrapper::updateWeight(double w_k, double lidar_k, double imu_k, double alpha_0, 
                    double lambda_decay, double beta, double& error_moving_avg, int time) 
{
    const double epsilon = 1e-5;  // Small value to prevent division by zero

    // Compute current error
    double error_k = fabs(imu_k - w_k * lidar_k);

    // Update moving average of error
    error_moving_avg = beta * error_moving_avg + (1 - beta) * error_k;

    // Compute adaptive learning rate
    double alpha_k = alpha_0 / (1 + lambda_decay * k);      // Time-based decay
    alpha_k *= error_k / (max(error_moving_avg, epsilon));  // Error-based scaling

    // Update weight
    w_k = w_k + alpha_k * (imu_k - w_k * lidar_k);

    return w_k;
};


// =====================================================
// TOOLS
// =====================================================

// Generate sinusoid signal for IMU debug 
void ROSWrapper::simulate_sinusoid_signal()
{
  RCLCPP_INFO(this->get_logger(), "Generating simulated data");
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


// Update the sliding window with new IMU data 
void ROSWrapper::update_window_imu(double x)
{
    if (window_imu.size() == window_size) {
            window_imu.pop_front();  // Remove the oldest value
        }
        window_imu.push_back(x);
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


// Save IMU and LiDAR roughness for analysis
void ROSWrapper::save_roughness_data()
{
  // Open an output file stream (CSV file).
  std::ofstream outFile("data_roughness.csv");
  if (!outFile.is_open()) 
  {
      RCLCPP_ERROR(this->get_logger(), "Error opening file for writing.");
      return;
  }

  // Write a header row (optional).
  outFile << "Index,LiDAR,IMU,SPEED\n";

  // Write data row by row.
  for (size_t i = 0; i < this->roughness_vector_lidar.size(); i++) 
  {
      outFile << i << "," << this->roughness_vector_lidar[i] << "," << this->roughness_vector_imu[i] << "," << this->velocity_vector_imu[i] << "\n";
  }

  outFile.close();
  RCLCPP_INFO(this->get_logger(), "Filtered data saved to filtered_data.csv");
};


// Convert a deque of double into a vector of double
vector<double> ROSWrapper::convert_deque_vector(deque<double> input)
{
  // Create a vector of floats with the same size as imuData
  vector<double> output(input.size());

  // Convert each double to double using transform
  transform(input.begin(), input.end(), output.begin(),
                  [](double d) { return static_cast<double>(d); });
  
  return output;
};


// Create global map
void ROSWrapper::create_global_map()
{
  unsigned int nb_cells = static_cast<unsigned int>(global_map_size/resolution);

  global_grid.clear();
  global_grid.resize(nb_cells);
  for(int ind_x=0; ind_x<nb_cells; ind_x++)
  {
      global_grid[ind_x].resize(nb_cells);
      for(int ind_y=0; ind_y<nb_cells; ind_y++)
      {
          global_grid[ind_x][ind_y].resize(2);
          global_grid[ind_x][ind_y][0] = 0.0;
          global_grid[ind_x][ind_y][1] = 0.0;
      }
  }
};


// Update global map
void ROSWrapper::update_global_map(coordinates_grid offset)
{
  previous_cell = offset; // Get the current cell (for IMu analysis)

  // Loop on each cell of the local grid
  for(int ind_x=0; ind_x<roughness.TGridLocal.size(); ind_x++)
  {
    for(int ind_y=0; ind_y<roughness.TGridLocal[0].size(); ind_y++)
    {

      // Compute the corresponding global grid cell.
      int global_x = ind_x + offset[0];
      int global_y = ind_y + offset[1];

      // Check if the computed global cell is within the bounds of the global grid.&
      if (global_x < 0 || global_x >= static_cast<int>(global_grid.size()) ||
          global_y < 0 || global_y >= static_cast<int>(global_grid.size()))
      {
        // Skip cells that fall outside the global grid.
        RCLCPP_ERROR(this->get_logger(), "Error: Local grid outside the global grid.");
        continue;
      }

      // Fill the global grid
      global_grid[global_x][global_y][0] = roughness.TGridLocal[ind_x][ind_y][0];
    }
  }
};


// Compute offset
coordinates_grid ROSWrapper::compute_offset()
{
    // Compute offset
    double local_origin_x =  (roughness.pose[0]) - (roughness.local_size / 2); // In meter
    double local_origin_y =  (roughness.pose[1]) - (roughness.local_size / 2); // In meter

    double global_origin = -(global_map_size) / 2; // In meter

    // Compute offset
    int cell_indx = static_cast<int>((local_origin_x - global_origin)/resolution);
    int cell_indy = static_cast<int>((local_origin_y - global_origin)/resolution);
    return {cell_indx, cell_indy};
};


// =====================================================
// MAIN
// =====================================================


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSWrapper>());
  rclcpp::shutdown();
  return 0;
}
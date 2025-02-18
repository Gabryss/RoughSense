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
    save_data = p["save_data"].GetBool();
    debug_info = p["debug_info"].GetBool();
    imu_correction = p["activate_imu_correction"].GetBool();
    debug_time_s = p["debug_time_s"].GetInt();
    global_map_size = p["global_map_size"].GetFloat();
    resolution = p["map_resolution"].GetFloat();
    roughness_lidar_threshold=p["roughness_lidar_threshold"].GetFloat();
    roughness_imu_threshold=p["roughness_imu_threshold"].GetFloat();
    window_size = p["imu_window_size"].GetInt();


    w_k = p["w_k"].GetFloat();
    alpha_0 = p["alpha_0"].GetFloat();
    lambda_decay = p["lambda_decay"].GetFloat();
    beta = p["beta"].GetFloat();
    error_moving_avg = p["error_moving_avg"].GetFloat();




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
    roughness.roughness_shift=p["roughness_shift"].GetFloat();
    roughness.roughness_lidar_threshold=p["roughness_lidar_threshold"].GetFloat();
    roughness.roughness_imu_threshold=p["roughness_imu_threshold"].GetFloat();
    roughness.height=p["height"].GetFloat();
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
  if(save_data)
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
      coordinates_local = {roughness.pose[0], roughness.pose[1]};

      coordinates_grid current_cell = compute_offset();

      // Calculate the point cloud roughness
      if(cloud->size()>3 && (current_cell != previous_cell))
      {
          roughness.CalculatePCRoughness(cloud);
          roughness.saveEntireGridToPCD(roughness.PCGrid);

          // Calculate the IMU roughness
          vector<double> window_imu_vector = convert_deque_vector(window_imu);
          vector<double> window_imu_vector_speed_normalized = speed_normalization(window_imu_vector);
          double roughness_imu = roughness.CalculateStd(window_imu_vector);
          double roughness_imu_normalized = roughness.roughness_normalization(roughness_imu, roughness.roughness_imu_threshold);

          long unsigned int local_grid_center = roughness.TGridLocal.size()/2;

          robot_imu_coordinates = {previous_cell[0] + local_grid_center, previous_cell[1] + local_grid_center};
          
          // Check if the computed global cell is within the bounds of the global grid.&
          if (robot_imu_coordinates[0] < 0 || robot_imu_coordinates[0] >= static_cast<int>(global_grid.size()) ||
              robot_imu_coordinates[1] < 0 || robot_imu_coordinates[1] >= static_cast<int>(global_grid.size()))
          {
            // Skip cells that fall outside the global grid.
            RCLCPP_ERROR(this->get_logger(), "Error: Local grid outside the global grid.");
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "IMU is going to be attributed %.3li, %.3li", local_grid_center, local_grid_center);
            roughness.TGridLocal[local_grid_center][local_grid_center][1] = roughness_imu;
            global_grid[robot_imu_coordinates[0]][robot_imu_coordinates[1]][1] = roughness_imu_normalized;
            last_imu_roughness = roughness_imu_normalized;
          }
        
          update_global_map(current_cell);
          
          // Publish maps
          publish_roughness_map(roughness.TGridLocal, true);  // Local map
          publish_roughness_map(global_grid, false);          // Global map
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

    int center_coordinates_x = -(grid.size()*resolution)/2;
    int center_coordinates_y = -(grid[0].size()*resolution)/2;


    if (is_local)
    {
      map_meta_data.origin.position.x = center_coordinates_x + coordinates_local[0];
      map_meta_data.origin.position.y = center_coordinates_y + coordinates_local[1];
    }
    else
    {
      map_meta_data.origin.position.x = center_coordinates_x;
      map_meta_data.origin.position.y = center_coordinates_y;
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
        occupancy_grid.data[index] = grid[y][x][0]*100;                    // Fill data
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
  std::ofstream outFile("data_imu_filtering.csv");
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
  RCLCPP_INFO(this->get_logger(), "Filtered data saved to data_imu_filtering.csv");
};


// Save IMU and LiDAR roughness for analysis
void ROSWrapper::save_roughness_data()
{
  // Open an output file stream (CSV file).
  std::ofstream outFile("data/sensors/data_analysis.csv");
  if (!outFile.is_open()) 
  {
      RCLCPP_ERROR(this->get_logger(), "Error opening file for writing.");
      return;
  }

  // Write a header row (optional).
  outFile << "Index,L_raw,L_raw_norm,L_corrected,IMU_raw,IMU_normalized,SPEED,X_local,Y_local,X_global,Y_global,Error\n";

  // Write data row by row.
  for (size_t i = 0; i < this->vector_roughness_lidar_raw.size(); i++) 
  {
      outFile << i << "," << this->vector_roughness_lidar_raw[i] << "," << 
      this->vector_roughness_lidar_raw_normalized[i] << "," << 
      this->vector_roughness_lidar_raw_normalized_corrected[i] << "," << 
      this->vector_roughness_imu[i] << "," << 
      this->vector_roughness_imu_normalized[i] << "," << 
      this->vector_velocity_imu[i] << "," << 
      this->vector_coordinates_local_x[i] << "," <<
      this->vector_coordinates_local_y[i] << "," <<
      this->vector_coordinates_global_x[i] << "," <<
      this->vector_coordinates_global_y[i] << "," <<
      this->vector_error[i]
      << "\n";
  }

  outFile.close();
  RCLCPP_INFO(this->get_logger(), "Experiment data saved to data_analysis.csv");
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
  previous_cell = offset; // Get the current cell (for IMU analysis)

  // Loop on each cell of the local grid
  for(int local_ind_x=0; local_ind_x<roughness.TGridLocal.size(); local_ind_x++)
  {
    for(int local_ind_y=0; local_ind_y<roughness.TGridLocal[0].size(); local_ind_y++)
    {

      // Compute the corresponding global grid cell.
      int global_x = local_ind_x + offset[0];
      int global_y = local_ind_y + offset[1];

      // Check if the computed global cell is within the bounds of the global grid.&
      if (global_x < 0 || global_x >= static_cast<int>(global_grid.size()) ||
          global_y < 0 || global_y >= static_cast<int>(global_grid.size()))
      {
        // Skip cells that fall outside the global grid.
        RCLCPP_ERROR(this->get_logger(), "Error: Local grid outside the global grid.");
        continue;
      }

      if (roughness.TGridLocal[local_ind_x][local_ind_y][0]>0) // Only takes the computed values
      {
        double normalized_lidar_roughness = roughness.roughness_normalization(roughness.TGridLocal[local_ind_x][local_ind_y][0], roughness.roughness_lidar_threshold);
        double error = last_imu_roughness - normalized_lidar_roughness;


        // Fill the global grid
        if (imu_correction)
        {
          global_grid[global_x][global_y][0] = normalized_lidar_roughness + error;
          // RCLCPP_ERROR(this->get_logger(), "Error: here is the error %.3f, here is the roughness %.3f, here is the imu rough %.3f", error, normalized_lidar_roughness, last_imu_roughness);
        }
        else
        {
          global_grid[global_x][global_y][0] = normalized_lidar_roughness;  
        }
        
        // Edge correction
        if (global_grid[global_x][global_y][0] > LETHAL)
        {
          global_grid[global_x][global_y][0] = LETHAL;
        }
        else if (global_grid[global_x][global_y][0] < 0)
        {
          global_grid[global_x][global_y][0] = 0;
        }


        // Save data for analysis
        if (global_x == robot_imu_coordinates[0] && global_y == robot_imu_coordinates[1])
        {
          // Lidar
          vector_roughness_lidar_raw.push_back(roughness.TGridLocal[local_ind_x][local_ind_y][0]);
          vector_roughness_lidar_raw_normalized.push_back(normalized_lidar_roughness);
          vector_roughness_lidar_raw_normalized_corrected.push_back(global_grid[global_x][global_y][0]);


          //IMU
          vector_roughness_imu.push_back(roughness.TGridLocal[local_ind_x][local_ind_y][1]);
          vector_roughness_imu_normalized.push_back(global_grid[global_x][global_y][1]);

          //Other
          vector_error.push_back(error);

          vector_coordinates_local_x.push_back(local_ind_x);
          vector_coordinates_local_y.push_back(local_ind_y);

          vector_coordinates_global_x.push_back(global_x);
          vector_coordinates_global_y.push_back(global_y);
          vector_velocity_imu.push_back(velocity_norm);
          
          //Save current cell
          // roughness.SaveCellASPC(local_ind_x, local_ind_y, vector_roughness_lidar_raw.size());
        }
      }
    }
  }
};


// Divide all the Norms by the speed to remove the speed variation
vector<double> ROSWrapper::speed_normalization(vector<double>& norms)
{
  if (velocity_norm != 0.0)
  {
    if(velocity_norm < 1.0){velocity_norm*=10;};
    for (int i=0; i<norms.size(); i++)
    {
      norms[i] /=  velocity_norm;
    }
  }
  return norms;
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
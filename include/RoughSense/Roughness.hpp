/**
 * @file Roughness.hpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2024-09-20
 * 
 * @copyright Gabriel Garcia | 2024
 * @brief Implementation file for Class Roughness.
 * @details This is a C++ Roughness library taking a PCL point cloud as an input and returning an image. CalculateRoughness is the core method of this library.
 */
#include <vector>
#include <iostream>
#include <cmath>

// PCL libraries
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// OpenCV library
#include <opencv2/opencv.hpp>

// Custom libraries
#include "Ransac.hpp"
#include "MapCreator.hpp"



using namespace cv;

using namespace std;


class Roughness
{
    public:
        Roughness(/* args */) {};
        ~Roughness() {};

        // ===========================
        // Attributes
        // ===========================
        // Traversability grids
        vector<vector<vector<double>>> TGridLocal; //Grid of std_deviation, slope and gradient for each cell - Local
        vector<vector<vector<double>>> TGridGlobal; //Grid of std_deviation, slope and gradient for each cell - Global
        
        // Point cloud grids
        vector<vector<vector<pcl::PointXYZI>>> PCGrid; //Grid with cells filled with points from the PC
        vector<vector<vector<pcl::PointXYZI>>> PCLowGrid; //Grid with cells filled with points from the PC
        

        vector<float> imu_data;
        vector<double> pose = {0.0,0.0,0.0};
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        float resolution;
        int low_grid_resolution=4;
        float size;
        float roughness_threshold=1;
        float roughness_shift=0;
        float height=1;
        float min_height=0;
        float min_roughness=0;
        bool stylized=false;
        int ransac_iterations;
        unsigned int nb_cells;

        // Traversability image conversion
        MapCreator map_creator;
        Mat image_roughness;

        // ===========================
        // Methods
        // ===========================
        double CalculateStd(vector<double>& distances);
        double calculateNorm(double linear_accel_x, double linear_accel_y, double linear_accel_z);

        //Main method
        void CalculatePCRoughness(pcl::PointCloud<pcl::PointXYZI>::Ptr Data_in);
        
        void DisplayGrid(); //For debug purposes


        
    protected:
        // ===========================
        // Attributes
        // ===========================
        pcl::PointCloud<pcl::PointXYZI>::Ptr Data_in;
        float LETHAL = 100.0;

        // ===========================
        // Methods
        // ===========================
        void ImportPCD(string path);
        void CreateTGridLocal(int nb_cells);
        void CreatePCGrid(int nb_cells);
        void FillPCGrid();
        void FillTGridLocal();
        double roughness_normalization(double value, float threshold);

};

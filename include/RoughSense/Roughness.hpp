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
#include <deque>


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

        // Public attributes
        vector<vector<vector<float>>> TGrid; //Grid of std_deviation, slope and gradient for each cell
        vector<vector<vector<float>>> TMeanGrid; //Grid of std_deviation, slope and gradient for each cell
        
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
        MapCreator map_creator;
        Mat image_roughness;


        float CalculateStd(vector<float>& distances);

        //Main method
        void CalculateRoughness(pcl::PointCloud<pcl::PointXYZI>::Ptr Data_in);
        
        void DisplayGrid(); //For debug purposes
        
    protected:
        // Protected attributes
        pcl::PointCloud<pcl::PointXYZI>::Ptr Data_in;
        float LETHAL = 100;

        vector<double> feedforward_coefficients = {0.71935955,1.08692705,3.49330396,3.41587308,5.56253496,3.41587308,3.49330396,1.08692705,0.71935955};  // IIR filter feedforward coefficients
        vector<double> feedback_coefficients = {1.0,1.38707862,4.06717958,3.66387359,5.47405587,3.10864829,2.92914834,0.84599977,0.5174782};   // IIR filter feedback coefficients
        deque<double> inputHistory;             // IIR filter input history
        deque<double> outputHistory;            // IIR filter output history


        // Utility methods
        void ImportPCD(string path);
        void CreateTGrid(int nb_cells);
        void CreatePCGrid(int nb_cells);
        void FillPCGrid();
        void FillTGrid();
        float roughness_normalization(float value, float threshold);
        double IIRfilter(double input);
        double calculateNorm(double linear_accel_x, double linear_accel_y, double linear_accel_z)

};

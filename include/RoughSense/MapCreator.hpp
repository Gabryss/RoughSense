/**
 * @file MapCreator.hpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2024-09-20
 * 
 * @copyright Cong Ma | 2016
 * @brief Implementation file for Class MapCreator.
 * @details This is a C++ library to create images based on a grid of vectors. Output an image
 */

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class MapCreator
{
    public:
        // ===========================
        // Attributes
        // ===========================
        int rows;                               // Number of rows in the grid
        int cols;                               // Number of columns in the grid
        int depth;                              // Depth to look at in the maatrix
        float max_value;                        // Max value of the matrix (for normalization)
        vector<vector<vector<float>>> Grid;     // Input grid
        Mat image;


        // ===========================
        // Methods
        // ===========================
        Mat MakeMap(vector<vector<vector<float>>>& Grid, int input_depth);                         // Create and save the map

    protected:
        // ===========================
        // Methods
        // ===========================
        void saveMap(string path, Mat image);   // Save the map on the hard drive
};
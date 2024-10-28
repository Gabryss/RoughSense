/**
 * @file MapCreator.cpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2024-09-20
 * 
 * @copyright Cong Ma | 2016
 * @brief Implementation file for Class MapCreator.
 * @details This is a C++ library to create images based on a grid of vectors. Output an image
 */

#include "MapCreator.hpp"

cv::Vec3b MapCreator::hue2rgb(float H) {
	uint8_t r, g, b;
    cv::Vec3b color;
	
    if (H == 0) {
        r = 0;
        g = 0;
        b = 0;
    } else {
        float h =  (1 - (H*0.85 + 0.15));// / 360;
        float v = 1;
        
        int i = floor(h * 6);
        float f = 255*(h * 6 - i);
        float q = (255 - f);
        
        switch (i % 6) {
            case 0: r = 255, g = f, b = 0; break;
            case 1: r = q, g = 255, b = 0; break;
            case 2: r = 0, g = 255, b = f; break;
            case 3: r = 0, g = q, b = 255; break;
            case 4: r = f, g = 0, b = 255; break;
            case 5: r = 255, g = 0, b = q; break;
        }
    }

	color[0] = b;
    color[1] = g;
    color[2] = r;
	return color;
};

Mat MapCreator::MakeMap(vector<vector<vector<float>>>& input_grid, int input_depth)
{
    Grid = input_grid;
    depth = input_depth;

    rows = Grid.size();              // Number of rows in the grid (5)
    cols = Grid[0].size();           // Number of columns in the grid (5)

    // Create an OpenCV matrix (image) to hold the 2D slice data
    std::vector< int > shape = {rows, cols};
    // image.reshape(CV_8UC1, shape);
    Mat new_image(rows, cols, CV_8UC1);
    image = new_image;
    // getMaxValue();    
    
    // Fill the image based on the values of the grid
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float value = Grid[i][j][depth];
            image.at<int8_t>(i, j) = value;
        }
    }

    return image;
};

Mat MapCreator::MakeMapRGB(vector<vector<vector<float>>>& input_grid, int input_depth)
{
    Grid = input_grid;
    depth = input_depth;

    rows = Grid.size();              // Number of rows in the grid (5)
    cols = Grid[0].size();           // Number of columns in the grid (5)

    // Create an OpenCV matrix (image) to hold the 2D slice data
    std::vector< int > shape = {rows, cols};
    Mat new_image(rows, cols, CV_8UC3);
    image = new_image;
    // getMaxValue();
    
    
    // Fill the image based on the values of the grid
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            float value = Grid[i][j][depth];
            image.at<cv::Vec3b>(i, j) = hue2rgb(value);
        }
    }

    return image;
};


void MapCreator::getMaxValue()
{
    max_value = 0.0;
    for(int i=0; i<rows; i++)
    {
        for(int j=0; j<cols; j++)
        {
            if(Grid[i][j][depth]>max_value)
            {
                max_value = Grid[i][j][depth];
            }
        }
    }
};


void MapCreator::saveMap(string path, Mat image)
{
    // Save the image
    imwrite(path, image);
};
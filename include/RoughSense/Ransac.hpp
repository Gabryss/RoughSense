/**
 * @file Ransac.hpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2024-09-16
 * 
 * @copyright Cong Ma | 2016
 * @brief Implementation file for Class RansacAlgorithm.
 * @details This is a C++ wrapper with slight modification of a Ransac algorithm implementation.
 */
// #ifndef RANSAC_H
// #define RANSAC_H

#include <vector>
#include <iostream>
#include <random>  // uniform_int_distribution
#include <cmath>
#include <pcl/point_types.h>


using namespace std;

class RansacAlgorithm
{
    public:
        RansacAlgorithm() {};           
        ~RansacAlgorithm() {};
        int t;                           // Threshold value to evaluate algorithm performance (inliners)
        int k=100;                       // Number of iterations allowed in the algorithm

        vector<pcl::PointXYZI> data;     // Input data
        vector<float> distances;         // Distances from point to plane
        vector<float> temp_distances;    // Temp distance vector
        
        vector<float> bestFit;           // Model parameters
        void FitPlane(int t, vector<pcl::PointXYZI>& data, vector<float>& bestFit);
        void ResetState();

    protected:
        vector<float> PlaneEquation(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2, const pcl::PointXYZI& p3);
        float CalculateDistanceFromPlane(const pcl::PointXYZI& points, const vector<float>& plane_eq);
        int CountInliers(vector<pcl::PointXYZI>& data, vector<float>& plane_eq, int t);
        
};
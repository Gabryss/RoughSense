/**
 * @file Ransac.cpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2024-09-16
 * 
 * @copyright Cong Ma | 2016
 * @brief Implementation file for Class RansacAlgorithm.
 * @details This is a C++ wrapper with slight modification of a Ransac algorithm implementation.
 */
#include "Ransac.hpp"



// =====================================================
// Process Ransac on a pointcloud cell
// =====================================================
void RansacAlgorithm::FitPlane(int t, vector<pcl::PointXYZI>& data, vector<float>& bestFit)
{
    int best_inliers = 0;
    random_device dev;
    mt19937 rng(dev());
    uniform_int_distribution<mt19937::result_type> dist_data(0,data.size()-1); // distribution in range [0, data.size-1]

    if(data.size()>0)
    {
        for(int i=0; i<k; i++)
        {
            // Randomly sample points

            int indx_1 = dist_data(rng);
            int indx_2 = dist_data(rng);
            int indx_3 = dist_data(rng);

            while (indx_2 == indx_1) indx_2 = dist_data(rng);
            while (indx_3 == indx_1 || indx_3 == indx_2) indx_3 = dist_data(rng);

            // Compute plane equation based on the sample taken
            vector<float> plane = RansacAlgorithm::PlaneEquation(data[indx_1],data[indx_2],data[indx_3]);


            // Evaluate inliners
            int inliner_count = RansacAlgorithm::CountInliers(data, plane, t);

            // Select the plane with the highest amount of inliers
            if(inliner_count > best_inliers)
            {
                best_inliers = inliner_count;
                RansacAlgorithm::distances = RansacAlgorithm::temp_distances; // Retreive the distance for std deviation in Traversability.cpp
                bestFit = plane;
            }
        }
    }
    
};


vector<float> RansacAlgorithm::PlaneEquation(const pcl::PointXYZI& p1, const pcl::PointXYZI& p2, const pcl::PointXYZI& p3)
{
    // Calculate two vectors that lies on the plane we wish to calculate
    vector<float> v1({p2.x-p1.x, p2.y-p1.y, p2.z-p1.z});
    vector<float> v2({p3.x-p1.x, p3.y-p1.y, p3.z-p1.z}); 

    // Calculate the cross product between those two vectors. -> Gives us a normal vector to the plane (aka A,B and C coefficients of the plane equation).
    //           |  A   B   C  |
    // V1 x V2 = | V1x V1y V1z |
    //           | V2x V2y V2z |

    // A = V1y * V2z - V2y * V1z
    // B = V2x * V1z - V1x * V2z
    // C = V1x * V2y - V2x * V1y

    float A = v1[1]*v2[2] - v2[1]*v1[2];
    float B = v2[0]*v1[2] - v1[0]*v2[2];
    float C = v1[0]*v2[1] - v2[0]*v1[1];

    // Calculate the coefficient D using the plane equation Ax + By + Cz = D and one of the input points

    float D = A*p1.x + B*p1.y + C*p1.z;

    return {A,B,C,D};
};






// =====================================================
// TOOLS
// =====================================================

float RansacAlgorithm::CalculateDistanceFromPlane(const pcl::PointXYZI& point, const vector<float>& plane_eq)
{
    float up = abs(plane_eq[0] * point.x + plane_eq[1] * point.y + plane_eq[2] * point.z + plane_eq[3]);
    float down = sqrt(plane_eq[0]*plane_eq[0] + plane_eq[1]*plane_eq[1] + plane_eq[2]*plane_eq[2]);
    return (up/down);
};


int RansacAlgorithm::CountInliers(vector<pcl::PointXYZI>& data, vector<float>& plane_eq, int t)
{
    int inliners_number = 0;
    vector<float> temp_distances;                       // Temp distance vector
    RansacAlgorithm::temp_distances = temp_distances;

    for(long unsigned int i=0; i<data.size(); i++)
    {
        float distance = RansacAlgorithm::CalculateDistanceFromPlane(data[i], plane_eq);
        RansacAlgorithm::temp_distances.push_back(distance);

        if(distance < t)
        {
            inliners_number++;
        }
    }
    return inliners_number;
};

void RansacAlgorithm::ResetState()
{
      *this = {};
};
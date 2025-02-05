/**
 * @file Roughness.cpp
 * @author gabriel.garcia@uni.lu
 * @version 0.1
 * @date 2024-09-20
 * 
 * @copyright Gabriel Garcia | 2024
 * @brief Implementation file for Class Roughness.
 * @details This is a C++ Roughness library taking a PCL point cloud as an input and returning an image. CalculateRoughness is the core method of this library.
 */
#include "Roughness.hpp"




// =====================================================
// Main point cloud function
// =====================================================
void Roughness::CalculatePCRoughness(pcl::PointCloud<pcl::PointXYZI>::Ptr Data_in)
// Core method that call the other methods to calculate the point cloud roughness
{
    nb_cells = static_cast<unsigned int>(local_size/resolution);

    cloud = Data_in;
    
    CreatePCGrid(nb_cells);
    CreateTGridLocal(nb_cells);
    FillPCGrid();
    FillTGridLocal();
    image_local_roughness = map_creator.MakeMap(TGridLocal, 0);
};




// =====================================================
// Process grids
// =====================================================

void Roughness::CreateTGridLocal(int nb_cells)
// Create the Roughness grid size nb_cells
{
    TGridLocal.clear();
    TGridLocal.resize(nb_cells);
    for(int ind_x=0; ind_x<nb_cells; ind_x++)
    {
        TGridLocal[ind_x].resize(nb_cells);
        for(int ind_y=0; ind_y<nb_cells; ind_y++)
        {
            TGridLocal[ind_x][ind_y].resize(3);
        }
    }
};


void Roughness::CreatePCGrid(int nb_cells)
{
    PCGrid.clear();
    // Initialize the point cloud grid to the right size
    PCGrid.resize(nb_cells);
    for(int indx=0; indx<nb_cells; indx++)
    {
        PCGrid[indx].resize(nb_cells);
    }

    PCLowGrid.clear();
    // Initialize the low resolution grid
    int nb_low_cells = nb_cells/low_grid_resolution;
    if(nb_low_cells == 0)
    {
        nb_low_cells = 1;
    }
    PCLowGrid.resize(nb_low_cells);
    for(int indx=0; indx<nb_low_cells; indx++)
    {
        PCLowGrid[indx].resize(nb_low_cells);
    }
};


void Roughness::FillPCGrid()
{
    //Fill each cell with the coresponding point from the input point cloud
    //All the points outside the map are discarded
    for(unsigned int i = 0; i < cloud->points.size(); i++)
    {
        if(((pose[0]-(local_size/2)) < cloud->points[i].x) && (cloud->points[i].x < (pose[0]+(local_size/2))) && ((pose[1]-(local_size/2)) < cloud->points[i].y) && (cloud->points[i].y < (pose[1]+(local_size/2))) && ( cloud->points[i].z < (height+pose[2])))
        {
            unsigned int indx = static_cast<unsigned int>((cloud->points[i].x - (pose[0]-(local_size/2))) / (local_size) * (local_size / resolution));
            unsigned int indy = static_cast<unsigned int>((cloud->points[i].y - (pose[1]-(local_size/2))) / (local_size) * (local_size / resolution));


            // Edge case for odd numbers
            if(indx == nb_cells)
            {
                indx = nb_cells - 1;
            }
            if(indy == nb_cells)
            {
                indy = nb_cells - 1;
            }

            PCGrid[indx][indy].push_back(cloud->points[i]);

            // Low resolution grid
            int indx_low = indx / low_grid_resolution;
            int indy_low = indy / low_grid_resolution;
            int nb_low_cells = nb_cells/low_grid_resolution;
            
            // Edge case for odd numbers
            if(indx_low == nb_low_cells)
            {
                indx_low = nb_low_cells - 1;
            }
            if(indy_low == nb_low_cells)
            {
                indy_low = nb_low_cells - 1;
            }
            PCLowGrid[indx_low][indy_low].push_back(cloud->points[i]);
        }
    }
};

void Roughness::FillTGridLocal()
{
    //Apply different algorithm to fill the Roughness grid
    //Now used:
    //      -Roughness (through Ransac and standard deviation)
    RansacAlgorithm ransac;
    ransac.k = ransac_iterations;
    MapCreator map_creator;

    map_creator.MakeMap(TGridLocal, 0);

    for(int ind_x=0; ind_x<nb_cells; ind_x++)
    {
        for(int ind_y=0; ind_y<nb_cells; ind_y++)
        {
            vector<double> best(4);
            int ind_x_low = ind_x / low_grid_resolution;
            int ind_y_low = ind_y / low_grid_resolution;

            // Edge case for odd numbers
            int nb_low_cells = nb_cells/low_grid_resolution;
            if(ind_x_low == nb_low_cells)
            {
                ind_x_low = nb_low_cells - 1;
            }
            if(ind_y_low == nb_low_cells)
            {
                ind_y_low = nb_low_cells - 1;
            }
            
            // Enough points in the cell to fit a plane
            if(PCGrid[ind_x][ind_y].size()>3)
            {
                ransac.FitPlane(1, PCGrid[ind_x][ind_y], best);
                double std = CalculateStd(ransac.distances);
                
                // Roughness grid
                TGridLocal[ind_x][ind_y][0] = roughness_normalization(std, roughness_threshold);
            }

            // Rough estimation based on the neighbors cells
            else if(PCLowGrid[ind_x_low][ind_y_low].size()>3)
            {
                ransac.FitPlane(1, PCLowGrid[ind_x_low][ind_y_low], best);
                double std = CalculateStd(ransac.distances);
                
                // Roughness grid
                TGridLocal[ind_x][ind_y][0] = roughness_normalization(std, roughness_threshold);
            }

            // No data result to unknown terrain
            else
            {
                if(!stylized)
                {
                    TGridLocal[ind_x][ind_y][0] = -1;
                }
            }
        }
    }
};



void Roughness::FillTGridGlobal(float x_pose, float y_pose)
{
    // int x = static_cast<int>(x_pose);
    // int y = static_cast<int>(y_pose);

    int x = static_cast<int>((x_pose*100)/globalGrid.resolution);
    int y = static_cast<int>((y_pose*100)/globalGrid.resolution);



    globalGrid.placeLocalGrid(TGridLocal, x, y);
    image_global_roughness = map_creator.MakeMap(globalGrid.grid, 0);
}



// =====================================================
// STD
// =====================================================

double Roughness::CalculateStd(vector<double>& distances)
{
    // Calculate mean
    double mean = accumulate(distances.begin(), distances.end(), 0.0)/distances.size();    

    // Calculate variance
    double var = 0.0;
    for(double data : distances)
    {
        var += (data - mean) * (data - mean);
    };
    var /= distances.size();

    // Return standard deviation
    return sqrt(var);
};





// =====================================================
// TOOLS
// =====================================================

// Function to calculate the norm of a 3D vector
double Roughness::calculateNorm(double linear_accel_x, double linear_accel_y, double linear_accel_z) {
    return std::sqrt(linear_accel_x * linear_accel_x +
                     linear_accel_y * linear_accel_y +
                     linear_accel_z * linear_accel_z);
};


// Normalize the values. (everything above the threshold get the LETHAL value).
double Roughness::roughness_normalization(double value, float threshold)
{
    // Normalize the roughness between 0 and 100
    // Any value above the threshold will be applied with the LETHAL value (100)
    double normalized_value;
    if (value >= threshold)
    {
        normalized_value = LETHAL;
    }
    else
    {
        if(value < min_roughness)
        {
            min_roughness = value;
        }
        // normalized_value = (((value + roughness_shift )/ threshold)*100);
        normalized_value = (((value - min_roughness )/ (threshold-min_roughness))*100);

    }

    return normalized_value;
};


// Display the roughness (traversability) grid in the terminal
void Roughness::DisplayGrid()
{
    for(long unsigned int ind_x=0; ind_x<TGridLocal.size(); ind_x++)
    {
        for(long unsigned int ind_y=0; ind_y<TGridLocal[0].size(); ind_y++)
        {
            cout<<"|"<<TGridLocal[ind_x][ind_y][0]<<"|"<<TGridLocal[ind_x][ind_y][1]<<"|"<<TGridLocal[ind_x][ind_y][2]<<"|   ";
        }
        cout<<endl;
        cout<<endl;
    }
};


// In case the user has a .pcd file. (Used for debug purposes)
void Roughness::ImportPCD(string path)
// Takes absolute path as an input, import a .pcd file and transform it into a PCL object
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);

    cout<<"PCD path: "<<path<<endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (path, *cloud_in) == -1) //* load the file
    {
        cout<<"Couldn't read file test_pcd.pcd"<<endl;
    }
    cout<<"Point cloud loaded from file"<<endl;
    cloud = cloud_in;
};



void Roughness::TestGrid()
{
    // Example 1: Place a local grid with positive offset.
    // Each cell is a vector<double> of size 3.
    TerrainGrid localGrid1 = {
        { {1.0, 1.1, 1.2}, {2.0, 2.1, 2.2} },
        { {3.0, 3.1, 3.2}, {4.0, 4.1, 4.2} }
    };
    // Place localGrid1 at world coordinate (0, 0)
    globalGrid.placeLocalGrid(localGrid1, 0, 0);
    std::cout << "After placing localGrid1 at (0,0):\n";
    globalGrid.print();

    // Example 2: Place another local grid with negative offset.
    TerrainGrid localGrid2 = {
        { {9.0, 9.1, 9.2} }
    };
    // Place localGrid2 at world coordinate (-1, -1)
    globalGrid.placeLocalGrid(localGrid2, -1, -1);
    std::cout << "After placing localGrid2 at (-1,-1):\n";
    globalGrid.print();

    // Example 3: Place a third local grid at a different position.
    TerrainGrid localGrid3 = {
        { {7.0, 7.1, 7.2}, {8.0, 8.1, 8.2} }
    };
    // Place localGrid3 at world coordinate (2, -2)
    globalGrid.placeLocalGrid(localGrid3, 2, -2);
    std::cout << "After placing localGrid3 at (2,-2):\n";
    globalGrid.print();

    return;
};
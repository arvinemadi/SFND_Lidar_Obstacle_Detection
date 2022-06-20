/*
As part of a Udacity sensor fusion project - the original uncomplete code from Udacity author mentioned below
Completed by Arvin.Emadi@Gmail.com
*/

/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"

#include <unordered_map>
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

// Only for debug and first trying - simulates a highway
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }
    return cars;
}

// Only for debug and first trying - simulates a highway
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    // RENDER OPTIONS
    bool renderScene = true;
    bool renderBoxOption = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create a "simulated" lidar sensor - parameters can be adjusted in lidar.h
    Lidar* my_Lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pointCloud = my_Lidar->scan();

    // Creating a pointProcesser object to run different functions on the point clouds
    // The pointProcessor is shown in a separate file
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;// = new ProcessPointClouds<pcl::PointXYZ>();
    
    //Option to render the simulated Rays if needed
    //renderRays(viewer, my_Lidar->position, lidar_pointCloud);

    //segmenting the road and obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> road_obstacles_cloud = pointProcessor.SegmentPlane(lidar_pointCloud, 200, 0.3);
    
    //Do the clustering on the obstacles 
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(road_obstacles_cloud.first, 1.2, 10, 300);
    
    //Visualize each of the clusters 
    //Option to put a bonding box around each cluster
    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(0,1,0), Color(0,0,1) };
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
        ++clusterId;
        //if selected using PCL library bounding box function calculate and render bonding box for each cluster
        if (renderBoxOption)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
    }
    
    //Options for Rendering the point clouds for all points or segmented road and obstacles.
    //renderPointCloud(viewer, lidar_pointCloud, "new_cloud", Color(1, 1, 1));
    //renderPointCloud(viewer, road_obstacles_cloud.first, "obstacles", Color(1, 0, 0));
    //renderPointCloud(viewer, road_obstacles_cloud.second, "road", Color(1, 1, 1));
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }
    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

//main function to load data and do processing
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>*  pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    //some settings:
    bool renderBoxOption = true; //Show Box around clusters

    //option to load the data here if dealing with single image for debug and code development
    // when there is an stream of data it is passed to this function as arguments
    //Loading the data when not in function argument or for debug 
    //std::string dataPath = "../src/sensors/data/pcd/data_1/0000000000.pcd";
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd(dataPath);
    //cout << "Original data size: "<<inputCloud->points.size() << endl;

    //Filtering the data
    Eigen::Vector4f minPoint(-10, -6.0, -2.5, 1);
    Eigen::Vector4f maxPoint(30, 6.0, 4, 1);
    //FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, .5, minPoint, maxPoint);
    //cout << "Filtered data size: " << filteredCloud->points.size() << endl;
        
    //segmenting the road and obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> road_obstacles_cloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.35);

    //Do the clustering on the obstacles 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(road_obstacles_cloud.first, .4, 40, 2000);
    
    //Visualize each of the clusters 
    //Option to put a bonding box around each cluster
    
    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(0,1,1), Color(1,0,1), Color(1,1,1) };
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size ";
        //pointProcessorI->numPoints(cluster);
        Box box = pointProcessorI->BoundingBox(cluster);
        if (box.z_max - box.z_min < 0.3)
            continue;
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 6]);
        
        //if selected using PCL library bounding box function calculate and render bonding box for each cluster
        if (renderBoxOption)
        {
            
            renderBox(viewer, box, clusterId, colors[clusterId % 6]);
        }
        ++clusterId;
    }
    renderPointCloud(viewer, road_obstacles_cloud.second, "road", Color(0, 1, 0));
    //renderPointCloud(viewer, road_obstacles_cloud.first, "Obstacles", Color(1,0,0));
    //renderPointCloud(viewer, filteredCloud, "inputCloud");
}

int main (int argc, char** argv)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    //load the data
    std::string dataPath = "C:/Users/HP/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1";
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(dataPath);
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        //Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //Load next pcd
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

        //Process and show the cloud
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}
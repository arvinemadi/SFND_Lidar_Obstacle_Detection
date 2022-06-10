// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "ransac3D.h"
#include "kdtree3D.h"
#include "boost/filesystem.hpp"
#include <unordered_map>
#include <iostream>

//using namespace st;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>::Ptr voxFiltered_cloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr regFitered_cloud (new pcl::PointCloud<PointT>);


    //std::cout<<"Input data size is : " << cloud->points.size()<<std::endl;

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Two filterings are applied 1- Voxel filtering to downsample 2- Region of interest filtering
    //1- Voxel filtering to downsample - using PCL library - e.g. https://pointclouds.org/documentation/tutorials/voxel_grid.html
     // Create the filtering object
    pcl::VoxelGrid<PointT> voxFilter;
    voxFilter.setLeafSize(filterRes / 4, filterRes , filterRes / 15);
    voxFilter.setInputCloud(cloud);
    voxFilter.filter(*voxFiltered_cloud);
    //std::cout<<"Voxel Filtered data size is : " << voxFiltered_cloud->points.size()<<std::endl;

    //2- Region based filtering using Crop Box - e.g https://pointclouds.org/documentation/classpcl_1_1_crop_box_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html
    
    typename pcl::CropBox<PointT> regFilter(true);
    regFilter.setMin(minPoint);
    regFilter.setMax(maxPoint);
    regFilter.setInputCloud(voxFiltered_cloud);
    regFilter.filter(*regFitered_cloud);
    
    //another step to remove roof point - values and procedure provided by original author
    std::vector<int> roof_points_indices;
    typename pcl::CropBox<PointT> roofFilter(true);
    roofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roofFilter.setInputCloud(regFitered_cloud);
    roofFilter.filter(roof_points_indices); //indices of the roof points will be here - need to take them out of cloud

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point : roof_points_indices)
        inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(regFitered_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*regFitered_cloud);
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    //std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return regFitered_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // Create two new point clouds, one cloud with obstacles and other with segmented plane
    
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planeCoud (new pcl::PointCloud<PointT>);
    
    //There is an extract function in PCL for separating one cloud from the other and is shown in comments below
    //But the function crashed so a simple Hashtable version is created below: Time and Memory complexity O(n)

    std::unordered_map<int, bool> planeMap;
    for (int i : inliers->indices) {
            planeCoud->points.push_back(cloud->points[i]);
            planeMap[i] = true;
        }

    for (int i = 0; i < (cloud->points).size(); i++) {
        if (!planeMap[i])
            obstCloud->points.push_back(cloud->points[i]);
    }

    //This is intended to do the same as above but causes memory crash
    //typename pcl::ExtractIndices<PointT> extract(true);
    //extract.setInputCloud(cloud);
    //extract.setIndices(inliers);
    //extract.setNegative(true);
    //extract.filter(*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCoud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    //some control for debug
    bool use_my_ransac = true;

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    if (use_my_ransac)
    {
        // Finding the inliers to the road segment
        // Set to use RANSAC algorithm - RANSAC implementation is included in a ransac3D.h
        std::unordered_set<int> plane_inlier_indices = My_Ransac3D(cloud, maxIterations, distanceThreshold);
        std::cout << plane_inlier_indices.size() << std::endl;
        auto ransacTime = std::chrono::steady_clock::now();
        auto elapsedTime1 = std::chrono::duration_cast<std::chrono::milliseconds>(ransacTime - startTime);
        std::cout << "RANSAC took " << elapsedTime1.count() << " milliseconds" << std::endl;

        //Separate road and obstacle clouds
        //Note that this may not be the most efficient way
        //Set timer to compare this implementation with PCL methods
        pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
        pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

        for (int index = 0; index < cloud->points.size(); index++)
        {
            PointT point = cloud->points[index];
            if (plane_inlier_indices.count(index))
                cloudInliers->points.push_back(point);
            else
                cloudOutliers->points.push_back(point);
        }

        //Giving some comments on size and time of calculations
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime2 = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "RANSAC + separation took " << elapsedTime2.count() << " milliseconds" << std::endl;

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
        return segResult;
    }
    else
    {
        typename pcl::SACSegmentation<PointT> seg;
        typename pcl::PointIndices::Ptr inliers(new typename pcl::PointIndices);
        typename pcl::ModelCoefficients::Ptr coefficients(new typename pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(maxIterations);
        seg.setDistanceThreshold(distanceThreshold);

        //Input the cloud with all the Lidar points
        seg.setInputCloud(cloud);
        //Generate inliners to the plance and get the coefficients
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0)
        {
            std::cout << "No plane could be found" << std::endl;
        }

        //Giving some comments on size and time of calculations
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
        //std::cout << "Number of points in inliers is " << inliers->indices.size() << std::endl;

        //Now that we have the two segments of road and obstacles, we need to separate them 
        //In principle we want to take road out of all points to be left with the a cloud of obstacles - that we would cluster into 
        //separate obstacles later on
        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
        return segResult;
    }

    
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    //Here we cluster all the obstacle points that were segmented before
    //some controls for the code
    bool use_my_kdTree = true;
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    if (use_my_kdTree)
    {
        KdTree<PointT>* tree = new KdTree<PointT>;
        std::vector<PointT> points;
        for (int i = 0; i < cloud->points.size(); i++)
        {
            tree->insert(cloud->points[i], i);
            points.push_back(cloud->points[i]);
        }

        std::vector<std::vector<int>> cluster_indices = tree->euclideanCluster(points, tree, clusterTolerance);


        int j = 0;
        for (std::vector<int> cloudIndices : cluster_indices)
        {
            if (cloudIndices.size() < minSize || cloudIndices.size() > maxSize)
                continue;

            typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            for (int i : cloudIndices)
                cloud_cluster->points.push_back(cloud->points[i]);

            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
            j++;
        }

        return clusters;
    }
    else
    {
        // function to perform euclidean clustering to group detected obstacles
        // Used PCL documentation @ https://pointclouds.org/documentation/tutorials/cluster_extraction.html
        // Creating the KdTree object for the search method of the extraction
        // The algorithm for clustering is implemented and shown in a separate folder
        // Here the code uses PCL functions and data structures
        // Clustering is based on KdTree and Binary Search
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(clusterTolerance); // in cm
        ec.setMinClusterSize(minSize);
        ec.setMaxClusterSize(maxSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        int j = 0;
        //for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        for (pcl::PointIndices getIndices : cluster_indices)
        {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
            for (int i : getIndices.indices)
                cloud_cluster->points.push_back(cloud->points[i]);

            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
            j++;
        }

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        //std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

        return clusters;
    }
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
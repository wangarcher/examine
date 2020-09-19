/*

Project: loop determination
Date: 20/08/12
@Author: Wang.
Detail: a sequence contains the 
Scenario: loop detection

*/
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <typeinfo>
#include <cmath>

//pcl related
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
//filters
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>


#define THRESHOLD_SCORE 0.1


using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudN;

typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> PointToPlane;

struct PCDMap
{
    int index; //new
    Eigen::Matrix4f global_transform;
    PointCloud::Ptr cloud;
    PCDMap():cloud(new PointCloud){};
};

//another attempt for p2p
//it seems worked, just for now
void point_to_plane (const PointCloud::Ptr& cloud1, const PointCloud::Ptr& cloud2, Eigen::Matrix4f&final_transform, float&score) 
{
    pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*cloud1, *src);
    pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*cloud2, *tgt);

    pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
    norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
    norm_est.setKSearch (10);
    norm_est.setInputCloud (tgt);
    norm_est.compute (*tgt);

    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
    boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
    icp.setTransformationEstimation(point_to_plane);
    icp.setInputCloud(src);
    icp.setInputTarget(tgt);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-3);
    pcl::PointCloud<pcl::PointNormal> output;
    icp.align(output);
    final_transform = icp.getFinalTransformation();
    score = icp.getFitnessScore();
}

//uniform sampling func
//data acquired: pcl::PointCloud<PointNormal> PointCloudN
void uniform_sampling (PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered(new PointCloud);
    //cout << "Cloud points number is:" << cloud->points.size() << endl;
    pcl::UniformSampling<PointT> US;
    US.setInputCloud(cloud);
    US.setRadiusSearch(0.25f);
    US.filter(*cloud_filtered);
    *cloud = *cloud_filtered;
    //cout << "Cloud points number after uniform sampling:" << cloud->points.size() << endl;
}


//remove singel func
//data acquired: pcl::PointCloud<PointNormal> PointCloudN
void remove_single (PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered(new PointCloud);
    //cout << "Cloud points number is:" << cloud->points.size() << endl;
    pcl::RadiusOutlierRemoval<PointT> RS;
    RS.setInputCloud(cloud);
    RS.setRadiusSearch(0.5);
    RS.setMinNeighborsInRadius(4);
    RS.filter(*cloud_filtered);
    *cloud = *cloud_filtered;
    //cout << "Cloud points number after remove single:" << cloud->points.size() << endl;

}

//another new filter to remove the point faraway from the origin
//added on 7/30 by Wang
//data acquired: pcl::PointCloud<PointNormal> PointCloudN
void remove_distant (PointCloud::Ptr cloud)
{
    PointCloud::Ptr cloud_filtered(new PointCloud);
    //cout << "Cloud points number is:" << cloud->points.size() << endl;
    pcl::StatisticalOutlierRemoval<PointT> RD;
    RD.setInputCloud(cloud);    
    RD.setMeanK(50);
    RD.setStddevMulThresh(1.0);
    RD.filter(*cloud_filtered);
    *cloud = *cloud_filtered;
    //cout << "Cloud points number after remove distant:" << cloud->points.size() << endl;

}


// input two PCDMap struct
// finds out their real global transformation 
// index are: int start & int end
int main()
{
    PCDMap start, end; 
    pcl::io::loadPCDFile ("../1.pcd", *start.cloud);
    pcl::io::loadPCDFile ("../2.pcd", *end.cloud);
    start.index = 2000;
    end.index = 6000;
    //it's not over, refinement needed.
    //we need to get the data from other process
    //directly initialization is a temporary method
    uniform_sampling(start.cloud);
    remove_single(start.cloud);
    remove_distant(start.cloud);
    uniform_sampling(end.cloud);
    remove_single(end.cloud);
    remove_distant(end.cloud);

    float score = 0; 
    int flag = 0;
    Eigen::Matrix4f transformation_matrix;
    point_to_plane(start.cloud, end.cloud, transformation_matrix, score);
    if (score < THRESHOLD_SCORE) 
    {
        cout << "seems fxxking fine for now, you sucker" << endl;
        flag = 1;
        cout << "according to time sequence, the loop is from " << start.index << " to " << end.index <<endl;
        
    }
    else
    {
        cout << "I am a stupid fxxking asshole." << endl;
    }
    return 0;
}

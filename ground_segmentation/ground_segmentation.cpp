#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/project_inliers.h>

using namespace std;

void planeSeg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
              pcl::ModelCoefficients::Ptr coefficients,
              pcl::PointIndices::Ptr planeIndices)
{
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setInputCloud(cloud);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.40);
  segmentation.setOptimizeCoefficients(true);
  segmentation.segment(*planeIndices, *coefficients);
}


void removePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                 pcl::PointIndices::Ptr cloudIndices,
                 bool setNeg)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setNegative(setNeg);
  extract.setInputCloud(inputCloud);
  extract.setIndices(cloudIndices);
  extract.filter(*outCloud);
}


int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile ("../test.pcd", *cloud);


  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);

  planeSeg(cloud, coefficients, inliers);

  if (inliers->indices.size () < 1000)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }
  else

  {    
    // Copy the points of the plane to a new cloud.
    removePlane(plane, cloud, inliers, false);

    // Retrieve the convex hull.
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(plane);
    
    // Make sure that the resulting hull is bidimensional.
    hull.setDimension(3);
    hull.reconstruct(*convexHull);

    // Prism object.
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud(cloud);
    prism.setInputPlanarHull(convexHull);

    prism.setHeightLimits(-2.0, 5.0);
    pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

    prism.segment(*objectIndices);

    //removePlane(objects, cloud, objectIndices, false);
    // Get points retrieved by the hull. 
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;



  //planeSeg(objects, coefficients, inliers);

  //removePlane(plane, objects, inliers, false);




  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_handler (plane, 255, 255, 255);
  viewer.addPointCloud (plane, cloud_handler, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");


 
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }
  return (0);
}

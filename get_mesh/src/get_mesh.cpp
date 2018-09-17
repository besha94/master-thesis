#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/surface/mls.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/impl/convex_hull.hpp>
#include <pcl/common/transforms.h>

#include <chrono>


pcl::PointCloud<pcl::PointXYZ> SACSegmentation(pcl::PointCloud<pcl::PointXYZ>& input);
pcl::PointCloud<pcl::PointXYZ> cloudFiltering(pcl::PointCloud<pcl::PointXYZ>& no_ground);
pcl::PointCloud<pcl::PointXYZ> cloudSmoothing(pcl::PointCloud<pcl::PointXYZ>& no_ground);
pcl::PolygonMesh cloudTriangulation(pcl::PointCloud<pcl::PointXYZ>& cloud_filtered);
pcl::PointCloud<pcl::PointXYZ> cloudTransform(pcl::PointCloud<pcl::PointXYZ>& mls);


//PARAMETRI
double delta = 3.14;

int main (int argc, char** argv)
{	
	
	pcl::PointCloud<pcl::PointXYZ> cloud_hull;
	
	pcl::PointCloud<pcl::PointXYZ> input;
 	if (pcl::io::loadPLYFile<pcl::PointXYZ> ("spuzva_raw.ply", input) == -1)
    {
	  PCL_ERROR ("Could not read file. \n");
	  return (-1);
    }
	
	//pcl::io::savePCDFile("upravljac_nhaf.pcd", input, true);
	
	//auto start = std::chrono::high_resolution_clock::now();	

	pcl::PointCloud<pcl::PointXYZ> no_ground (SACSegmentation(input));
	
	pcl::PointCloud<pcl::PointXYZ> cloud_filtered (cloudFiltering(no_ground));
	
	pcl::PointCloud<pcl::PointXYZ> mls_cloud (cloudSmoothing(cloud_filtered));
	
	pcl::PointCloud<pcl::PointXYZ> cloud_transformed(cloudTransform(mls_cloud));

	pcl::PolygonMesh triangles(cloudTriangulation(cloud_transformed));

	//cloudTransform(mls_cloud);

	//auto finish = std::chrono::high_resolution_clock::now();

	//auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(finish-start);

	//std::cout << microseconds.count() << "µs\n"; 


	//pcl::io::savePLYFile("casa_haf.ply", input, true);
	

}

pcl::PointCloud<pcl::PointXYZ> SACSegmentation(pcl::PointCloud<pcl::PointXYZ>& input) {

	//--SAC Segmentation--
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
 	pcl::PointCloud<pcl::PointXYZ> no_ground;
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.295);
	seg.setMaxIterations(1000);
    seg.setInputCloud (input.makeShared());
    seg.segment (*inliers, *coefficients); 
	//pcl::copyPointCloud<pcl::PointXYZRGB>(*input, inliers->indices, *no_ground);
	 
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true); 
    extract.filter(no_ground);

	if (inliers->indices.size () == 0)
	{
    	PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	} 
	
	pcl::io::savePLYFileASCII ("cloud_ng.ply", no_ground);
	
	return no_ground;
}

pcl::PointCloud<pcl::PointXYZ> cloudFiltering(pcl::PointCloud<pcl::PointXYZ>& no_ground) {

	 // Create the filtering object
	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  	sor.setInputCloud (no_ground.makeShared());
  	sor.setMeanK (50);
  	sor.setStddevMulThresh (1.0);
  	sor.filter (cloud_filtered);

	pcl::io::savePLYFileASCII ("cloud_filtered.ply", cloud_filtered);
	//pcl::io::savePCDFileASCII ("cloud_filtered.pcd", cloud_filtered);

	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ> cloudSmoothing(pcl::PointCloud<pcl::PointXYZ>& no_ground) {
	
  	// Create a KD-Tree
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

 	// Output has the PointNormal type in order to store the normals calculated by MLS
  	pcl::PointCloud<pcl::PointNormal> mls_points;

  	// Init object (second point type is for the normals, even if unused)
  	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
 	mls.setComputeNormals (true);

  	// Set parameters
  	mls.setInputCloud (no_ground.makeShared());
  	mls.setPolynomialFit (true);
  	mls.setSearchMethod (tree);
  	mls.setSearchRadius (0.03);

  	// Reconstruct
  	mls.process (mls_points);

  	pcl::PointCloud<pcl::PointXYZ> mls_cloud;
	pcl::copyPointCloud(mls_points, mls_cloud);
	
  	pcl::io::savePLYFile ("cloud_smoothed.ply", mls_cloud);
  	
  	return mls_cloud;

}

pcl::PolygonMesh cloudTriangulation(pcl::PointCloud<pcl::PointXYZ>& cloud_filtered) {

	// Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered.makeShared());
    n.setInputCloud (cloud_filtered.makeShared());
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (cloud_filtered, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.5);
	
	// Set typical values for the parameters
    gp3.setMu (3); // 2.5-3.0
    gp3.setMaximumNearestNeighbors (200); // 100-200
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);
	pcl::io::savePLYFile ("cloud_triangulated.ply", triangles);
	
	return triangles;
}

pcl::PointCloud<pcl::PointXYZ> cloudTransform(pcl::PointCloud<pcl::PointXYZ> &mls) {

	Eigen::Vector4f centroid;	
	pcl::ConvexHull<pcl::PointXYZ> chull;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	chull.setInputCloud(mls.makeShared());
	chull.setDimension(3);
	chull.setComputeAreaVolume(true);
	chull.reconstruct (*cloud_hull);
	
	pcl::compute3DCentroid(*cloud_hull, centroid);	
	
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

	transform_1 (0,3) = -centroid[0];
	transform_1 (1,3) = -centroid[1];
	transform_1 (2,3) = -centroid[2];

	transform_2 (0,0) = cos (-delta);
 	transform_2 (0,2) = sin(-delta);
 	transform_2 (2,0) = -sin (-delta);
 	transform_2 (2,2) = cos (-delta);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
 	pcl::transformPointCloud (mls, *transformed_cloud, transform_1);
	pcl::transformPointCloud (*transformed_cloud, *transformed_cloud, transform_2);

	pcl::io::savePLYFile ("cloud_transformed.ply", *transformed_cloud);

	return *transformed_cloud;
}













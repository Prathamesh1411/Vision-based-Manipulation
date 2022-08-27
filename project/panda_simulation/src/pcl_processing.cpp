#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <thread>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.h>

using namespace std;
using namespace ros;
using namespace pcl;

typedef pcl::PointCloud<PointXYZ> PointCloud;


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Simple Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr normalsVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer1"));
  viewer->setBackgroundColor (0, 0, 0);
//   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZ> rgb(cloud);
//   viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb, "sample cloud");
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::PointCloud<PointXYZRGB> downSample( pcl::PointCloud<PointXYZRGB> cloudRGB) {
	pcl::VoxelGrid<PointXYZRGB> sor;
	sor.setInputCloud (cloudRGB.makeShared());
	sor.setLeafSize (0.008f, 0.008f, 0.008f);
	sor.filter (cloudRGB);
	return (cloudRGB);
}

pcl::PointCloud<PointXYZ> distanceFilter( pcl::PointCloud<PointXYZ> cloud) {
	pcl::PassThrough<pcl::PointXYZ> passZ;
	passZ.setInputCloud (cloud.makeShared());
	passZ.setFilterFieldName ("z");
	passZ.setFilterLimits (0.25, 1.0);
	passZ.filter (cloud);

	pcl::PassThrough<pcl::PointXYZ> passX;
	passX.setInputCloud (cloud.makeShared());
	passX.setFilterFieldName ("x");
	passX.setFilterLimits (-0.15, 0.25);
	passX.filter (cloud);

	pcl::PassThrough<pcl::PointXYZ> passY;
	passY.setInputCloud (cloud.makeShared());
	passY.setFilterFieldName ("y");
	passY.setFilterLimits (-0.25, 0.25);
	passY.filter (cloud);
	return(cloud);
}

std::mutex viewerMutex;
pcl::visualization::PCLVisualizer::Ptr viewerN;
pcl::visualization::PCLVisualizer::Ptr viewerN1, n2, n3, n4, n5, n6, n7, n8;

pcl::PointCloud<PointXYZRGB>::iterator b1;
pcl::PointCloud<PointXYZ>::iterator b2;
pcl::PointCloud<PointXYZRGB> cloudRGB;
pcl::PointCloud<PointXYZRGB> cloudRGB1;
pcl::PointCloud<PointXYZ> cloud, cloud1, cloud2;
sensor_msgs::PointCloud2 sensMsg1, sensMsg2, sensMsg3;

pcl::PointCloud<PointXYZRGB> majorPlaneRGB;
pcl::PointCloud<PointXYZ> majorPlane;

pcl::PointCloud<PointXYZ> object;
pcl::PointCloud<PointXYZ> objectPlane;
pcl::PointCloud<PointXYZRGB> objectPlaneRGB;
vector<int> inliers;
Publisher pub;
bool firstCB = true;
bool capture = false;
void foo() {
	cloudRGB = downSample(cloudRGB);

	pcl::copyPointCloud(cloudRGB, cloud);

	// n3 = simpleVis(cloud.makeShared());
	// pcl::copyPointCloud(*cloud_filteredRGB, *cloud_filtered);
	

	// n3 = simpleVis(cloud.makeShared());
	cloud = distanceFilter(cloud);

	// viewerN1 = simpleVis(cloud.makeShared());

	// n4 = simpleVis(cloud.makeShared());

	for(b1 = cloudRGB.points.begin(), b2 = cloud.points.begin(); b1 < cloudRGB.points.end(); b1++, b2++) {
		b1->y = -b1->y;
		b1->z = -b1->z;

		b2->y = -b2->y;
		b2->z = -b2->z;
	}

	
	

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud.makeShared()));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (.012);
	ransac.computeModel();
	ransac.getInliers(inliers);

	pcl::copyPointCloud (cloudRGB, inliers, majorPlaneRGB);
	pcl::copyPointCloud (cloud, inliers, majorPlane);
	
	// n5 = simpleVis(majorPlane.makeShared());

	pcl::PointIndices::Ptr inliers1 (new pcl::PointIndices ());
	inliers1->indices = inliers;

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud.makeShared());
	extract.setIndices(inliers1);
	extract.setNegative(true);
	extract.filter(object); 

	// n6 = simpleVis(object.makeShared());

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_o (new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(object.makeShared()));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac1 (model_o);
	ransac1.setDistanceThreshold (0.01);
	ransac1.computeModel();
	ransac1.getInliers(inliers);
	pcl::copyPointCloud (object, inliers, objectPlane);
	Eigen::VectorXf coeff;
	ransac1.getModelCoefficients(coeff);
	cout << coeff << endl;

	// n7 = simpleVis(objectPlane.makeShared());
	

	pcl::CentroidPoint<pcl::PointXYZ> centroid;

	for(b2 = objectPlane.points.begin(); b2 < objectPlane.points.end(); b2++) {
		pcl::PointXYZ p(b2->x, b2->y, b2->z);
		centroid.add(p);
	}
	pcl::PointXYZ c;
	centroid.get(c);

	cout << c << endl;

	
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (objectPlane.makeShared());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (0.03);
	ne.compute (*cloud_normals);
	pcl::PointCloud<PointXYZRGB> objectPlaneRGB;
	pcl::copyPointCloud (objectPlane, objectPlaneRGB);

	n8 = normalsVis(objectPlane.makeShared(), cloud_normals);

	// viewerN.addPointCloudNormals<pcl::PointXYZ,pcl::Normal> (objectPlane, cloud_normals);

	pcl::PointXYZ p(coeff[0], coeff[1], coeff[2]);
	pcl::ModelCoefficients line_coeff;
	line_coeff.values.resize (6);    // We need 6 values
	line_coeff.values[0] = c.x;
	line_coeff.values[1] = c.y;
	line_coeff.values[2] = c.z;
	cout << coeff.x() << ", " << coeff.y() << ", " << coeff.z() << endl;
	line_coeff.values[3] = coeff.x();
	line_coeff.values[4] = coeff.y();
	line_coeff.values[5] = coeff.z();
	// viewerN->addSphere(c, 0.005, "sphere");
	n8->addSphere(c, 0.005, "sphere1");
	n8->addLine(line_coeff, "line");
	n8->addLine(c, pcl::PointXYZ(c.x, c.y+1, c.z), "line1");

	// geometry_msgs::PoseStamped pose;
	// pose.pose.position.x = c.x;
	// pose.pose.position.y = c.y;
	// pose.pose.position.z = c.z;

	// pose.pose.orientation.x = coeff.x();
	// pose.pose.orientation.y = coeff.y();
	// pose.pose.orientation.z = coeff.z();
	// pose.pose.orientation.w = coeff.w();

	// // pub.publish(pose);

	// while (!(n4->wasStopped ()))
	// {
	// 	// viewerN1->spinOnce (100);
	// 	// viewerN->spinOnce (100);
	// 	// n2->spinOnce(100);
	// 	// n3->spinOnce(100);
	// 	n4->spinOnce(100);
	// 	// n5->spinOnce(100);
	// 	// n6->spinOnce(100);
	// 	// n7->spinOnce(100);
	// 	// n8->spinOnce(100);
	// 	std::this_thread::sleep_for(100ms);
	// }
}
class transFunc {
	public:
	tf::TransformListener tfListener;
	sensor_msgs::PointCloud2 c3;
	sensor_msgs::PointCloud2 getConverted(sensor_msgs::PointCloud2 c1) {
		
		pcl_ros::transformPointCloud("/pcl_1_frame", c1, c3, tfListener);
		return c3;
	}
};
int counter = 0;
void cloudCB(const sensor_msgs::PointCloud2 &input)
{
      
	// fromROSMsg (input, cloud);
	if(counter == 0 && firstCB && capture) {
		fromROSMsg (input, cloudRGB);
		sensMsg1 = input;
		counter++;
		firstCB = false;
		// cout << "=============================" << endl;
		// foo();
	}
	else if(counter == 1 && firstCB && capture) {
		fromROSMsg (input, cloudRGB1);
		sensMsg2 = input;
		counter++;
		firstCB = false;
		// cout << "++++++++++++++++++++++++++" << endl;
		// foo();
	}
	if(counter > 1 && firstCB) {
		counter++;
		cloudRGB = downSample(cloudRGB);
		cloudRGB1 = downSample(cloudRGB1);

		pcl::copyPointCloud(cloudRGB, cloud);
		pcl::copyPointCloud(cloudRGB1, cloud1);

		cloud = distanceFilter(cloud);
		cloud1 = distanceFilter(cloud1);
		firstCB = false;

		transFunc aVar;
		sensMsg3 = aVar.getConverted(sensMsg2);

		fromROSMsg(sensMsg3, cloud2);
		
		n2 = simpleVis(cloud.makeShared());
		n3 = simpleVis(cloud2.makeShared());
		n8 = simpleVis(cloud1.makeShared());
	}
	// cout << "----------------------------" << endl;
	// cout << counter << endl;
	// n2 = rgbVis(cloudRGB.makeShared());
	// pcl::PointCloud<PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<PointXYZ>);
	// pcl::PointCloud<PointXYZRGB>::Ptr cloud_filteredRGB (new pcl::PointCloud<PointXYZRGB>);
	
	if(counter == 3 && !(n8->wasStopped() || n2->wasStopped() || n3->wasStopped())) {
		n8->spinOnce(10);
		n2->spinOnce(10);
		n3->spinOnce(10);
		std::this_thread::sleep_for(10ms);
	}
}

void combineClouds() {

}

void captureCB(std_msgs::Bool data) {
	cout << "00000000000000000000000000000000000000" << endl;
	if(data.data) {
		capture = true;
		// sleep(5);
		// foo();
	} else {
		capture = false;
		firstCB = true;
		if(counter == 2) {
			combineClouds();
		}
	}
}

int main(int argc, char** argv) {

    init(argc, argv, "pcl_node");
    NodeHandle nh;
    Subscriber sub = nh.subscribe("/panda_camera/depth/points", 1, cloudCB);
	Subscriber sub1 = nh.subscribe("/py_moveit_node/capture_pcl", 1, captureCB);
	pub = nh.advertise<geometry_msgs::PoseStamped>("/pcl_node/pose",1);
    sleep(1);
    spin();
    return 0;
}
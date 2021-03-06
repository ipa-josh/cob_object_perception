#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/registration/transforms.h>	//PCL 1.1

#include <pcl/filters/voxel_grid.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	if(argc<3) printf("help: transform ouput_file inputfile1 inputfile2 ...\n");
   
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	for(int i=2; i<argc; i++) {
		pcl::PointCloud<pcl::PointXYZRGB> pc, pc2;
		pcl::io::loadPCDFile(argv[i],pc);

		for(size_t i=0; i<pc.size(); i++)
			if(!(pc[i].x==0&&pc[i].y==0&&pc[i].z==0))
				pc2.push_back( pc[i] );

		Eigen::Vector3f trans = pc.sensor_origin_.head<3>();
		Eigen::Quaternionf rot = pc.sensor_orientation_;

		Eigen::Affine3f T = Eigen::Affine3f::Identity();
		T.translate(trans);
		T.rotate(rot);

		pcl::transformPointCloud (pc2, pc2, T);
		cloud += pc2;
	}

	pcl::io::savePCDFile(argv[1], cloud);

	pcl::PointCloud<pcl::PointXYZRGB> cloud_voxel;
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud.makeShared());
	sor.setLeafSize (0.005f, 0.005f, 0.005f);
	sor.filter ( cloud_voxel);
	pcl::io::savePCDFile(std::string(argv[1])+"_voxel.pcd", cloud);

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid (cloud_voxel, centroid);
	std::cout<<"centroid: "<<centroid.transpose()<<std::endl;

	return 0;
}

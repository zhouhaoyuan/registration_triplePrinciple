#include "utilities.h"
#include "recognition.h"
#include <iostream>
#include"sstream"


int main()
{

	std::string filePath = "E:\\vsProgram\\registration_triplePrinciple\\smallDavid";
	std::vector<std::string> files;
	std::string format = ".ply";
	getFiles(filePath, format, files);

	pcl::PLYReader reader;
	std::vector< FeatureCloud > cloudlist;

	for (int i = 0; i < files.size(); ++i)
	{
		PointCloudPtr cloud(new PointCloud);

		reader.read<PointT>(files[i], *cloud);

		//改变点云y方向
		//for (size_t i = 0; i < cloud->points.size(); ++i)
		//{
		//	cloud->points[i].y = -cloud->points[i].y;
		//}

		FeatureCloud tempCloud;
		tempCloud.setInputCloud(cloud);
		cloudlist.push_back(tempCloud);
	}

	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > trans_list;
	PointCloudPtr resultcloud(new PointCloud);
	pcl::copyPointCloud(*(cloudlist[0].getCloud()), *resultcloud);

	Eigen::Matrix4f tran = Eigen::Matrix4f::Identity();
	pcl::PLYWriter writer;
	for (int i = 0; i < files.size() - 1; ++i)
	{
		FeatureCloud temp1;
		FeatureCloud temp2;
		PointCloudPtr tempcloud1(new PointCloud);
		PointCloudPtr tempcloud2(new PointCloud);
		reader.read<PointT>(files[i], *tempcloud1);
		reader.read<PointT>(files[i + 1], *tempcloud2);
		temp1.setInputCloud(tempcloud1);
		temp2.setInputCloud(tempcloud2);

		//std::stringstream ss1,ss2;
		//ss1 << i;
		//writer.write("temp" + ss1.str() + ".ply", *tempcloud1, true);
		//ss2 << i + 1;
		//writer.write("temp" + ss2.str() + ".ply", *tempcloud2, true);

		Eigen::Matrix4f tranTemp = Eigen::Matrix4f::Identity();

		if (RotationTranslationCompute(temp1, temp2, tranTemp))
		{
			tran = tranTemp * tran;
			trans_list.push_back(tran);

			PointCloudPtr tempCloud(new PointCloud);
			pcl::copyPointCloud(*(cloudlist[i + 1].getCloud()), *tempCloud);

			pcl::transformPointCloud(*tempCloud, *tempCloud, tran);

			//std::stringstream ss;
			//ss << i ;			
			//writer.write( ss.str() + ".ply" , *tempCloud, true);
			std::cout << "comprehensive tran: " << std::endl << tran << std::endl;

			*resultcloud += *tempCloud;
		}
		else {
			std::cout << "Error: RotationTranslationCompute was failed. PointNum : " << i << std::endl;
		}
	}
	float resolution = 0.0;
	resolution = computeResolution(cloudlist[0].getCloud());
	float uniform_Radius = resolution * 3;
	//VoxelGrid_Filter(resultcloud, resultcloud, leafSize);
	//Uniform_Filter(resultcloud, resultcloud, uniform_Radius);
	writer.write("E:\\vsProgram\\registration_triplePrinciple\\RestrationResult.ply", *resultcloud, true);
	
	
	system("pause");
	return 0;
}
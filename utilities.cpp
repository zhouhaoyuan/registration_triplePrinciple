#include "utilities.h"

bool computePairNum(std::pair<double, std::string> pair1, std::pair<double, std::string> pair2)
{
	return pair1.first < pair2.first;
}
bool compareScore(std::pair<float, int> pair1, std::pair<float, int> pair2)
{
	return pair1.first < pair2.first;
}
void sort_filelists(std::vector<std::string>& filists)
{
	if (filists.empty())
		return;
	std::vector<std::pair<double, std::string> > filelists_pair;

	for (int i = 0; i < filists.size(); ++i) {

		std::string tmp_string = filists[i];
		int nposBEGIN = tmp_string.find_last_of("_");
		int nposEND = tmp_string.find_last_of(".");
		std::string tmp_num_string = tmp_string.substr(nposBEGIN + 1, nposEND);
		double tmp_num = atof(tmp_num_string.c_str());//atof()把字符串转换成浮点数
		std::pair<double, std::string> tmp_pair;
		tmp_pair.first = tmp_num;
		tmp_pair.second = tmp_string;
		filelists_pair.push_back(tmp_pair);
	}
	std::sort(filelists_pair.begin(), filelists_pair.end(), computePairNum);
	filists.clear();
	for (int i = 0; i < filelists_pair.size(); ++i) {
		filists.push_back(filelists_pair[i].second);
		std::cout << filists[i] << std::endl;
	}
}
//获取特定格式点云文件名
void getFiles(std::string path, std::string ext, std::vector<std::string>& files)
{
	//文件句柄    
	intptr_t  hFile = 0;
	//文件信息    
	struct _finddata_t fileinfo;
	std::string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之    
			//如果不是,加入列表    
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append("\\").append(fileinfo.name), ext, files);
			}
			else
			{
				std::string s = p.assign(path).append("\\").append(fileinfo.name);  //获取此文件的完整路径  
				char fileDrive[_MAX_DRIVE];
				char fileDir[_MAX_DIR];
				char fileName[_MAX_FNAME];
				char fileExt[_MAX_EXT];
				_splitpath(s.c_str(), fileDrive, fileDir, fileName, fileExt);  //将完整路径分解  
				if (strcmp(fileExt, ext.c_str()) == 0)  //筛选出符合后缀条件的文件  
				{
					std::string ss = p.assign(path).append("\\").append(fileinfo.name);					
					files.push_back(ss);
				}

			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
	sort_filelists(files);
};



//求重心
void computeMean(PointCloudPtr pointInput, Eigen::Vector3f &mean)
{
	if (pointInput->points.empty())
		return;

	// Initialize to 0
	mean.setZero();

	for (size_t i = 0; i < pointInput->points.size(); ++i)
	{
		mean[0] += pointInput->points[i].x;
		mean[1] += pointInput->points[i].y;
		mean[2] += pointInput->points[i].z;
	}
	mean /= (int)pointInput->points.size();
}
//求解点云重心，并减去重心
void remove_Mean(PointCloudPtr pointCloud_, Eigen::Vector3f& mean)
{
	clock_t start, end;
	start = clock();

	int npti = pointCloud_->size();
	for (size_t i = 0; i < npti; ++i)
	{
		Eigen::Vector3f p((*pointCloud_)[i].x,
			(*pointCloud_)[i].y,
			(*pointCloud_)[i].z);
		mean += p;
	}
	mean /= npti;

	for (size_t i = 0; i < npti; ++i)
	{
		(*pointCloud_)[i].x -= mean(0);
		(*pointCloud_)[i].y -= mean(1);
		(*pointCloud_)[i].z -= mean(2);
	}
	end = clock();
	std::cout << "remove_Mean has finished in "
		<< (end - start) / CLOCKS_PER_SEC << " s \n";
}
//求去重心的协方差矩阵并归一化
void computeCovarianceMatrixNormalized(PointCloudPtr pointInput,
	Eigen::Vector3f& centroid,
	Eigen::Matrix3f& covariance)
{
	if (pointInput->points.empty())
		return;
	
	covariance.setZero();

	unsigned int point_count = static_cast<unsigned int>(pointInput->points.size());
	for (size_t i = 0; i < point_count; ++i)
	{
		Eigen::Matrix<float, 3, 1> pt;
		pt[0] = pointInput->points[i].x - centroid[0];
		pt[1] = pointInput->points[i].y - centroid[1];
		pt[2] = pointInput->points[i].z - centroid[2];

		covariance(0, 0) += pt.x() * pt.x();
		covariance(0, 1) += pt.x() * pt.y();
		covariance(0, 2) += pt.x() * pt.z();
		covariance(1, 1) += pt.y() * pt.y();
		covariance(1, 2) += pt.y() * pt.z();
		covariance(2, 2) += pt.z() * pt.z();
	}
	covariance(1, 0) = covariance(0, 1);
	covariance(2, 0) = covariance(0, 2);
	covariance(2, 1) = covariance(1, 2);

	covariance /= point_count;

}
//归一化
void normalizePoints(PointCloudPtr cloudInput, Eigen::Vector3f& mean,
	float global_scale)
{
	int npti = cloudInput->points.size();
	if (mean[0] == 0 && mean[1] == 0 && mean[2] == 0)
	{
		for (int i = 0; i < npti; ++i)
		{
			Eigen::Vector3f p(cloudInput->points[i].x,
				cloudInput->points[i].y,
				cloudInput->points[i].z);
			mean = mean + p;
		}
		mean = mean / npti;

		for (int i = 0; i < npti; ++i)
		{
			cloudInput->points[i].x -= mean(0);
			cloudInput->points[i].y -= mean(1);
			cloudInput->points[i].z -= mean(2);
		}
	}

	float max_scale = 0;
	for (int i = 0; i < npti; ++i)
	{
		Eigen::Vector3f p(cloudInput->points[i].x,
						cloudInput->points[i].y,
						cloudInput->points[i].z);
		float temp = p.norm();//点积开方
		if (temp > max_scale)
			max_scale = temp;//获取最大距离
	}
	if (max_scale > global_scale)
		global_scale = max_scale;
	//将所有点归一化
	for (int i = 0; i < npti; ++i)
	{
		cloudInput->points[i].x /= global_scale;
		cloudInput->points[i].y /= global_scale;
		cloudInput->points[i].z /= global_scale;
	}
}
//计算分辨率
float computeResolution(PointCloudPtr cloud)
{
	clock_t start, end;
	start = clock();

	float resolution = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			resolution += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		resolution /= n_points;
	}

	std::cout << "resolution : " << resolution << endl;

	end = clock();
	std::cout << "computeResolution has finished in "
		<< (end - start) / CLOCKS_PER_SEC << " s \n";

	return resolution;
}
//去除NAN点
void removeNANfromNormal(FeatureCloud &cloud)
{
	PointCloudPtr nanremoved_(new PointCloud);
	PointCloudPtr cloud_ = cloud.getCloud();
	NormalsPtr normals_ = cloud.getNormal();
	std::vector<int> index;

	int num_point = cloud_->size();
	int num_normal = normals_->size();

	pcl::removeNaNNormalsFromPointCloud(*normals_, *normals_, index);
	pcl::copyPointCloud(*cloud_, index, *nanremoved_);
	cloud.setInputCloud(nanremoved_);

	// std::cout << "Remove NAN From Normals:" << std::endl;
	// std::cout << (num_normal - normals_->size()) << " Normals Are Removed: " \
	    // << num_normal << "->" << normals_->size() << std::endl;
// std::cout << (num_point - nanremoved_->size()) << " Points Are Removed: " \
    // << num_point << "->" << nanremoved_->size() << std::endl << std::endl;

}
//去除NAN
void removeNANfromFPFH(FPFH_features::Ptr feature_descriptor,
						FPFH_features::Ptr nanremoved,
							FeatureCloud& cloud)
{
	PointCloudPtr keyPoints_ptr_nanremoved(new PointCloud);
	pcl::PointIndices::Ptr indices_ptr_nanremoved(new pcl::PointIndices);

	for (int i = 0; i < feature_descriptor->points.size(); ++i)
	{
		//std::cout << feature_descriptor->points[i].histogram[0] << std::endl;
		float p = feature_descriptor->points[i].histogram[0];

		if ( !pcl_isfinite(p) )
		{
			continue;
		}
		else
		{
			nanremoved->push_back(feature_descriptor->points[i]);
			keyPoints_ptr_nanremoved->push_back( (cloud.getKeypoint() )->points[i]);
			indices_ptr_nanremoved->indices.push_back((cloud.getKeyPoint_indices())->indices[i]);
		}
	}

	cloud.setKeypoints(keyPoints_ptr_nanremoved);
	cloud.setKeyPoint_indices(indices_ptr_nanremoved);

	std::cout << "Remove NAN From Feature Descriptors:" << std::endl;
	std::cout << (feature_descriptor->points.size() - nanremoved->points.size())
		<< " Feature Descriptors Are Removed: "
		<< feature_descriptor->points.size() << "->"
		<< nanremoved->points.size() << std::endl ;

}
//SHOT去除NAN点
void removeNANfromSHOT(SHOT_features::Ptr feature_descriptor,
	SHOT_features::Ptr nanremoved,
	FeatureCloud& cloud)
{
	PointCloudPtr keyPoints_ptr_nanremoved(new PointCloud);
	pcl::PointIndices::Ptr indices_ptr_nanremoved(new pcl::PointIndices);

	for (int i = 0; i < feature_descriptor->points.size(); ++i)
	{
		float p = feature_descriptor->points[i].descriptor[0];

		if (!pcl_isfinite(p))
		{
			continue;
		}
		else
		{
			nanremoved->push_back(feature_descriptor->points[i]);
			keyPoints_ptr_nanremoved->push_back((cloud.getKeypoint())->points[i]);
			indices_ptr_nanremoved->indices.push_back((cloud.getKeyPoint_indices())->indices[i]);
		}
	}

	cloud.setKeypoints(keyPoints_ptr_nanremoved);
	cloud.setKeyPoint_indices(indices_ptr_nanremoved);

	std::cout << "Remove NAN From Feature Descriptors:" << std::endl;
	std::cout << (feature_descriptor->points.size() - nanremoved->points.size())
		<< " Feature Descriptors Are Removed: "
		<< feature_descriptor->points.size() << "->"
		<< nanremoved->points.size() << std::endl;
}
//体素滤波
void VoxelGrid_Filter(PointCloudPtr input,
						PointCloudPtr output,
							float leafsize)
{
	clock_t start, end;
	start = clock();

	int num = input->size();
	pcl::VoxelGrid<PointT> voxelgrid_filter;//对体素网格中所有点求均值,以期望均值点代替原始点集,更精确
	voxelgrid_filter.setLeafSize(leafsize, leafsize, leafsize);
	voxelgrid_filter.setInputCloud(input);
	voxelgrid_filter.filter(*output);

	//pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;//利用体素网格的中心（长方体的中心）代替原始点
	//approximate_voxel_filter.setLeafSize(leafsize, leafsize, leafsize);
	//approximate_voxel_filter.setInputCloud(input);
	//approximate_voxel_filter.filter(*output);

	std::cout << "VoxelGrid_Filter, Input points: " << num
		<< "; Output points: " << output->size() << std::endl;
	end = clock();
	std::cout << "VoxelGrid_Filter has finished in "
		<< (end - start) / CLOCKS_PER_SEC << " s \n";
}
//统计学滤波
void StatisticalOutlierRemoval_Filter(PointCloudPtr input,
									 PointCloudPtr output,
									 int K ,
									 float stddevMulThresh )
{
	clock_t start, end;
	start = clock();

	int num = input->size();
	pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
	statistical_filter.setMeanK(K);
	statistical_filter.setStddevMulThresh(stddevMulThresh);
	statistical_filter.setInputCloud(input);
	statistical_filter.filter(*output);
	std::cout << "StatisticalOutlierRemoval_Filter, Input points: " << num
		<< "; Output points: " << output->size() << std::endl;
	end = clock();
	std::cout << "StatisticalOutlierRemoval_Filter has finished in "
		<< (end - start) / CLOCKS_PER_SEC << " s \n";
}
//均匀滤波
void Uniform_Filter(PointCloudPtr input,
					PointCloudPtr output, 
					float uniform_Radius )
{
	clock_t start, end;
	start = clock();

	int num = input->size();
	pcl::UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud(input);
	uniform_sampling.setRadiusSearch(uniform_Radius);
	uniform_sampling.filter(*output);
	std::cout << "Uniform_Filter, Input points: " << num << "; Output points: " << output->size() << std::endl;

	end = clock();
	std::cout << "StatisticalOutlierRemoval_Filter has finished in "
		<< (end - start) / CLOCKS_PER_SEC << " s \n";
}
//直通滤波
void PassThrough_Filter(PointCloudPtr input,
	PointCloudPtr output,
	std::string axis,
	float upBound,
	float downBound,
	bool negative)
{
	clock_t start, end;
	start = clock();

	int num = input->size();

	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(input);
	pass.setFilterFieldName(axis);
	pass.setFilterLimits(downBound, upBound);
	pass.setFilterLimitsNegative(negative);//设置不在范围内的点保留还是去除
	pass.filter(*output);

	std::cout << "PassThrough_Filter, Input points: " << num << "; Output points: " << output->size() << std::endl;

	end = clock();
	std::cout << "PassThrough_Filter has finished in "
		<< (end - start) / CLOCKS_PER_SEC << " s \n";
}
//法向量 
void computeSurfaceNormals(FeatureCloud& cloud,
					  int K ,
					  float radius ,
					  int numofthreads )
{
	clock_t start, end;
	start = clock();

	NormalsPtr normals_(new SurfaceNormals);
	pcl::NormalEstimationOMP<PointT, NormalT> norm_est;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	norm_est.setNumberOfThreads(numofthreads);
	norm_est.setSearchMethod(tree);
	if (radius != 0) {

		norm_est.setRadiusSearch(radius);
	}
	else {
		norm_est.setKSearch(K);
	}
	norm_est.setInputCloud(cloud.getCloud());
	norm_est.compute(*normals_);
	std::vector<int> index;
	pcl::removeNaNNormalsFromPointCloud(*normals_, *normals_, index);
	cloud.setInputNormal(normals_);
	pcl::copyPointCloud(*cloud.getCloud(), index, *cloud.getCloud());
	end = clock();
	std::cout << "computeSurfaceNormals() has finished in "
		<< (end - start) / CLOCKS_PER_SEC << " s \n";
};

//构建法向量点云
void construct_PointNormal(FeatureCloud& targetCloud,
	FeatureCloud& sourceCloud)
{
	PointCloudNormal::Ptr pointNormal_src(new PointCloudNormal);
	PointCloudNormal::Ptr pointNormal_tgt(new PointCloudNormal);

	PointCloudPtr tgtCloud = targetCloud.getKeypoint();
	PointCloudPtr srcCloud = sourceCloud.getKeypoint();
	NormalsPtr tgtNormal = targetCloud.getkeypointsNormal();
	NormalsPtr srcNormal = sourceCloud.getkeypointsNormal();

	for (size_t i = 0; i < srcCloud->points.size(); ++i)
	{
		PointNormal point_normal;
		point_normal.x = srcCloud->points[i].x;
		point_normal.y = srcCloud->points[i].y;
		point_normal.z = srcCloud->points[i].z;

		point_normal.normal_x = srcNormal->points[i].normal_x;
		point_normal.normal_y = srcNormal->points[i].normal_y;
		point_normal.normal_z = srcNormal->points[i].normal_z;

		pointNormal_src->push_back(point_normal);
	}
	sourceCloud.setPointCloudNormal(pointNormal_src);

	for (size_t i = 0; i < tgtCloud->points.size(); ++i)
	{
		PointNormal point_normal;
		point_normal.x = tgtCloud->points[i].x;
		point_normal.y = tgtCloud->points[i].y;
		point_normal.z = tgtCloud->points[i].z;

		point_normal.normal_x = tgtNormal->points[i].normal_x;
		point_normal.normal_y = tgtNormal->points[i].normal_y;
		point_normal.normal_z = tgtNormal->points[i].normal_z;

		pointNormal_tgt->push_back(point_normal);
	}
	targetCloud.setPointCloudNormal(pointNormal_tgt);
}

//NARF关键点
void keyPoints_NARF(const PointCloudPtr cloud_src,
	const PointCloudPtr keyPoints_NARF,
	float angular_resolution ,
	float support_size)
{
	PointCloud& point_cloud = *cloud_src;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;//带视角的点构成的点云
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());//仿射变换

	angular_resolution = pcl::deg2rad(angular_resolution);
	scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
															point_cloud.sensor_origin_[1],
															point_cloud.sensor_origin_[2])) *
										Eigen::Affine3f(point_cloud.sensor_orientation_);//设置传感器的姿势
	// -----------------------------------------------
	// -----Create RangeImage from the PointCloud-----
	// -----------------------------------------------
	float noise_level = 0.0f;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	range_image.createFromPointCloud(point_cloud, angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),
		scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	range_image.integrateFarRanges(far_ranges);

	// --------------------------------
	// -----Extract NARF keypoints-----
	// --------------------------------
	pcl::RangeImageBorderExtractor range_image_border_extractor;
	pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
	narf_keypoint_detector.setRangeImage(&range_image);
	narf_keypoint_detector.getParameters().support_size = support_size;
	//narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
	//narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

	pcl::PointCloud<int> keypoint_indices;
	narf_keypoint_detector.compute(keypoint_indices);

	PointCloud& keypoints = *keyPoints_NARF;
	keypoints.points.resize(keypoint_indices.points.size());
	for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
	{
		keypoints.points[i].getVector3fMap() = range_image.points[keypoint_indices.points[i]].getVector3fMap();
		// std::cout << keypoint_indices.points[i] << ", "<< keypoints.points[i].index << std::endl;
	}
	std::cout << std::endl << keypoint_indices.points.size() << " NARF Key Points Are Detected." << std::endl ;
}


//Harris关键点
void keyPoints_HARRIS( PointCloudPtr cloud_src, 
		 PointCloudPtr keyPoints_Harris,
	float radius,
	float threshold)
{
	clock_t start, end;
	start = clock();

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;

	harris.setInputCloud(cloud_src);

	// Filter only
	harris.setRadius(0.5);//0.5
	//harris.setThreshold(threshold);//0.02
	harris.setNonMaxSupression(true);
	harris.setRefine(true);

	//cloud_out->height = 1;
	//cloud_out->width = 100;
	//cloud_out->resize(cloud_out->height*cloud_src->width);
	//cloud_out->clear();

	harris.compute(*cloud_out);
	int size = cloud_out->size();

	//keyPoints_Harris->height = 1;
	//keyPoints_Harris->width = 100;
	//keyPoints_Harris->resize(cloud_out->height*cloud_src->width);
	//keyPoints_Harris->clear();

	pcl::PointXYZ point;
	for (int i = 0; i<size; i++)
	{
		point.x = cloud_out->at(i).x;
		point.y = cloud_out->at(i).y;
		point.z = cloud_out->at(i).z;
		keyPoints_Harris->push_back(point);
	}
    std::cout << std::endl << keyPoints_Harris->size() << " HARRIS Key Points Are Detected." << std::endl;
	end = clock();
	std::cout << "keyPoints_HARRIS has finished in "
		<< (end - start) / CLOCKS_PER_SEC << " s \n";
}


//SHOT计算
void computeFeatures_SHOT(FeatureCloud &cloud, float R)
{
	clock_t start, end;
	start = clock();

	SHOT_features::Ptr shot_features_(new SHOT_features);
	SHOT_features::Ptr nanremoved_(new SHOT_features);

	pcl::SHOTEstimationOMP<PointT, NormalT, SHOT352_FeatureT> shot_est;
	shot_est.setNumberOfThreads(4);
	shot_est.setSearchSurface(cloud.getCloud());
	shot_est.setInputNormals(cloud.getNormal());
	shot_est.setInputCloud(cloud.getKeypoint());

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	shot_est.setSearchMethod(tree);
	shot_est.setRadiusSearch(R); //0.5

	shot_est.compute(*shot_features_);

	removeNANfromSHOT(shot_features_, nanremoved_, cloud);

	//cloud.setFeatures_SHOT(shot_features_);
	cloud.setKeypoints_SHOT(nanremoved_);
	end = clock();
	std::cout << "computeFeatures_SHOT has finished in "
		<< (end - start) / CLOCKS_PER_SEC << " s \n";
}
//FPFH计算
void computeFeatures_FPFH(FeatureCloud &cloud,
							float R)
{
	clock_t start, end;
	start = clock();

	FPFH_features::Ptr fpfh_features_(new FPFH_features);
	FPFH_features::Ptr nanremoved_(new FPFH_features);

	pcl::FPFHEstimationOMP<PointT, NormalT, FPFH33_FeatureT> fpfh;
	fpfh.setNumberOfThreads(4);
	fpfh.setSearchSurface(cloud.getCloud());
	fpfh.setInputCloud(cloud.getKeypoint());
	fpfh.setInputNormals(cloud.getNormal());
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(R);

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	// Compute the features
	fpfh.compute(*fpfh_features_);
	removeNANfromFPFH(fpfh_features_, nanremoved_, cloud);

	//cloud.setFeatures_FPFH(fpfh_features_);
	cloud.setKeypoints_FPFH(nanremoved_);
	end = clock();
	std::cout << "computeFeatures_FPFH has finished in "
		<< (end - start) / CLOCKS_PER_SEC << " s \n";
}
//局部参考坐标系LRF
void computeFeatures_LRF(FeatureCloud &cloud,
						float rf_radius)
{
	clock_t start, end;
	start = clock();

	LRF_features::Ptr RFoutput(new LRF_features);
	pcl::BOARDLocalReferenceFrameEstimation<PointT, NormalT, LRFType> rf_est;
	rf_est.setFindHoles(true);//指代孔洞区域是否估计吧？
	rf_est.setRadiusSearch(rf_radius);

	rf_est.setInputCloud(cloud.getKeypoint());
	rf_est.setInputNormals(cloud.getNormal());
	rf_est.setSearchSurface(cloud.getCloud());
	rf_est.compute(*RFoutput);

	cloud.setKeypoints_LRF(RFoutput);

	end = clock();
	std::cout << "computeFeatures_LRF has finished in "
		<<(end - start)/CLOCKS_PER_SEC<<" s \n";
}
//VFH全局特征描述子
void computeFeatures_VFH(FeatureCloud &cloud, VFH_method method)
{
	VFH_features::Ptr vfhOutput(new VFH_features);

	if (method != VFH_method::VFH && method != VFH_method::CVFH)
	{
		std::cout << "Error: the method input is wrong!\n";
		cloud.setFeatures_VFH(vfhOutput);
		return;
	}

	clock_t start, end;
	start = clock();

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	if (method == VFH_method::VFH) {

		// Create the VFH estimation class, and pass the input dataset+normals to it
		pcl::VFHEstimation< PointT, NormalT, VFH308_FeatureT> vfh;
		vfh.setInputCloud(cloud.getKeypoint());
		vfh.setInputNormals(cloud.getkeypointsNormal());
		// alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);
		vfh.setSearchMethod(tree);
		// Compute the features
		vfh.compute(*vfhOutput);
	}

	if (method == VFH_method::CVFH) {

		pcl::CVFHEstimation<PointT, NormalT, VFH308_FeatureT> cvfh;
		cvfh.setInputCloud(cloud.getKeypoint());
		cvfh.setInputNormals(cloud.getkeypointsNormal());
		cvfh.setSearchMethod(tree);
		cvfh.compute(*vfhOutput);
	}

	cloud.setkeypoints_VFH(vfhOutput);
	if (!pcl_isfinite(vfhOutput->at(0).histogram[0]))
	{
		std::cout << "Error: computeFeatures_VFH has failed !\n";
	}
	else {
		//std::cout << "Features_VFH size : " << vfhOutput->at(0) << std::endl;
	}
	
	end = clock();
	std::cout << "computeFeatures_VFH has finished in "
		<< (end - start) / CLOCKS_PER_SEC << " s \n";
}
//kdTree搜索最近点
void getNearestIndices(const PointCloudPtr cloudIn,
	const PointCloudPtr cloudQuery,
	PointCloudPtr cloudResult,
	pcl::PointIndices::Ptr indicesPtr)
{
	pcl::search::KdTree<PointT> kdtree;
	kdtree.setInputCloud(cloudIn);
	std::vector<float> pointNKNSquareDistance(1);
	std::vector<int> pointIdxNKNSearch(1);

	int indice = 0;
	for (size_t i = 0; i < cloudQuery->size(); ++i)
	{
		int num = kdtree.nearestKSearch(cloudQuery->points[i], 1, pointIdxNKNSearch, pointNKNSquareDistance);
		if (num == 1) {

			indice = pointIdxNKNSearch[0];
			indicesPtr->indices.push_back(indice);
			cloudResult->points.push_back(cloudIn->points[indice]);
		}
	}
}
//kdTree搜索最近点(SHOT)
void getNearestIndices_SHOT(const SHOT_features::Ptr cloudIn,
	const SHOT_features::Ptr cloudQuery,
	pcl::CorrespondencesPtr corrPtr)
{
	pcl::search::KdTree<SHOT352_FeatureT> kdtree;
	kdtree.setInputCloud(cloudIn);
	std::vector<float> pointNKNSquareDistance(1);
	std::vector<int> pointIdxNKNSearch(1);

	for (size_t i = 0; i < cloudQuery->points.size(); ++i)
	{
		if (!pcl_isfinite(cloudQuery->at(i).descriptor[0]))
		{
			continue;
		}
		int num = kdtree.nearestKSearch(cloudQuery->points[i], 1, pointIdxNKNSearch, pointNKNSquareDistance);
		if (num == 1 && pointNKNSquareDistance[0] < 0.25f )//距离的值在0~1
		{
			pcl::Correspondence corr;
			corr.index_query = static_cast<int>(i);
			corr.index_match = pointIdxNKNSearch[0];
			corr.distance = pointNKNSquareDistance[0];
			corrPtr->push_back(corr);
		}
	}

}
//对应点对估计
void correspondence_estimation(std::string kernel_descriptors,
	FPFH_features::Ptr source_cloud,
	FPFH_features::Ptr target_cloud,
	pcl::Correspondences &all_corres)
{
	if(kernel_descriptors == "FPFH"){

		pcl::registration::CorrespondenceEstimation<FPFH33_FeatureT, FPFH33_FeatureT> est;
		est.setInputSource(source_cloud);
		est.setInputTarget(target_cloud);
		est.determineReciprocalCorrespondences(all_corres);
	}


}
//对应点对剔除
void correspondences_rejection(FeatureCloud &source_cloud,
	FeatureCloud &target_cloud,
	pcl::Correspondences &correspondences,
	pcl::Correspondences &inliers,
	int MaximumIterations, float Inlierthreshold)
{
	clock_t start, end;
	start = clock();

	pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
	sac.setInputSource(source_cloud.getKeypoint());
	sac.setInputTarget(target_cloud.getKeypoint());

	// sac.setInputSource(source_cloud.getCloud ());
	// sac.setInputTarget(target_cloud.getCloud ());

	// Set the threshold for rejection iteration
	sac.setInlierThreshold(Inlierthreshold);
	sac.setMaximumIterations(MaximumIterations);
	sac.getRemainingCorrespondences(correspondences, inliers);
	end = clock();
	std::cout << "CorrespondenceRejectorSampleConsensus computation time : "
		<< float(end - start) / CLOCKS_PER_SEC << "s" << std::endl;
}
//对应点对剔除（ 自定义约束）
void advancedMatching(PointCloudPtr target, PointCloudPtr source,
	pcl::Correspondences &correspondences,
	pcl::Correspondences &inliers,
	float tupleScale,
	int tuple_max_cnt_)
{
	clock_t start, end;
	start = clock();
	srand(time(NULL));
	printf("[tuple constraint] ");

	int rand0, rand1, rand2;
	int idi0, idi1, idi2;
	int idj0, idj1, idj2;
	float scale = tupleScale;
	int ncorr = correspondences.size();
	int number_of_trial = ncorr * 100;

	pcl::Correspondence corr0;
	pcl::Correspondence corr1;
	pcl::Correspondence corr2;

	int cnt = 0;
	int i = 0;
	for (i = 0; i < number_of_trial; ++i)
	{
		rand0 = rand() % ncorr;
		rand1 = rand() % ncorr;
		rand2 = rand() % ncorr;

		idi0 = correspondences[rand0].index_match;
		idj0 = correspondences[rand0].index_query;
		corr0.index_match = idi0;
		corr0.index_query = idj0;
		corr0.distance = correspondences[rand0].distance;

		idi1 = correspondences[rand1].index_match;
		idj1 = correspondences[rand1].index_query;
		corr1.index_match = idi1;
		corr1.index_query = idj1;
		corr1.distance = correspondences[rand1].distance;

		idi2 = correspondences[rand2].index_match;
		idj2 = correspondences[rand2].index_query;
		corr2.index_match = idi2;
		corr2.index_query = idj2;
		corr2.distance = correspondences[rand2].distance;

		// collect 3 points from i-th fragment
		Eigen::Vector3f pti0 = target->points[idi0].getVector3fMap();
		Eigen::Vector3f pti1 = target->points[idi1].getVector3fMap();
		Eigen::Vector3f pti2 = target->points[idi2].getVector3fMap();

		float li0 = (pti0 - pti1).norm();
		float li1 = (pti1 - pti2).norm();
		float li2 = (pti2 - pti0).norm();

		// collect 3 points from j-th fragment
		Eigen::Vector3f ptj0 = source->points[idj0].getVector3fMap();
		Eigen::Vector3f ptj1 = source->points[idj1].getVector3fMap();
		Eigen::Vector3f ptj2 = source->points[idj2].getVector3fMap();

		float lj0 = (ptj0 - ptj1).norm();
		float lj1 = (ptj1 - ptj2).norm();
		float lj2 = (ptj2 - ptj0).norm();

		if ((li0 * scale < lj0) && (lj0 < li0 / scale) &&
			(li1 * scale < lj1) && (lj1 < li1 / scale) &&
			(li2 * scale < lj2) && (lj2 < li2 / scale))
		{
			inliers.push_back(corr0);
			inliers.push_back(corr1);
			inliers.push_back(corr2);
			cnt++;
		}

		if (cnt >= tuple_max_cnt_)
			break;
	}
	end = clock();
	std::cout<<"advancedMatching ---Tuple computation time : "
		<< float(end - start) / CLOCKS_PER_SEC <<"s"<< std::endl;
}
//提取或去除索引的点云
void extractIndicesPoints(PointCloudPtr pointInput,
	PointCloudPtr pointOutput,
	pcl::PointIndices::Ptr inliers,
	bool extractNegative)
{
	clock_t start, end;
	start = clock();

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(pointInput);
	extract.setIndices(inliers);
	//true 为提取索引， false 为剔除索引
	extract.setNegative(extractNegative);
	extract.filter(*pointOutput);

	if (!extractNegative)
		std::cout << "Extract the rest-component: "
		<< pointOutput->points.size() << std::endl;
	else
		std::cout << "Extract the indice-component: "
		<< pointOutput->points.size() << std::endl;

	end = clock();
	std::cout << "extractIndicesPoints has finished in "
		<< (float)(end - start) / CLOCKS_PER_SEC << " s \n";
}
// 基于RANSAC的形状提取
bool SACSegmentation_model(PointCloudPtr pointInput,
	pcl::ModelCoefficients::Ptr coefficients,
	pcl::PointIndices::Ptr inliers,
	pcl::SacModel modeltype,
	int maxIteration ,
	float distancethreshold)
{
	clock_t start, end;
	start = clock();
	//Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(modeltype);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(maxIteration);
	seg.setDistanceThreshold(distancethreshold);
	
	seg.setInputCloud(pointInput);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a model for the given dataset.");
		return false;
	}
	end = clock();
	std::cout << "SACSegmentation_model has finished in " 
		<< float(end - start) / CLOCKS_PER_SEC << " s" << std::endl;
}
//基于SAC的垂直平面提取
bool SACSegmentation_plane(PointCloudPtr pointInput,
	pcl::ModelCoefficients::Ptr coefficients,
	pcl::PointIndices::Ptr inliers,
	int maxIteration ,
	float distancethreshold )
{
	double degree = 0.0;
	pcl::SampleConsensusModelPerpendicularPlane<PointT>::Ptr sac_plane(new pcl::SampleConsensusModelPerpendicularPlane<PointT>(pointInput));
	sac_plane->setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));//设置所搜索平面垂直的轴
	sac_plane->setEpsAngle(pcl::deg2rad(degree));
	
	pcl::RandomSampleConsensus<PointT> ransac(sac_plane);
	ransac.setMaxIterations(maxIteration);
	ransac.setDistanceThreshold(distancethreshold);
	ransac.computeModel();
	ransac.getInliers(inliers->indices);

	Eigen::VectorXf coefficient_;
	ransac.getModelCoefficients(coefficient_);

	for (size_t i = 0; i < coefficient_.size(); ++i)
	{
		coefficients->values.push_back(coefficient_[i]);
	}

}
// 最小包围盒
void OrientedBoundingBox(PointCloudPtr pointInput,
	Eigen::Vector3f &whd,
	Eigen::Vector3f &bboxT,
	Eigen::Quaternionf &bboxQ,
	float scalar,
	PointT& pcX,PointT& pcY, PointT& pcZ, PointT& initialoriginalPoint)
{
	if (pointInput == nullptr || pointInput->points.empty())
	{
		std::cout << "\nError: OrientedBoundingBox --- the pointInput is empty!\n";
		return;
	}

	PointCloudPtr cloudInput(new PointCloud);
	pcl::copyPointCloud(*pointInput, *cloudInput);
	//重心
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloudInput, pcaCentroid);
	//协方差矩阵
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloudInput, pcaCentroid, covariance);
	//特征向量
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	//校正主方向间垂直
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

	// 另一种计算点云协方差矩阵特征值和特征向量的方式:通过pcl中的pca接口，如下，这种情况得到的特征向量相似特征向量
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloudSegmented);
	pca.project(*cloudSegmented, *cloudPCAprojection);
	std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;//计算特征向量
	std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;//计算特征值
	*/
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();//R
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head(3));//-R*T
	tm_inv = tm.inverse();

	PointCloudPtr transformedCloud(new PointCloud);
	pcl::transformPointCloud(*cloudInput, *transformedCloud, tm);

	PointT min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());//两点的对称中心,形心

	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);

	Eigen::Vector3f whd1;
	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	whd = whd1;
    scalar = (whd1(0) + whd1(1) + whd1(2)) / 3;//点云平均尺度

	const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
	const Eigen::Vector3f bboxT1(c1);

	bboxQ = tm_inv.block<3, 3>(0, 0);
	bboxT = c;

	//变换到原点的点云主方向
	PointT originalPoint;
	originalPoint.x = 0.0;
	originalPoint.y = 0.0;
	originalPoint.z = 0.0;
	Eigen::Vector3f px, py, pz;
	Eigen::Affine3f tm_aff(tm);
	pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);

	PointT pcaX, pcaY, pcaZ;
	pcaX.x = scalar * px(0);
	pcaX.y = scalar * px(1);
	pcaX.z = scalar * px(2);

	pcaY.x = scalar * py(0);
	pcaY.y = scalar * py(1);
	pcaY.z = scalar * py(2);

	pcaZ.x = scalar * pz(0);
	pcaZ.y = scalar * pz(1);
	pcaZ.z = scalar * pz(2);

	//初始点云的主方向
	initialoriginalPoint.x = pcaCentroid(0);
	initialoriginalPoint.y = pcaCentroid(1);
	initialoriginalPoint.z = pcaCentroid(2);

	pcX.x = scalar * eigenVectorsPCA(0, 0) + initialoriginalPoint.x;
	pcX.y = scalar * eigenVectorsPCA(1, 0) + initialoriginalPoint.y;
	pcX.z = scalar * eigenVectorsPCA(2, 0) + initialoriginalPoint.z;

	pcY.x = scalar * eigenVectorsPCA(0, 1) + initialoriginalPoint.x;
	pcY.y = scalar * eigenVectorsPCA(1, 1) + initialoriginalPoint.y;
	pcY.z = scalar * eigenVectorsPCA(2, 1) + initialoriginalPoint.z;

	pcZ.x = scalar * eigenVectorsPCA(0, 2) + initialoriginalPoint.x;
	pcZ.y = scalar * eigenVectorsPCA(1, 2) + initialoriginalPoint.y;
	pcZ.z = scalar * eigenVectorsPCA(2, 2) + initialoriginalPoint.z;

}
//基于欧氏距离的聚类
void EuclideanClusterExtraction(PointCloudPtr pointInput,
	std::vector<PointCloudPtr>& cloudListOutput,
	float clusterTolerance ,
	int minClusterSize ,
	int maxClusterSize )
{
	if (pointInput == nullptr || pointInput->points.empty())
	{
		std::cout << "Error: pointInput is empty!\n";
		return;
	}

	clock_t start, end;
	start = clock();
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	std::vector<pcl::PointIndices> cluster_indices;

	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setInputCloud(pointInput);
	ec.setSearchMethod(tree);
	ec.setClusterTolerance(clusterTolerance);
	ec.setMinClusterSize(minClusterSize);
	ec.setMaxClusterSize(maxClusterSize);

	ec.extract(cluster_indices);

	int num = 1;
	std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
	for (; it != cluster_indices.end(); ++it)
	{
		PointCloudPtr cloud(new PointCloud);
		pcl::PointIndices::Ptr indice(new pcl::PointIndices);
		*indice = *it;
		extractIndicesPoints(pointInput, cloud, indice, false);

		cloudListOutput.push_back(cloud);
		std::cout << "PointCloud representing the Cluster: " << num << " , "
			<< cloud->points.size() << " data points." << std::endl;
		num++;
	}

	end = clock();
	std::cout << "EuclideanClusterExtraction has finished in "
		<< (float)(end - start) / CLOCKS_PER_SEC <<" s "<< std::endl;
}
//基于区域生长的聚类
void RegionGrowingClusterExtraction(PointCloudPtr pointInput,
	std::vector<PointCloudPtr>& cloudListOutput,
	SurfaceNormals::Ptr normalInput,
	int minClusterSize,
	int maxClusterSize,
	int numofNeighbour ,
	float smoothThreshold ,
	float curvatureThreshold)
{
	clock_t start, end;
	start = clock();
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	pcl::RegionGrowing<PointT, NormalT> reg;
	reg.setMinClusterSize(minClusterSize);
	reg.setMaxClusterSize(maxClusterSize);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(numofNeighbour);
	reg.setInputCloud(pointInput);
	reg.setInputNormals(normalInput);
	reg.setSmoothnessThreshold(smoothThreshold);

	std::vector<pcl::PointIndices> cluster_indices;
	reg.extract(cluster_indices);
	std::cout << "Number of clusters is equal to " << cluster_indices.size() << std::endl;

	int j = 1;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		PointCloudPtr cloud_cluster(new PointCloud);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cloud_cluster->points.push_back(pointInput->points[*pit]);
		}
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << j << " , " << cloud_cluster->points.size() << " data points." << std::endl;
		j++;
		cloudListOutput.push_back(cloud_cluster);

	}
	end = clock();
	std::cout << "RegionGrowingClusterExtraction has finished in "
		<< float(end - start) / CLOCKS_PER_SEC << " s \n";
}
//ICP
float iterative_closest_points(std::string solver,
	bool flag_reciprocal, bool flag_ransac,
	FeatureCloud &source_cloud, FeatureCloud &target_cloud,
	float transEps, float corresDist, float EuclFitEps, float outlThresh, int maxIteration,
	Eigen::Matrix4f &final_transformation)
{
	std::cout << "----------------- ICP -----------" << std::endl;
	PointCloudNormal Final;

	PointCloudNormal::Ptr pointNormal_src = source_cloud.getPointCloudNormal();
	PointCloudNormal::Ptr pointNormal_tgt = target_cloud.getPointCloudNormal();

	pcl::registration::CorrespondenceRejector::Ptr ransac_rej \
		(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointNormal>());

	if (solver == "SVD")
	{
		std::cout << "SVD Solver for ICP Is Running!" << std::endl;
		pcl::IterativeClosestPoint<PointNormal, PointNormal> icp;

		if (flag_reciprocal == true)
			icp.setUseReciprocalCorrespondences(true);
		if (flag_ransac == true)
		{
			icp.setRANSACOutlierRejectionThreshold(outlThresh);
			// add ransac rejector
			icp.addCorrespondenceRejector(ransac_rej);
		}

		icp.setInputSource(pointNormal_src);
		icp.setInputTarget(pointNormal_tgt);

		icp.setMaximumIterations(maxIteration);
		icp.setTransformationEpsilon(transEps);
		//icp.setMaxCorrespondenceDistance(corresDist);
		//icp.setEuclideanFitnessEpsilon(EuclFitEps);
		icp.setRANSACOutlierRejectionThreshold(0.01);

		icp.align(Final, final_transformation);

		final_transformation = icp.getFinalTransformation();
		std::cout<<"SVD Solver for ICP, FitnessScore: "<< icp.getFitnessScore()<<std::endl;
		std::cout << std::endl << final_transformation << std::endl;

		return icp.getFitnessScore();
	}
	if (solver == "LM")
	{
		std::cout << "LM Solver for ICP Is Running!" << std::endl;
		pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> icp_lm;

		if (flag_reciprocal == true)
			icp_lm.setUseReciprocalCorrespondences(true);
		if (flag_ransac == true)
		{
			icp_lm.setRANSACOutlierRejectionThreshold(outlThresh);
			// add ransac rejector
			icp_lm.addCorrespondenceRejector(ransac_rej);
		}

		icp_lm.setInputSource(pointNormal_src);
		icp_lm.setInputTarget(pointNormal_tgt);

		icp_lm.setMaximumIterations(maxIteration);
		icp_lm.setTransformationEpsilon(transEps);
		//icp_lm.setMaxCorrespondenceDistance(corresDist);
		//icp_lm.setEuclideanFitnessEpsilon(EuclFitEps);

		icp_lm.align(Final, final_transformation);

		final_transformation = icp_lm.getFinalTransformation();
		std::cout << "LM Solver for ICP , FitnessScore: " << icp_lm.getFitnessScore() << std::endl;
		std::cout << std::endl << final_transformation << std::endl;

		return icp_lm.getFitnessScore();
	}
}
//SAC-IA
void SAC_IA_Transform(FeatureCloud &source_cloud,
	FeatureCloud &target_cloud,
	float minsampleDistance,
	int numofSample,
	int correspondenceRandomness,
	Eigen::Matrix4f& final_transformation)
{
	std::cout << "--------------- SAC-IA ------------------" << std::endl;
	clock_t start;
	clock_t end;
	start = clock();
	//SAC配准
	pcl::SampleConsensusInitialAlignment<PointT, PointT, FPFH33_FeatureT> scia;
	scia.setInputSource(source_cloud.getKeypoint());
	scia.setInputTarget(target_cloud.getKeypoint());
	scia.setSourceFeatures(source_cloud.getkeypointsFeatures_FPFH());
	scia.setTargetFeatures(target_cloud.getkeypointsFeatures_FPFH());
	
	//scia.setMaxCorrespondenceDistance();
	scia.setMinSampleDistance(minsampleDistance);
	scia.setNumberOfSamples(numofSample);//设置每次迭代计算中使用的样本数量（可省）,可节省时间
	scia.setCorrespondenceRandomness(correspondenceRandomness);//设置计算协方差时选择多少近邻点，该值越大，
															   //协方差越精确，但是计算效率越低.(可省)
	
	PointCloudPtr result(new PointCloud);
	scia.align(*result, final_transformation);
	end = clock();
	std::cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC <<"s"<< endl;
	std::cout << "SAC has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	std::cout << std::endl << scia.getFinalTransformation() << std::endl;
	final_transformation = scia.getFinalTransformation();
}
//SACPrerejective变换
void SAC_Transform(PointCloudNormal::Ptr target,
	PointCloudNormal::Ptr source,
	FPFH_features::Ptr fpfh_tgt,
	FPFH_features::Ptr fpfh_src,
	int maxIteration,
	float maxCorrDist,
	float inlierFration,
	Eigen::Matrix4f& final_transformation)
{
	pcl::SampleConsensusPrerejective<PointNormal, PointNormal, FPFH33_FeatureT> align;
	align.setInputSource(source);
	align.setSourceFeatures(fpfh_src);
	align.setInputTarget(target);
	align.setTargetFeatures(fpfh_tgt);
	align.setMaximumIterations(maxIteration); //  采样一致性迭代次数
	align.setNumberOfSamples(3); //  创建假设所需的样本数
	align.setCorrespondenceRandomness(5); //  使用的临近特征点的数目
	align.setSimilarityThreshold(0.9f); // 多边形边长度相似度阈值
	align.setMaxCorrespondenceDistance(maxCorrDist); //  判断是否为内点的距离阈值
	align.setInlierFraction(inlierFration); //接受位姿假设所需的内点比例

	PointCloudNormal::Ptr result(new PointCloudNormal);
	align.align(*result);
	final_transformation = align.getFinalTransformation();
	
}
//SVD求解
void SVD_Transform(PointCloudPtr target,
	PointCloudPtr source,
	Eigen::Matrix4f &transformation_SVD, 
	int numOfIteration,
	float threshold)
{
	std::cout << "---------------SVD-------------\n";
	clock_t start, end;
	start = clock();

	float error = 0.0;
	PointCloudPtr tempTarget(new PointCloud);
	PointCloudPtr tempSource(new PointCloud);
	pcl::copyPointCloud(*target, *tempTarget);
	pcl::copyPointCloud(*source, *tempSource);

	Eigen::Vector3f target_mean, source_mean;
	target_mean.setZero();
	source_mean.setZero();
	remove_Mean(tempSource, source_mean);
	remove_Mean(tempTarget, target_mean);

	int pointSize = target->size();
	Eigen::Matrix3f BaseH = Eigen::Matrix3f::Zero();
	for (size_t i = 0; i < pointSize; ++i)
	{
		BaseH += (tempSource->points[i].getVector3fMap())*
			(tempTarget->points[i].getVector3fMap().transpose());
	}
	BaseH /= pointSize;
	Eigen::JacobiSVD<Eigen::Matrix3f> svd(BaseH, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3f U = svd.matrixU();
	Eigen::Matrix3f V = svd.matrixV();
	Eigen::Matrix3f D = Eigen::Matrix3f::Identity();
	if (U.determinant() * V.determinant() < 0)
	{
		D(2, 2) = -1;
	}
	Eigen::Matrix3f R = U * D * V.transpose();
	Eigen::Matrix4f trans = Eigen::Matrix4f::Zero();
	trans.topLeftCorner(3, 3) = R;
	trans.block<3, 1>(0, 3) = target_mean - R * source_mean;
	trans(3, 3) = 1;

	pcl::transformPointCloud(*tempSource, *tempSource, trans);
		
	float tempError = 0.0;
	for (size_t i = 0; i < pointSize; ++i)
	{
		Eigen::Vector3f q(tempTarget->points[i].x, tempTarget->points[i].y, tempTarget->points[i].z);
		Eigen::Vector3f p(tempSource->points[i].x, tempSource->points[i].y, tempSource->points[i].z);

		tempError += (q - p).squaredNorm();
	}
	tempError /= pointSize;
	error = tempError;
	std::cout << "SVD Error : " << tempError << std::endl;
	transformation_SVD = trans * transformation_SVD;

	end = clock();
	std::cout << "SVD computation time is: " << float(end - start) / CLOCKS_PER_SEC << "s" << endl;
	std::cout << "SVD Transformation transformation_SVD : \n" << transformation_SVD << std::endl;
}
//FGR变换
void FGR_Transform(PointCloudPtr target,
	PointCloudPtr source,
	Eigen::Matrix4f &transformation_SVD,
	int numOfIteration,
	float threshold)
{

}
//计算转换矩阵
bool RotationTranslationCompute(FeatureCloud& cloudtarget,
	FeatureCloud& cloudsource,
	Eigen::Matrix4f &tranResult)
{
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloudtarget.getCloud(), *cloudtarget.getCloud(), mapping);
	pcl::removeNaNFromPointCloud(*cloudsource.getCloud(), *cloudsource.getCloud(), mapping);
	//Resolution Calculation
	float resolution = 0.0;
    resolution = computeResolution(cloudtarget.getCloud());
	//Statistical filter
	StatisticalOutlierRemoval_Filter(cloudtarget.getCloud(), cloudtarget.getCloud(), 30, 1.0);
	StatisticalOutlierRemoval_Filter(cloudsource.getCloud(), cloudsource.getCloud(), 30, 1.0);
	
	//降采样获取关键点
	if (true)
	{
		float leafSize = resolution * 5;
		VoxelGrid_Filter(cloudtarget.getCloud(), cloudtarget.getCloud(), leafSize);
		VoxelGrid_Filter(cloudsource.getCloud(), cloudsource.getCloud(), leafSize);
	}
	//归一化
	if (false)
	{
		Eigen::Vector3f mean_target = Eigen::Vector3f::Zero();
		Eigen::Vector3f mean_source = Eigen::Vector3f::Zero();
		float gloablScale_target = 0.0;
		float gloablScale_source = 0.0;
		normalizePoints(cloudtarget.getCloud(), mean_target, gloablScale_target);
		normalizePoints(cloudsource.getCloud(), mean_source, gloablScale_source);
	}

	resolution = computeResolution(cloudtarget.getCloud());//更新分辨率

	//Normal Calculation
	int normal_K = 50;
	float normal_R = resolution * 5;
	computeSurfaceNormals(cloudtarget, normal_K, normal_R);
	computeSurfaceNormals(cloudsource, normal_K, normal_R);

	PointCloudPtr cloudtargetPtr = cloudtarget.getCloud();
	PointCloudPtr cloudsourcePtr = cloudsource.getCloud();
	PointCloudPtr targetKeyPoints(new PointCloud);
	PointCloudPtr sourceKeyPoints(new PointCloud);
	PointCloudPtr targetKeyPoints_cal(new PointCloud);
	PointCloudPtr sourceKeyPoints_cal(new PointCloud);
	pcl::PointIndices::Ptr targetIndices_ptr(new pcl::PointIndices);
	pcl::PointIndices::Ptr sourceIndices_ptr(new pcl::PointIndices);

	// Keypoints Detection关键点
	//float angularResolution = 20;
	//float supportSize = resolution * 2;//感兴趣点的尺寸（球面的直径）
	//keyPoints_NARF(cloudtargetPtr, targetKeyPoints_cal, angularResolution, supportSize);
	//keyPoints_NARF(cloudsourcePtr, sourceKeyPoints_cal, angularResolution, supportSize);

	//keyPoints_HARRIS(cloudtargetPtr, targetKeyPoints_cal);
	//keyPoints_HARRIS(cloudsourcePtr, sourceKeyPoints_cal);
	std::cout << endl;
	targetKeyPoints_cal = cloudtargetPtr;
	sourceKeyPoints_cal = cloudsourcePtr;
	if (targetKeyPoints_cal->points.empty() || sourceKeyPoints_cal->points.empty())
	{
		std::cout << "No Keypoints were detected ... \n";
		return false;
	}

	// Calculate the indices of keypoints
	getNearestIndices(cloudtargetPtr, targetKeyPoints_cal, 
		              targetKeyPoints, targetIndices_ptr);
	getNearestIndices(cloudsourcePtr, sourceKeyPoints_cal,
		              sourceKeyPoints, sourceIndices_ptr);

	cloudtarget.setKeypoints(targetKeyPoints);
	cloudsource.setKeypoints(sourceKeyPoints);
	cloudtarget.setKeyPoint_indices(targetIndices_ptr);
	cloudsource.setKeyPoint_indices(sourceIndices_ptr);

	SurfaceNormals::Ptr tgt_keypoints_normal(new SurfaceNormals);
	SurfaceNormals::Ptr src_keypoints_normal(new SurfaceNormals);
	for (size_t i = 0; i < targetIndices_ptr->indices.size(); ++i)
	{
		int index = targetIndices_ptr->indices[i];
		tgt_keypoints_normal->points.push_back(cloudtarget.getNormal()->points[index]);
	}
	for (size_t i = 0; i < sourceIndices_ptr->indices.size(); ++i)
	{
		int index = sourceIndices_ptr->indices[i];
		src_keypoints_normal->points.push_back(cloudsource.getNormal()->points[index]);
	}
	cloudtarget.setKeypoints_Normal(tgt_keypoints_normal);
	cloudsource.setKeypoints_Normal(src_keypoints_normal);

	//Feature describe
	float FPFH_radius = resolution * 5;
	computeFeatures_FPFH(cloudtarget, FPFH_radius);
	computeFeatures_FPFH(cloudsource, FPFH_radius);
	
	// Correspondence Estimation
	pcl::Correspondences all_correspondences;//剔除前
	pcl::Correspondences tuple_inliers;//tuple剔除后

	correspondence_estimation("FPFH", cloudsource.getkeypointsFeatures_FPFH(), 
							cloudtarget.getkeypointsFeatures_FPFH(),
							all_correspondences);
	std::cout << "correspondence_estimation size : " << all_correspondences.size() << std::endl;
	//约束去除错误点对
	float tupleScale = 0.93;
	int tuple_max_cnt_ = 1000;
	advancedMatching(cloudtarget.getKeypoint(), cloudsource.getKeypoint(),
		all_correspondences, tuple_inliers, tupleScale, tuple_max_cnt_);
	std::cout << "advancedMatching size : " << tuple_inliers.size() << std::endl;
		
	pcl::Correspondences tmpInliers;//剔除后
	tmpInliers.insert(tmpInliers.begin(), tuple_inliers.begin(), tuple_inliers.end());
	//基于SAC去除
	if (true)
	{
		correspondences_rejection(cloudsource, cloudtarget,
			tmpInliers, tmpInliers,
			35, resolution*3);//0 1 为 50， 

	}
	std::cout << "SAC  correspondences_rejection size : " << tmpInliers.size() << std::endl;

	PointCloudPtr temp_target(new PointCloud);
	PointCloudPtr temp_source(new PointCloud);
	pcl::copyPointCloud(*cloudtarget.getCloud(), *temp_target);
	pcl::copyPointCloud(*cloudsource.getCloud(), *temp_source);
	if (true)
	{
		showPointCloudCorrespondences("all_correspondences", temp_target,
			temp_source, all_correspondences, 10);
		showPointCloudCorrespondences("inliers", temp_target,
			temp_source, tmpInliers, 10);
	}

	if (tmpInliers.size() == 0)
	{
		std::cout << "Registration Error : inliers size : 0 " << std::endl;
		return false;
	}

	//根据匹配点对重新确立关键点
	FeatureCloud targetCloud_Keypoint, sourceCloud_Keypoint;
	PointCloudPtr targetKeypoint_(new PointCloud);
	PointCloudPtr sourceKeypoint_(new PointCloud);
	NormalsPtr targetKeypointNormal_(new SurfaceNormals);
	NormalsPtr sourceKeypointNormal_(new SurfaceNormals);
	FPFH_features::Ptr targetKeypointFPFH_(new FPFH_features);
	FPFH_features::Ptr sourceKeypointFPFH_(new FPFH_features);

	for (size_t i = 0; i < tmpInliers.size(); ++i)
	{
		PointT source = cloudsource.getKeypoint()->at(tmpInliers[i].index_query);
		PointT target = cloudtarget.getKeypoint()->at(tmpInliers[i].index_match);

		NormalT sourceNormal = cloudsource.getkeypointsNormal()->at(tmpInliers[i].index_query);
		NormalT targetNormal = cloudtarget.getkeypointsNormal()->at(tmpInliers[i].index_match);

		FPFH33_FeatureT sourceFPFH = cloudsource.getkeypointsFeatures_FPFH()->at(tmpInliers[i].index_query);
		FPFH33_FeatureT targetFPFH = cloudtarget.getkeypointsFeatures_FPFH()->at(tmpInliers[i].index_match);

		targetKeypoint_->points.push_back(target);
		sourceKeypoint_->points.push_back(source);

		targetKeypointNormal_->points.push_back(targetNormal);
		sourceKeypointNormal_->points.push_back(sourceNormal);

		targetKeypointFPFH_->points.push_back(targetFPFH);
		sourceKeypointFPFH_->points.push_back(sourceFPFH);
	}
	targetCloud_Keypoint.setKeypoints(targetKeypoint_);
	sourceCloud_Keypoint.setKeypoints(sourceKeypoint_);
	targetCloud_Keypoint.setKeypoints_Normal(targetKeypointNormal_);
	sourceCloud_Keypoint.setKeypoints_Normal(sourceKeypointNormal_);
	targetCloud_Keypoint.setKeypoints_FPFH(targetKeypointFPFH_);
	sourceCloud_Keypoint.setKeypoints_FPFH(sourceKeypointFPFH_);
	//Construct PointNormal建立法向量点云
	construct_PointNormal(cloudtarget, cloudsource);
	construct_PointNormal(targetCloud_Keypoint, sourceCloud_Keypoint);

	Eigen::Matrix4f tran = Eigen::Matrix4f::Identity();
	//SVD
	if (false) {

		float threshold = 0.01;
		int numIteration = 1;
		SVD_Transform(targetCloud_Keypoint.getKeypoint(),
			sourceCloud_Keypoint.getKeypoint(),
			tran, numIteration, threshold);
	}

	//SAC-IA
	if (true)
	{
		float minsampleDistance = resolution *2;
		int numofSample = tmpInliers.size() / 4;
		int correspondenceRandomness = 20;
		SAC_IA_Transform(sourceCloud_Keypoint, targetCloud_Keypoint, minsampleDistance,
			numofSample, correspondenceRandomness, tran);
	}

	//ICP
	if (true)
	{
		float transEps = 1e-10;//设置两次变化矩阵之间的差值（一般设置为1e-10即可）
		float maxCorresDist = resolution*0.8;//设置对应点对之间的最大距离（此值对配准结果影响较大）
		float EuclFitEps = 0.0001;//设置收敛条件是均方误差和小于阈值,停止迭代；
		float outlThresh = resolution*1.5;
		int maxIteration = 100;
		iterative_closest_points("SVD", false, false,
			sourceCloud_Keypoint, targetCloud_Keypoint,
			transEps, maxCorresDist, EuclFitEps,
			outlThresh, maxIteration, tran);
	}

		//score_list.push_back(scoreIdx);
		//tran_list.push_back(tran);
	//}
	//std::sort(score_list.begin(), score_list.end(), compareScore);
	//float score = score_list[0].first;
	//std::cout << "\n *************** The score result : " << score<<"  **************" << std::endl;
	//tranResult = tran_list[score_list[0].second];

	tranResult = tran;
	return true;
}
//基于Hough3DGrouping的聚类识别粗配准
void Hough3DGrouping_recognize(PointCloudPtr modelPointInput,
	PointCloudPtr scenePointInput,
	pcl::CorrespondencesPtr model_scene_corrs,
	pcl::PointCloud<LRFType>::Ptr modelRF,
	pcl::PointCloud<LRFType>::Ptr sceneRF,
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> & rototranslation,
	std::vector<pcl::Correspondences>& clustered_corr,
	float BinSize,
	float cg_thresh_ ,
	float rf_radius ,
	bool interpolation ,
	bool distanceWeight )
{
	clock_t start, end;
	start = clock();
	//clustering
	pcl::Hough3DGrouping<PointT, PointT, LRFType, LRFType> Hough3D_cluster;
	//Hough3D_cluster.setHoughBinSize(BinSize);
	//Hough3D_cluster.setHoughThreshold(cg_thresh_);
	Hough3D_cluster.setUseInterpolation(interpolation);//分数插值
	Hough3D_cluster.setUseDistanceWeight(distanceWeight);

	Hough3D_cluster.setInputCloud(scenePointInput);
	Hough3D_cluster.setInputRf(sceneRF);
	Hough3D_cluster.setSceneCloud(modelPointInput);
	Hough3D_cluster.setSceneRf(modelRF);
	Hough3D_cluster.setModelSceneCorrespondences(model_scene_corrs);

	Hough3D_cluster.recognize(rototranslation, clustered_corr);

	end = clock();
	std::cout << "Hough3DGrouping_recognize has finished in "
		<< (float)(end - start) / CLOCKS_PER_SEC << " s \n";
}

//基于几何一致性聚类
void GeometricConsistencyGrouping(float GCsize,
	float GCthreshold,
	PointCloudPtr modelPointInput,
	PointCloudPtr scenePointInput,
	pcl::CorrespondencesPtr model_scene_corrs,
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> & rototranslation,
	std::vector<pcl::Correspondences>& clustered_corr)
{
	clock_t start, end;
	start = clock();
	
	pcl::GeometricConsistencyGrouping<PointT, PointT> gc_clusterer;
	gc_clusterer.setGCSize(GCsize);
	gc_clusterer.setGCThreshold(GCthreshold);

	gc_clusterer.setInputCloud(modelPointInput);
	gc_clusterer.setSceneCloud(scenePointInput);
	gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

	gc_clusterer.recognize(rototranslation, clustered_corr);

	end = clock();
	std::cout << "Hough3DGrouping_recognize has finished in "
		<< (float)(end - start) / CLOCKS_PER_SEC << " s \n";
}
//显示点云对应点对
void showPointCloudCorrespondences(std::string viewerName,
								PointCloudPtr cloudTarget_,
								PointCloudPtr cloudSource_,
								pcl::Correspondences &corr_, int showThreshold)
{
	pcl::visualization::PCLVisualizer viewer(viewerName);
	viewer.setBackgroundColor(255, 255, 255);
	//viewer.initCameraParameters();

	//  We are translating the model so that it doesn't end in the middle of the scene representation
	Eigen::Matrix4f tran = Eigen::Matrix4f::Identity();
	tran(0, 3) = 500;
	PointCloudPtr tmpTarget(new PointCloud);
	pcl::copyPointCloud(*cloudTarget_, *tmpTarget);
	pcl::transformPointCloud(*tmpTarget, *tmpTarget, tran);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> color(tmpTarget, 0, 255, 0);
	viewer.addPointCloud(tmpTarget, color, "Target");
	viewer.addPointCloud(cloudSource_, color, "Source");

	for (size_t i = 0; i < corr_.size(); ++i)
	{
		if (i % showThreshold == 0)
		{
			PointT source = cloudSource_->at(corr_[i].index_query);
			PointT target = tmpTarget->at(corr_[i].index_match);
			char name[80] = "correspondece_line";
			sprintf(name, "_%d", i);
			viewer.addLine<PointT, PointT>(target, source, 0, 0, 255, name);
		}
	}

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(10);
	}
}
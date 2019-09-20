#pragma once
#include <limits>
#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>
#include <io.h>
#include <vector>
#include <sys/stat.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>

#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

//filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>   //滤波类头文件  （使用体素网格过滤器处理的效果比较好）
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>//提取滤波器
#include <pcl/filters/project_inliers.h>//投影滤波类头文件

// key points
#include <pcl/range_image/range_image.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
//SIFT keypoints
#include <pcl/keypoints/sift_keypoint.h>
//Harris keypoints
#include <pcl/keypoints/harris_3d.h>

#include <boost/thread/thread.hpp>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/features/vfh.h>
#include <pcl/features/cvfh.h>

// registration
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>//随机采样一致性去除
#include <pcl/registration/correspondence_rejection_features.h>//特征的错误对应关系去除
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>

#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd.h>

//segmentation
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>

//recognition
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/correspondence_grouping.h>
#include <pcl/recognition/hv/hv_go.h>

//visualizer
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

typedef pcl::Normal NormalT;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::Normal>::Ptr NormalsPtr;
typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormal;

typedef pcl::ReferenceFrame LRFType;
typedef pcl::FPFHSignature33 FPFH33_FeatureT;
typedef pcl::VFHSignature308 VFH308_FeatureT;
typedef pcl::SHOT352 SHOT352_FeatureT;
typedef pcl::PointCloud<FPFH33_FeatureT> FPFH_features;
typedef pcl::PointCloud<VFH308_FeatureT> VFH_features;
typedef pcl::PointCloud<SHOT352_FeatureT> SHOT_features;
typedef pcl::PointCloud<LRFType> LRF_features;

class FeatureCloud
{
public:
	FeatureCloud()
	{
		cloudPtr.reset(new PointCloud);
		keyPointsPtr.reset(new PointCloud);

		key_indices_.reset(new pcl::PointIndices);

		normalPtr.reset(new SurfaceNormals);
		FPFH_ptr.reset(new FPFH_features);
		SHOT_ptr.reset(new SHOT_features);
		VFH_ptr.reset(new VFH_features);
		LRF_ptr.reset(new LRF_features);

		keypointsNormalPtr.reset(new SurfaceNormals);
		keypointsFPFH_ptr.reset(new FPFH_features);
		keypointsSHOT_ptr.reset(new SHOT_features);
		keypointsVFH_ptr.reset(new VFH_features);
		keypointsLRF_ptr.reset(new LRF_features);
	}

	~FeatureCloud(){}
	//set point
	void setInputCloud(PointCloudPtr cloud)
	{
		cloudPtr = cloud;
	}
	//set normal
	void setInputNormal(SurfaceNormals::Ptr normal)
	{
		normalPtr = normal;
	}
	//set keypoint
	void setKeypoints(PointCloudPtr keypoint)
	{
		keyPointsPtr = keypoint;
	}
	//set keypoint indices
	void setKeyPoint_indices(pcl::PointIndices::Ptr indices)
	{
		key_indices_ = indices;
	}
	// Set keypoints normal
	void setKeypoints_Normal(SurfaceNormals::Ptr normal)
	{
		keypointsNormalPtr = normal;
	}
	// Set SHOT features.
	void setFeatures_SHOT(SHOT_features::Ptr shot_features)
	{
		SHOT_ptr = shot_features;
	}
	// Set keypoints SHOT
	void setKeypoints_SHOT(SHOT_features::Ptr shot_features)
	{
		keypointsSHOT_ptr = shot_features;
	}
	// Set FPFH features.
	void setFeatures_FPFH(FPFH_features::Ptr fpfh_features)
	{
		FPFH_ptr = fpfh_features;
	}
	void setKeypoints_FPFH(FPFH_features::Ptr fpfh_features)
	{
		keypointsFPFH_ptr = fpfh_features;
	}
	// Set VFH features
	void setFeatures_VFH(VFH_features::Ptr vfh_features)
	{
		VFH_ptr = vfh_features;
	}
	void setkeypoints_VFH(VFH_features::Ptr vfh_features)
	{
		keypointsVFH_ptr = vfh_features;
	}
	// Set LRF features
	void setFeatures_LRF(LRF_features::Ptr lrf_features)
	{
		LRF_ptr = lrf_features;
	}
	void setKeypoints_LRF(LRF_features::Ptr lrf_features)
	{
		keypointsLRF_ptr = lrf_features;
	}
	// Set point cloud with normal(PointCloudNormal Type).
	void setPointCloudNormal(PointCloudNormal::Ptr xyzn)
	{
		xyzn_ = xyzn;
	}



	// Return the pointer to the Point Cloud with Normal 
	// (PointCloudNormal Type).
	PointCloudNormal::Ptr getPointCloudNormal() const
	{
		return (xyzn_);
	}

	//get pointcloud
	PointCloudPtr getCloud() const
	{
		return cloudPtr;
	}
	//get keypoint
	PointCloudPtr getKeypoint() const
	{
		return keyPointsPtr;
	}
	//get keypoint indices
	pcl::PointIndices::Ptr getKeyPoint_indices() const
	{
		return key_indices_;
	}
	//get normal
	SurfaceNormals::Ptr getNormal() const
	{
		return normalPtr;
	}
	SurfaceNormals::Ptr getkeypointsNormal() const
	{
		return keypointsNormalPtr;
	}
	// Get the pointer to the cloud of feature descriptors (FPFH).
	FPFH_features::Ptr getFeatures_FPFH() const
	{
		return (FPFH_ptr);
	}
	FPFH_features::Ptr getkeypointsFeatures_FPFH() const
	{
		return (keypointsFPFH_ptr);
	}
	// Get the pointer to the cloud of feature descriptors (SHOT).
	SHOT_features::Ptr getFeatures_SHOT() const
	{
		return (SHOT_ptr);
	}
	SHOT_features::Ptr getkeypointsFeatures_SHOT() const
	{
		return (keypointsSHOT_ptr);
	}
	// Get the pointer to the cloud of feature descriptors (VFH).
	VFH_features::Ptr getFeatures_VFH() const
	{
		return (VFH_ptr);
	}
	VFH_features::Ptr getkeypointsFeatures_VFH() const
	{
		return (keypointsVFH_ptr);
	}
	// Get the pointer to the cloud of feature descriptors (LRF).
	LRF_features::Ptr getFeatures_LRF() const
	{
		return (LRF_ptr);
	}
	LRF_features::Ptr getkeypointsFeatures_LRF() const
	{
		return (keypointsLRF_ptr);
	}

	FeatureCloud& operator=(FeatureCloud& input)
	{
		FeatureCloud tmp;

		pcl::copyPointCloud(*input.cloudPtr, *tmp.cloudPtr);
		pcl::copyPointCloud(*input.xyzn_, *tmp.xyzn_);
		pcl::copyPointCloud(*input.keyPointsPtr, *tmp.keyPointsPtr);
		pcl::copyPointCloud(*input.normalPtr, *tmp.normalPtr);
		pcl::copyPointCloud(*input.FPFH_ptr, *tmp.FPFH_ptr);
		pcl::copyPointCloud(*input.SHOT_ptr, *tmp.SHOT_ptr);
		pcl::copyPointCloud(*input.VFH_ptr, *tmp.VFH_ptr);
		pcl::copyPointCloud(*input.LRF_ptr, *tmp.LRF_ptr);
		pcl::copyPointCloud(*input.keypointsNormalPtr, *tmp.keypointsNormalPtr);
		pcl::copyPointCloud(*input.keypointsFPFH_ptr, *tmp.keypointsFPFH_ptr);
		pcl::copyPointCloud(*input.keypointsSHOT_ptr, *tmp.keypointsSHOT_ptr);
		pcl::copyPointCloud(*input.keypointsVFH_ptr, *tmp.keypointsVFH_ptr);
		pcl::copyPointCloud(*input.keypointsLRF_ptr, *tmp.keypointsLRF_ptr);

		return tmp;
	}

private:

	PointCloudPtr cloudPtr;

	// Point Cloud with Normal
	PointCloudNormal::Ptr xyzn_;

	PointCloudPtr keyPointsPtr;
	pcl::PointIndices::Ptr key_indices_;

	SurfaceNormals::Ptr normalPtr;
	FPFH_features::Ptr FPFH_ptr;
	SHOT_features::Ptr SHOT_ptr;
	VFH_features::Ptr VFH_ptr;
	LRF_features::Ptr LRF_ptr;

	SurfaceNormals::Ptr keypointsNormalPtr;
	FPFH_features::Ptr keypointsFPFH_ptr;
	SHOT_features::Ptr keypointsSHOT_ptr;
	VFH_features::Ptr keypointsVFH_ptr;
	LRF_features::Ptr keypointsLRF_ptr;
};
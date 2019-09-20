#pragma once

#include "typeDef.h"
#include <utility>　　//pair的头文件
#include "recognition.h"

static enum VFH_method{VFH, CVFH};

extern "C"
{
	bool compareScore(std::pair<float, int> pair1, std::pair<float, int> pair2);
	//获取文件名
	bool computePairNum(std::pair<double, std::string> pair1, 
		std::pair<double, std::string> pair2);
	void sort_filelists(std::vector<std::string>& filists);//文件名排序
	void getFiles(std::string path, std::string ext, std::vector<std::string>& files);

	//求重心
	void computeMean(PointCloudPtr pointInput, Eigen::Vector3f &mean);
	//求重心及减去重心
	void remove_Mean(PointCloudPtr point, Eigen::Vector3f& mean);
	//求去重心的协方差矩阵并归一化
	void computeCovarianceMatrixNormalized(PointCloudPtr pointInput,
		Eigen::Vector3f& centroid,
		Eigen::Matrix3f& covariance);
	//归一化
	void normalizePoints(PointCloudPtr pointCloud, Eigen::Vector3f& mean,
		float global_scale);
	//计算分辨率
	float computeResolution(PointCloudPtr cloud);

	//法向量去除NAN点
	void removeNANfromNormal(FeatureCloud &cloud);
	//FPFH去除NAN点
	void removeNANfromFPFH(FPFH_features::Ptr feature_descriptor,
		FPFH_features::Ptr nanremoved,
		FeatureCloud& cloud);
	//SHOT去除NAN点
	void removeNANfromSHOT(SHOT_features::Ptr feature_descriptor,
		SHOT_features::Ptr nanremoved,
		FeatureCloud& cloud);
	//体素滤波
	void VoxelGrid_Filter(PointCloudPtr input,
		PointCloudPtr output,
		float leafsize = 1.0);
	//基于统计学滤波
	void StatisticalOutlierRemoval_Filter(PointCloudPtr input,
		PointCloudPtr output,
		int K = 30,
		float stddevMulThresh = 1.0);
	//均匀滤波
	void Uniform_Filter(PointCloudPtr input,
		PointCloudPtr output,
		float uniform_Radius = 1.0);
	//直通滤波
	void PassThrough_Filter(PointCloudPtr input,
		PointCloudPtr output,
		std::string axis,
		float upBound,
		float downBound,
		bool negative);

	//法向量计算
	void computeSurfaceNormals(FeatureCloud& cloud,
		int K = 50,
		float radius = 0,
		int numofthreads = 4);

	//构建法向量点云
	void construct_PointNormal(FeatureCloud& targetCloud,
		FeatureCloud& sourceCloud);

	//SHOT计算
	void computeFeatures_SHOT(FeatureCloud &cloud, 
		float R);
	//FPFH计算
	void computeFeatures_FPFH(FeatureCloud &cloud,
		float R);
	//局部参考坐标系LRF
	void computeFeatures_LRF(FeatureCloud &cloud,
		float rf_radius = 5.0);
	//VFH全局特征描述子
	void computeFeatures_VFH(FeatureCloud &cloud, VFH_method method);

	void keyPoints_NARF(const PointCloudPtr cloud_src,
		const PointCloudPtr keyPoints_NARF,
		float angular_resolution ,
		float support_size );

	//Harris关键点
	void keyPoints_HARRIS(PointCloudPtr cloud_src,
		 PointCloudPtr keyPoints_Harris,
		float radius = 0.03f,
		float threshold = 0.05f);

	//kdTree搜索最近点(PointT)
	void getNearestIndices(const PointCloudPtr cloudIn,
		const PointCloudPtr cloudQuery,
		PointCloudPtr cloudResult,
		pcl::PointIndices::Ptr indicesPtr);

	//kdTree搜索最近点(SHOT)
	void getNearestIndices_SHOT(const SHOT_features::Ptr cloudIn,
		const SHOT_features::Ptr cloudQuery,
		pcl::CorrespondencesPtr corrPtr);

	//对应点对估计
	void correspondence_estimation(std::string kernel_descriptors,
		FPFH_features::Ptr source_cloud,
		FPFH_features::Ptr target_cloud,
		pcl::Correspondences &all_corres);

	//对应点对剔除（ 互 相 对 应 ）
	void correspondences_rejection(FeatureCloud &source_cloud,
		FeatureCloud &target_cloud,
		pcl::Correspondences &correspondences,
		pcl::Correspondences &inliers,
		int MaximumIterations, float Inlierthreshold);
	//对应点对剔除（ 自定义约束）
	// tupleScale	  0.95	// Similarity measure used for tuples of feature points.
	// tuple_max_cnt_   1000	// Maximum tuple numbers.
	void advancedMatching(PointCloudPtr target, PointCloudPtr source, 
		pcl::Correspondences &correspondences,
		pcl::Correspondences &inliers,
		float tupleScale,
		int tuple_max_cnt_);
	//提取或去除索引的点云
	void extractIndicesPoints(PointCloudPtr pointInput,
		PointCloudPtr pointOutput,
		pcl::PointIndices::Ptr inliers,
		bool extractNegative);

	// 基于RANSAC的形状提取
	bool SACSegmentation_model(PointCloudPtr pointInput,
							pcl::ModelCoefficients::Ptr coefficients,
							pcl::PointIndices::Ptr inliers,
							pcl::SacModel modeltype = pcl::SACMODEL_PLANE,
							int maxIteration = 100,
							float distancethreshold = 1.0);
	//基于SAC的垂直平面检测、提取或删除
	bool SACSegmentation_plane(PointCloudPtr pointInput,
		pcl::ModelCoefficients::Ptr coefficients,
		pcl::PointIndices::Ptr inliers,
		int maxIteration = 100,
		float distancethreshold = 1.0
		);
	// 最小包围盒
	void OrientedBoundingBox(PointCloudPtr pointInput,
		Eigen::Vector3f &whd,
		Eigen::Vector3f &bboxT,
		Eigen::Quaternionf &bboxQ,
		float scalar,
		PointT& pcX, PointT& pcY, PointT& pcZ, PointT& initialoriginalPoint);

	//基于欧氏距离的聚类
	void EuclideanClusterExtraction(PointCloudPtr pointInput,
		std::vector<PointCloudPtr>& cloudListOutput,
		float clusterTolerance = 0.02,
		int minClusterSize = 100,
		int maxClusterSize = 1000);
	//基于区域生长的聚类
	void RegionGrowingClusterExtraction(PointCloudPtr pointInput,
		std::vector<PointCloudPtr>& cloudListOutput,
		SurfaceNormals::Ptr normalInput,
		int minClusterSize,
		int maxClusterSize,
		int numofNeighbour = 30,
		float smoothThreshold = 3.0 / 180.0 * M_PI,
		float curvatureThreshold = 1.0);

	/*****配准***/
	//SAC-IA
	void SAC_IA_Transform(FeatureCloud &source_cloud, 
		FeatureCloud &target_cloud,
		float minsampleDistance,
		int numofSample,
		int correspondenceRandomness,
		Eigen::Matrix4f& final_transformation);
	//ICP
	float iterative_closest_points(std::string solver,
		bool flag_reciprocal, bool flag_ransac,
		FeatureCloud &source_cloud, FeatureCloud &target_cloud,
		float transEps, float corresDist, float EuclFitEps, float outlThresh,int maxIteration,
		Eigen::Matrix4f &final_transformation);

	//SVD变换
	void SVD_Transform(PointCloudPtr target,
					PointCloudPtr source,
					Eigen::Matrix4f &transformation_SVD,
					int numOfIteration,
					 float threshold);
	//FGR变换
	void FGR_Transform(PointCloudPtr target,
		PointCloudPtr source,
		Eigen::Matrix4f &transformation_SVD,
		int numOfIteration,
		float threshold);
	//SACPrerejective变换
	void SAC_Transform(PointCloudNormal::Ptr target,
		PointCloudNormal::Ptr source,
		FPFH_features::Ptr fpfh_tgt,
		FPFH_features::Ptr fpfh_src,
		int maxIteration,
		float maxCorrDist,
		float inlierFration,
		Eigen::Matrix4f& final_transformation);


	//计算转换矩阵
	bool RotationTranslationCompute(FeatureCloud& cloudtarget,
		FeatureCloud& cloudsource,
		Eigen::Matrix4f &tranResult);

	//基于Hough3DGrouping的聚类识别粗配准
	void Hough3DGrouping_recognize(PointCloudPtr modelPointInput,
		PointCloudPtr scenePointInput,
		pcl::CorrespondencesPtr model_scene_corrs,
		pcl::PointCloud<LRFType>::Ptr modelRF,
		pcl::PointCloud<LRFType>::Ptr sceneRF,
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> & rototranslation,
		std::vector<pcl::Correspondences>& clustered_corr,
		float BinSize = 2.0,
		float cg_thresh_ = 5.0,
		float rf_radius = 5.0,
		bool interpolation = true,
		bool distanceWeight = false);

	//基于几何一致性聚类
	void GeometricConsistencyGrouping(float GCsize,
		float GCthreshold,
		PointCloudPtr modelPointInput,
		PointCloudPtr scenePointInput,
		pcl::CorrespondencesPtr model_scene_corrs,
		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> & rototranslation,
		std::vector<pcl::Correspondences>& clustered_corr);

	//显示点云对应点对
	void showPointCloudCorrespondences(std::string viewerName,
		PointCloudPtr cloudTarget_,
		PointCloudPtr cloudSource_,
		pcl::Correspondences &corr_, int showThreshold = 1);
	
}

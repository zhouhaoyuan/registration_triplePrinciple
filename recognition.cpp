#include "recognition.h"

bool Recognition::set_ObjectLibrary(std::vector< PointCloudPtr > &objectLib)
{
	if (objectLib.empty())
	{
		std::cout << "Error: the objectLib is empty\n";
		return false;
	}

	objectLibrary.clear();

	for (size_t i = 0; i < objectLib.size(); ++i)
	{
		std::cout << "\n ------- object" << i+1 << " ------- \n\n";
		FeatureCloud tempObj;
		PointCloudPtr tempCloudPtr(new PointCloud);

		tempCloudPtr = objectLib[i];

		std::vector<int> mapping;
		pcl::removeNaNFromPointCloud(*tempCloudPtr, *tempCloudPtr, mapping);
		//Resolution Calculation
		float resolution = 0.0;
		resolution = computeResolution(tempCloudPtr);
		//Statisstical filter
		StatisticalOutlierRemoval_Filter(tempCloudPtr, tempCloudPtr, 30, 1.0);
		//降采样获取关键点
		if (true)
		{
			float leafSize = resolution * 3;
			VoxelGrid_Filter(tempCloudPtr, tempCloudPtr, leafSize);
		}
		//更新分辨率
		resolution = computeResolution(tempCloudPtr);

		tempObj.setInputCloud(tempCloudPtr);
		//Normal Calculation
		int normal_K = 50;
		float normal_R = resolution * 5;
		computeSurfaceNormals(tempObj, normal_K, normal_R);
		//剔除法向量无效的点
		removeNANfromNormal(tempObj);

		PointCloudPtr sourceCloudPtr = tempObj.getCloud();
		PointCloudPtr sourceKeyPoints(new PointCloud);
		PointCloudPtr sourceKeyPoints_cal(new PointCloud);
		pcl::PointIndices::Ptr sourceIndices_ptr(new pcl::PointIndices);

		// Keypoints Detection关键点
		keyPoints_HARRIS(sourceCloudPtr, sourceKeyPoints_cal);
		//sourceKeyPoints_cal = sourceCloudPtr;
		std::cout << endl;
		
		if (sourceKeyPoints_cal->points.empty())
		{
			std::cout << "Model: " <<i<<" , No Keypoints were detected ... \n";
			sourceKeyPoints_cal = sourceCloudPtr;
		}

		// Calculate the indices of keypoints
		getNearestIndices(sourceCloudPtr, sourceKeyPoints_cal,
			sourceKeyPoints, sourceIndices_ptr);
		if (sourceKeyPoints->points.empty())
		{
			std::cout << "Error: the sourceKeypoints were empty ... \n";
		}

		tempObj.setKeypoints(sourceKeyPoints);
		tempObj.setKeyPoint_indices(sourceIndices_ptr);

		SurfaceNormals::Ptr sourcekeypointNormal(new SurfaceNormals);
		for (int i = 0; i < sourceIndices_ptr->indices.size(); ++i)
		{
			int index = sourceIndices_ptr->indices[i];
			sourcekeypointNormal->push_back(tempObj.getNormal()->points[index]);
		}
		tempObj.setKeypoints_Normal(sourcekeypointNormal);

		//Feature describe
		resolution = computeResolution(tempObj.getKeypoint());
		float FPFH_radius = resolution * 7;
		float SHOT_radius = resolution * 7;
		computeFeatures_FPFH(tempObj, FPFH_radius);
		computeFeatures_SHOT(tempObj, SHOT_radius);
		computeFeatures_VFH(tempObj, VFH_method::VFH);

		objectLibrary.push_back(tempObj);
	}
	return true;
}

bool Recognition::findTheMatchingObject(PointCloudPtr scene)
{
	if (scene == nullptr)
	{
		std::cout << "Error: the ptr is nullptr\n";
		return false;
	}
	if (scene->points.empty())
	{
		std::cout << "Error: the scene pointCloud is empty!\n";
		return false;
	}
	if (objectLibrary.empty())
	{
		std::cout << "\nError: the objectLibrary is empty!\n";
		return false;
	}

	std::cout << "\n---------- Scene Cloud ----------\n\n";

	FeatureCloud sceneSource;
	PointCloudPtr tempCloudPtr(new PointCloud);

	pcl::copyPointCloud(*scene, *tempCloudPtr);

	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*tempCloudPtr, *tempCloudPtr, mapping);
	//Resolution Calculation
	float resolution = 0.0;
	resolution = computeResolution(tempCloudPtr);
	//Statisstical filter
	StatisticalOutlierRemoval_Filter(tempCloudPtr, tempCloudPtr, 30, 1.0);
	//降采样获取关键点
	if (true)
	{
		float leafSize = resolution * 3;//分割时用3倍
		VoxelGrid_Filter(tempCloudPtr, tempCloudPtr, leafSize);
	}
	//更新分辨率
	resolution = computeResolution(tempCloudPtr);
	//设置 FeatureCloud
	sceneSource.setInputCloud(tempCloudPtr);

	/**
	* segmentation
	*/
	std::cout << "\n---- plane segmentation ----" << std::endl;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	int maxSACsegInteration = 100;
	float distThreshold = resolution * 8;
	SACSegmentation_model(sceneSource.getCloud(), coefficients, inliers,
		pcl::SACMODEL_PLANE, maxSACsegInteration, distThreshold);

	PointCloudPtr sceneWithoutPlane(new PointCloud);
	bool extractNegative = true;
	extractIndicesPoints(sceneSource.getCloud(), sceneWithoutPlane, inliers, extractNegative);

	if (true)
	{
		/**
		* Cluster Extration
		*/
		std::cout << "\n---- Euclidean_Cluster ----\n" << std::endl;
		std::vector<PointCloudPtr> clusteredPointList;
		float clusterTolerance = resolution * 2;
		int minClusterSize = 5000;
		int maxClusterSize = 100000;
		EuclideanClusterExtraction(sceneWithoutPlane, clusteredPointList,
			clusterTolerance, minClusterSize, maxClusterSize);

		/**
		* Oriented Bounding Box
		*/
		std::cout << "\n---- Oriented Bounding Box ----\n" << std::endl;
		std::vector<Eigen::Quaternionf> bboxQuaternionf_s(clusteredPointList.size(), Eigen::Quaternionf::Identity());
		std::vector<Eigen::Vector3f> bboxT_s(clusteredPointList.size(), Eigen::Vector3f::Zero());
		std::vector<Eigen::Vector3f> bboxWHD_s(clusteredPointList.size(), Eigen::Vector3f::Zero());
		std::vector<PointT> pcX_s(clusteredPointList.size(), PointT(0, 0, 0));
		std::vector<PointT> pcY_s(clusteredPointList.size(), PointT(0, 0, 0));
		std::vector<PointT> pcZ_s(clusteredPointList.size(), PointT(0, 0, 0));
		std::vector<PointT> initialoriginalPoint_s(clusteredPointList.size(), PointT(0, 0, 0));

		for (int i = 0; i < clusteredPointList.size(); ++i)
		{

			Eigen::Vector3f whd;
			Eigen::Vector3f bboxT;
			Eigen::Quaternionf bboxQ;
			float scalar = 0.0;
			PointT pcX, pcY, pcZ, initialoriginalPoint;

			OrientedBoundingBox(clusteredPointList[i], whd, bboxT, bboxQ,
				scalar, pcX, pcY, pcZ, initialoriginalPoint);

			bboxWHD_s[i] = whd;
			bboxQuaternionf_s[i] = bboxQ;
			bboxT_s[i] = bboxT;
			pcX_s[i] = pcX;
			pcY_s[i] = pcY;
			pcZ_s[i] = pcZ;
			initialoriginalPoint_s[i] = initialoriginalPoint;

			std::cout << "  whd : \n" << whd << std::endl
				<< "  bboxQ : \n" << bboxQ.toRotationMatrix() << std::endl
				<< "  bboxT : \n" << bboxT << std::endl
				<< "  pcX : " << pcX << " , pcY : " << pcY << " , pcZ : " << pcZ << std::endl
				<< "  initialoriginalPoint : " << initialoriginalPoint << std::endl << std::endl;
		}
		/**
		* recognition
		*/
		std::cout << "\n----------------------- recognition --------------------\n" << std::endl;
		std::vector<int> class_indexs;
		class_indexs.resize(clusteredPointList.size());

		for (int i = 0; i < clusteredPointList.size(); ++i)
		{
			std::cout << "\n-------- clusteredPointList " << i+1 << " --------\n\n";
			float resolution_ = computeResolution(clusteredPointList[i]);

			FeatureCloud clusteredCloud;
			clusteredCloud.setInputCloud(clusteredPointList[i]);

			//法向量
			int normalK = 50;
			float normalR = resolution_ * 5;
			computeSurfaceNormals(clusteredCloud, normalK, normalR);
			//剔除法向量无效的点
			removeNANfromNormal(clusteredCloud);
			//关键点
			PointCloudPtr keypoint_cal(new PointCloud);
			PointCloudPtr keypoints(new PointCloud);
			pcl::PointIndices::Ptr keypointIndices_ptr(new pcl::PointIndices);

			keyPoints_HARRIS(clusteredCloud.getCloud(), keypoint_cal);

			if (keypoint_cal->points.empty())
			{
				std::cout << "No Keypoints were detected ... \n";
				keypoint_cal = clusteredCloud.getCloud();
			}
			// Calculate the indices of keypoints
			getNearestIndices(clusteredCloud.getCloud(), keypoint_cal,
				keypoints, keypointIndices_ptr);

			clusteredCloud.setKeypoints(keypoints);
			clusteredCloud.setKeyPoint_indices(keypointIndices_ptr);

			SurfaceNormals::Ptr keypointNormal(new SurfaceNormals);
			for (int j = 0; j < keypointIndices_ptr->indices.size(); ++j)
			{
				int index = keypointIndices_ptr->indices[j];
				keypointNormal->push_back(clusteredCloud.getNormal()->points[index]);
			}
			clusteredCloud.setKeypoints_Normal(keypointNormal);

			//SHOT
			resolution = computeResolution(clusteredCloud.getKeypoint());
			float SHOT_radius = resolution_ * 7;
			computeFeatures_SHOT(clusteredCloud, SHOT_radius);
			computeFeatures_FPFH(clusteredCloud, SHOT_radius);
			computeFeatures_VFH(clusteredCloud, VFH_method::VFH);

			//与模板库比对特征的欧氏距离
			float* weightCount = new float[objectLibrary.size()];
			for (size_t ii = 0; ii < objectLibrary.size(); ++ii)
			{
				weightCount[ii] = 0;
			}
			std::cout << "\nThe weightCount : " << std::endl;
			for (size_t ii = 0; ii < objectLibrary.size(); ++ii)
			{
				bool vfh = true;

				if (vfh)
				{
					float dist = 0.0;
					for (size_t nn = 0; nn < 308; ++nn)
					{
						dist += std::pow(clusteredCloud.getkeypointsFeatures_VFH()->at(0).histogram[nn] - 
							objectLibrary[ii].getkeypointsFeatures_VFH()->at(0).histogram[nn], 2);
					}
					dist = std::pow(dist, 0.5);
					weightCount[ii] = dist;
					std::cout << dist << " , ";
				}							
			}
			std::cout << std::endl<<std::endl;
			
			//排序
			int* index_weight = new int[objectLibrary.size()];
			for (size_t ii = 0; ii < objectLibrary.size(); ++ii)
			{
				index_weight[ii] = ii;
			}

			float* tempWeightList = new float[objectLibrary.size()];
			*tempWeightList = *weightCount;

			shellsort<float>(tempWeightList, 0, objectLibrary.size() - 1, index_weight);

			/**
			* 配准验证
			*/
			std::cout << "\n---------------- Hypothesis Verification  ---------------\n\n";
			int number = 3;
			float* scoreList = new float[number];

			for (size_t num = 0; num < number; ++num)
			{
				std::cout << "\n---------------- Matching "<<num<<" ---------------\n\n";
				pcl::CorrespondencesPtr corr_(new pcl::Correspondences);
				pcl::CorrespondencesPtr corr_rejected(new pcl::Correspondences);

				bool shot = true;
				bool fpfh = false;

				if (shot)
				{
					pcl::registration::CorrespondenceEstimation<SHOT352_FeatureT, SHOT352_FeatureT> est;
					est.setInputSource(clusteredCloud.getkeypointsFeatures_SHOT());
					est.setInputTarget(objectLibrary[ index_weight[num] ].getkeypointsFeatures_SHOT());
					est.determineReciprocalCorrespondences(*corr_);
					std::cout << "get the corresponding Indices_SHOT size : " << corr_->size() << std::endl;
				}
				if (fpfh)
				{

					pcl::registration::CorrespondenceEstimation<FPFH33_FeatureT, FPFH33_FeatureT> est;
					est.setInputSource(clusteredCloud.getkeypointsFeatures_FPFH());
					est.setInputTarget(objectLibrary[i].getkeypointsFeatures_FPFH());
					est.determineReciprocalCorrespondences(*corr_);
					std::cout << "get the corresponding Indices_FPFH size : " << corr_->size() << std::endl;
				}

				//约束去除错误点对
				*corr_rejected = *corr_;
				//float tupleScale = 0.95;
				//int tuple_max_cnt_ = 500;
				//advancedMatching(objectLibrary[index_weight[num]].getKeypoint(),
				//	clusteredCloud.getKeypoint(),
				//	*corr_, *corr_rejected, tupleScale, tuple_max_cnt_);

				//std::cout << "advancedMatching size : " << corr_rejected->size() << std::endl;

				//根据匹配点对重新确立关键点
				FeatureCloud targetCloud_Keypoint, sourceCloud_Keypoint;
				PointCloudPtr targetKeypoint_(new PointCloud);
				PointCloudPtr sourceKeypoint_(new PointCloud);
				NormalsPtr targetKeypointNormal_(new SurfaceNormals);
				NormalsPtr sourceKeypointNormal_(new SurfaceNormals);
				FPFH_features::Ptr targetKeypointFPFH_(new FPFH_features);
				FPFH_features::Ptr sourceKeypointFPFH_(new FPFH_features);

				for (size_t ii = 0; ii < corr_rejected->size(); ++ii)
				{
					PointT source = clusteredCloud.getKeypoint()->at(corr_rejected->at(ii).index_query);
					PointT target = objectLibrary[index_weight[num]].getKeypoint()->at(corr_rejected->at(ii).index_match);

					NormalT sourceNormal = clusteredCloud.getkeypointsNormal()->at(corr_rejected->at(ii).index_query);
					NormalT targetNormal = objectLibrary[index_weight[num]].getkeypointsNormal()->at(corr_rejected->at(ii).index_match);

					FPFH33_FeatureT sourceFPFH = clusteredCloud.getkeypointsFeatures_FPFH()->at(corr_rejected->at(ii).index_query);
					FPFH33_FeatureT targetFPFH = objectLibrary[index_weight[num]].getkeypointsFeatures_FPFH()->at(corr_rejected->at(ii).index_match);

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
				//construct_PointNormal(objectLibrary[index_weight[num]], clusteredCloud);
				construct_PointNormal(targetCloud_Keypoint, sourceCloud_Keypoint);

				std::cout << "\n--------------- registration ---------------\n\n";

				Eigen::Matrix4f tran = Eigen::Matrix4f::Identity();
				//SVD
				if (true) 
				{
					float threshold = 0.01;
					int numIteration = 1;
					SVD_Transform(targetCloud_Keypoint.getKeypoint(),
						sourceCloud_Keypoint.getKeypoint(),
						tran, numIteration, threshold);
				}

				//SAC-IA
				if (false)
				{
					float minsampleDistance = resolution * 2;
					int numofSample = 100;
					int correspondenceRandomness = 20;
					SAC_IA_Transform(sourceCloud_Keypoint, targetCloud_Keypoint, minsampleDistance,
						numofSample, correspondenceRandomness, tran);
				}

				//ICP
				float score = 0.0;
				if (true)
				{
					float transEps = 1e-10;//设置两次变化矩阵之间的差值（一般设置为1e-10即可）
					float maxCorresDist = 0.7;//设置对应点对之间的最大距离（此值对配准结果影响较大）
					float EuclFitEps = 0.0001;//设置收敛条件是均方误差和小于阈值,停止迭代；
					float outlThresh = 0.1;
					int maxIteration = 60;
					score = iterative_closest_points("SVD", false, false,
							sourceCloud_Keypoint, targetCloud_Keypoint,
							transEps, maxCorresDist, EuclFitEps,
							outlThresh, maxIteration, tran);
				}
				scoreList[num] = score;
			}
			
			int final_Index = index_weight[0];
			float final_Score = scoreList[0];
			for (size_t ii = 1; ii < number; ++ii)
			{
				if (final_Score > scoreList[ii])
				{
					final_Score = scoreList[ii];
					final_Index = index_weight[ii];
				}
				else if (final_Score == scoreList[ii])
				{
					if (weightCount[final_Index] > weightCount[index_weight[ii]])
					{
						final_Score = scoreList[ii];
						final_Index = index_weight[ii];
					}
				}
			}

			std::cout << "cluster_pointlist " << i + 1 << " , weight : " << weightCount[final_Index]
				<< " , score : "<<final_Score
				<< " , Category : " << final_Index + 1 << std::endl;

			class_indexs[i] = final_Index + 1;

			delete[] scoreList;
			delete[] tempWeightList;
			delete[] index_weight;
			delete[] weightCount;
			
		}

		/**
		* 显示
		*/
		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.setBackgroundColor(0, 0, 0);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> color(sceneWithoutPlane, 0, 0, 255);
		viewer.addPointCloud(sceneWithoutPlane, color, "sceneWithoutPlane");

		//if (!clusteredPointList.empty())
		if (true)
		{
			pcl::PLYWriter writer;

			srand((int)time(NULL));

			for (int i = 0; i < clusteredPointList.size(); ++i)
			{
				std::stringstream ss;
				ss << i;

				//writer.write<PointT>("cluster" + ss.str() + ".ply", *clusteredPointList[i]);

				int r, g, b;

				//r = 255 * rand() / (RAND_MAX + 1.0f);
				//g = 255 * rand() / (RAND_MAX + 1.0f);
				//b = 255 * rand() / (RAND_MAX + 1.0f);

				if (class_indexs[i] == 1)
				{
					r = 255; g = 0; b = 0;
				}
				else if (class_indexs[i] == 2)
				{
					r = 0; g = 255; b = 0;
				}
				else if (class_indexs[i] == 3)
				{
					r = 0; g = 0; b = 255;
				}

				pcl::visualization::PointCloudColorHandlerCustom<PointT> color_(clusteredPointList[i], r, g, b);

				std::string cloudName = "cluster_" + ss.str();
				viewer.addPointCloud(clusteredPointList[i], color_, cloudName);

				std::string bboxName = "OBB_box_" + ss.str();
				viewer.addCube(bboxT_s[i], bboxQuaternionf_s[i],
					bboxWHD_s[i](0), bboxWHD_s[i](1), bboxWHD_s[i](2), bboxName);

				viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
					pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, bboxName);
				viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
					0.0, 1.0, 0.0, bboxName);

				viewer.addArrow(pcX_s[i], initialoriginalPoint_s[i], 1.0, 0.0, 0.0, false, "arrow_X_" + ss.str());
				viewer.addArrow(pcY_s[i], initialoriginalPoint_s[i], 0.0, 1.0, 0.0, false, "arrow_Y" + ss.str());
				viewer.addArrow(pcZ_s[i], initialoriginalPoint_s[i], 0.0, 0.0, 1.0, false, "arrow_Z" + ss.str());

			}
		}

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(1);
		}

	}

	if (false)
	{
		//Normal Calculation
		int normal_K = 50;
		float normal_R = resolution * 5;

		//sceneSource.setInputCloud(sceneWithoutPlane);

		computeSurfaceNormals(sceneSource, normal_K, normal_R);
		//剔除法向量无效的点
		removeNANfromNormal(sceneSource);

		PointCloudPtr sourceCloudPtr = sceneSource.getCloud();
		PointCloudPtr sourceKeyPoints(new PointCloud);
		PointCloudPtr sourceKeyPoints_cal(new PointCloud);
		pcl::PointIndices::Ptr sourceIndices_ptr(new pcl::PointIndices);

		// Keypoints Detection关键点
		keyPoints_HARRIS(sourceCloudPtr, sourceKeyPoints_cal);
		//sourceKeyPoints_cal = sourceCloudPtr;
		std::cout << endl;

		if (sourceKeyPoints_cal->points.empty())
		{
			std::cout << "Error: the scene has No Keypoints detected ... \n";
			sourceKeyPoints_cal = sourceCloudPtr;
		}

		// Calculate the indices of keypoints
		getNearestIndices(sourceCloudPtr, sourceKeyPoints_cal,
			sourceKeyPoints, sourceIndices_ptr);

		sceneSource.setKeypoints(sourceKeyPoints);
		sceneSource.setKeyPoint_indices(sourceIndices_ptr);

		SurfaceNormals::Ptr sourcekeypointNormal(new SurfaceNormals);
		for (int i = 0; i < sourceIndices_ptr->indices.size(); ++i)
		{
			int index = sourceIndices_ptr->indices[i];
			sourcekeypointNormal->push_back(sceneSource.getNormal()->points[index]);
		}
		sceneSource.setKeypoints_Normal(sourcekeypointNormal);
		/**
		* compute SHOT for keypoints
		*/
		float shot_radius = resolution * 7;
		computeFeatures_SHOT(sceneSource, shot_radius);

		std::vector< std::vector<bool> > recognizeMask;
		std::vector< std::vector< pcl::Correspondences>> recognizeClusterCorrs;
		std::vector< std::vector< Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>  > > recognizeTranslations;

		/**
		*  Find Model-Scene Correspondences with KdTree
		*/
		for (size_t objectNum = 0; objectNum < objectLibrary.size(); ++objectNum)
		{
			pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences);
			
			if (false)
			{			
				pcl::search::KdTree<SHOT352_FeatureT> match_search;
				match_search.setInputCloud(objectLibrary[objectNum].getkeypointsFeatures_SHOT());
				std::vector<int> model_good_keypoints_indices;
				std::vector<int> scene_good_keypoints_indices;

				for (size_t i = 0; i < sceneSource.getkeypointsFeatures_SHOT()->points.size(); ++i)
				{
					std::vector<int> neigh_indices(1);
					std::vector<float> neigh_sqr_dists(1);
					if (!pcl_isfinite(sceneSource.getkeypointsFeatures_SHOT()->at(i).descriptor[0]))  //skipping NaNs
					{
						continue;
					}
					int found_neighs = match_search.nearestKSearch(sceneSource.getkeypointsFeatures_SHOT()->at(i),
						1, neigh_indices, neigh_sqr_dists);
					if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
					{
						pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
						model_scene_corrs->push_back(corr);
						model_good_keypoints_indices.push_back(corr.index_query);
						scene_good_keypoints_indices.push_back(corr.index_match);
					}
				}
				pcl::PointCloud<PointT>::Ptr model_good_kp(new pcl::PointCloud<PointT>());
				pcl::PointCloud<PointT>::Ptr scene_good_kp(new pcl::PointCloud<PointT>());
				pcl::copyPointCloud(*objectLibrary[objectNum].getKeypoint(), model_good_keypoints_indices, *model_good_kp);
				pcl::copyPointCloud(*sceneSource.getKeypoint(), scene_good_keypoints_indices, *scene_good_kp);

				std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
			}

			pcl::registration::CorrespondenceEstimation<SHOT352_FeatureT, SHOT352_FeatureT> est;
			est.setInputSource(sceneSource.getkeypointsFeatures_SHOT());
			est.setInputTarget(objectLibrary[objectNum].getkeypointsFeatures_SHOT());
			est.determineReciprocalCorrespondences(*model_scene_corrs);
			std::cout << "getNearestIndices_SHOT size : " << model_scene_corrs->size() << std::endl;

			//约束去除错误点对
			//float tupleScale = 0.95;
			//int tuple_max_cnt_ = 500;
			//advancedMatching(objectLibrary[objectNum].getKeypoint(), sceneSource.getKeypoint(),
			//	*model_scene_corrs, *model_scene_corrs, tupleScale, tuple_max_cnt_);
			//std::cout << "advancedMatching reject size : " << model_scene_corrs->size() << std::endl;

			//correspondences_rejection(sceneSource, objectLibrary[i],
			//	*model_scene_corrs, *model_scene_corrs, 1000, 20);
			//std::cout << "correspondences_rejection size : " << model_scene_corrs->size() << std::endl;

			/**
			* Clustering
			*/
			std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
			std::vector< pcl::Correspondences > clustered_corrs;
			//clustered_corrs.push_back(*model_scene_corrs);

			if (true)
			{
				//LRF
				computeFeatures_LRF(sceneSource, shot_radius);

				float binSize = 5 * resolution;
				float houghThreshold = 20 * resolution;
				float rf_radius =  5 * resolution;
				bool interpolation = true;
				bool distanceWeight = false;

				Hough3DGrouping_recognize(
					objectLibrary[objectNum].getKeypoint(),
					sceneSource.getKeypoint(),
					model_scene_corrs,
					objectLibrary[objectNum].getkeypointsFeatures_LRF(),
					sceneSource.getkeypointsFeatures_LRF(),
					rototranslations, clustered_corrs,
					binSize, houghThreshold, rf_radius,
					interpolation, distanceWeight);
			}
			else
			{
				float GCsize = 2.0 * resolution;
				float GCthreshold = 5.0 * resolution;

				 GeometricConsistencyGrouping( 
					 GCsize,
					 GCthreshold,
					 objectLibrary[objectNum].getKeypoint(),
					 sceneSource.getKeypoint(),
					 model_scene_corrs,
					 rototranslations,
					 clustered_corrs);
			}
			std::cout << "Hough3DGrouping_recognize, clustered_corrs size : " << clustered_corrs.size() << std::endl;
			recognizeClusterCorrs.push_back(clustered_corrs);
			recognizeTranslations.push_back(rototranslations);

			///**
			//* Stop if no instances
			//*/
			//if (rototranslations.size() <= 0)
			//{
			//	cout << "*** No instances found! ***" << endl;
			//	//return false;
			//}
			//else
			//{
			//	cout << "Recognized Instances: " << rototranslations.size() << endl << endl;
			//}

			///**
			//* Generates clouds for each instances found
			//*/
			//std::vector<PointCloudPtr> instances;

			//for (size_t i = 0; i < rototranslations.size(); ++i)
			//{
			//	PointCloudPtr rotated_model(new PointCloud);
			//	pcl::transformPointCloud( *objectLibrary[objectNum].getKeypoint(), *rotated_model, rototranslations[i]);
			//	instances.push_back(rotated_model);
			//}

			/**
			* ICP
			*/
			//std::vector<PointCloudConstPtr> registered_instances;
			//if (false)
			//{
			//	cout << "----------- ICP ---------" << endl;

			//	std::vector<double> score;
			//	std::vector<bool> mask(rototranslations.size(), false);
			//	for (size_t i = 0; i < rototranslations.size(); ++i)
			//	{
			//		pcl::IterativeClosestPoint<PointT, PointT> icp;
			//		icp.setMaximumIterations(100);
			//		icp.setTransformationEpsilon(1e-10);
			//		//icp.setMaxCorrespondenceDistance();
			//		icp.setInputTarget(sceneSource.getKeypoint());
			//		icp.setInputSource(instances[i]);
			//		PointCloudPtr registered(new PointCloud);
			//		icp.align(*registered);
			//		registered_instances.push_back(registered);
			//		cout << "Instance " << i << " ";
			//		if (icp.hasConverged())
			//		{
			//			cout << "Aligned! score: " << icp.getFitnessScore() << endl;
			//		}
			//		else
			//		{
			//			cout << "Not Aligned!" << endl;
			//		}
			//		double score_ = icp.getFitnessScore();
			//		score.push_back(score_);
			//	}
			//	int index = 0;
			//	double min = score[0];
			//	for (size_t ii = 0; ii < score.size(); ++ii)
			//	{
			//		if (score[ii] < min)
			//		{
			//			min = score[ii];
			//			index = ii;
			//		}					
			//	}
			//	mask[index] = true;
			//	recognizeMask.push_back(mask);
			//	cout << "-----------------" << endl << endl;
			//}

		}
		/**
		* Oriented Bounding Box
		*/
		//if (false)
		//{
		//	std::vector<std::vector<PointCloudPtr>> recognize_instances;
		//	std::vector< std::vector<Eigen::Quaternionf>> recognize_bboxQuaternionf_s;
		//	std::vector< std::vector<Eigen::Vector3f>>  recognize_bboxT_s;
		//	std::vector< std::vector<Eigen::Vector3f>> recognize_bboxWHD_s;
		//	std::vector< std::vector<PointT>> recognize_pcX_s;
		//	std::vector< std::vector<PointT>> recognize_pcY_s;
		//	std::vector< std::vector<PointT>> recognize_pcZ_s;
		//	std::vector< std::vector<PointT>> recognize_initialoriginalPoint_s;

		//	for (size_t i = 0; i < recognizeClusterCorrs.size(); ++i)
		//	{
		//		int size = recognizeClusterCorrs[i].size();

		//		std::vector<PointCloudPtr> pointCloudPtr_s;
		//		std::vector<Eigen::Quaternionf> bboxQuaternionf_s(size, Eigen::Quaternionf::Identity());
		//		std::vector<Eigen::Vector3f> bboxT_s(size, Eigen::Vector3f::Zero());
		//		std::vector<Eigen::Vector3f> bboxWHD_s(size, Eigen::Vector3f::Zero());
		//		std::vector<PointT> pcX_s(size, PointT(0, 0, 0));
		//		std::vector<PointT> pcY_s(size, PointT(0, 0, 0));
		//		std::vector<PointT> pcZ_s(size, PointT(0, 0, 0));
		//		std::vector<PointT> initialoriginalPoint_s(size, PointT(0, 0, 0));

		//		for (size_t clusterNum = 0; clusterNum < size; ++clusterNum)
		//		{
		//			//if (recognizeMask[i][clusterNum] == false)
		//			//	continue;

		//			/**
		//			* Orienter Bounding Box
		//			*/
		//			pcl::Correspondences corr_ = recognizeClusterCorrs[i][clusterNum];
		//			PointCloudPtr clusteredCloud(new PointCloud);

		//			for (size_t j = 0; j < corr_.size(); ++j)
		//			{
		//				clusteredCloud->points.push_back(sceneSource.getKeypoint()->points[corr_[j].index_match]);
		//			}
		//			pointCloudPtr_s.push_back(clusteredCloud);

		//			Eigen::Vector3f whd;
		//			Eigen::Vector3f bboxT;
		//			Eigen::Quaternionf bboxQ;
		//			float scalar = 0.0;
		//			PointT pcX, pcY, pcZ, initialoriginalPoint;

		//			OrientedBoundingBox(clusteredCloud, whd, bboxT, bboxQ,
		//				scalar, pcX, pcY, pcZ, initialoriginalPoint);

		//			bboxWHD_s[i] = whd;
		//			bboxQuaternionf_s[i] = bboxQ;
		//			bboxT_s[i] = bboxT;
		//			pcX_s[i] = pcX;
		//			pcY_s[i] = pcY;
		//			pcZ_s[i] = pcZ;
		//			initialoriginalPoint_s[i] = initialoriginalPoint;

		//			std::cout << "  whd : \n" << whd << std::endl
		//				<< "  bboxQ : \n" << bboxQ.toRotationMatrix() << std::endl
		//				<< "  bboxT : \n" << bboxT << std::endl
		//				<< "  pcX : " << pcX << " , pcY : " << pcY << " , pcZ : " << pcZ << std::endl
		//				<< "  initialoriginalPoint : " << initialoriginalPoint << std::endl;
		//		}

		//		recognize_bboxQuaternionf_s.push_back(bboxQuaternionf_s);
		//		recognize_bboxT_s.push_back(bboxT_s);
		//		recognize_bboxWHD_s.push_back(bboxWHD_s);
		//		recognize_initialoriginalPoint_s.push_back(initialoriginalPoint_s);
		//		recognize_pcX_s.push_back(pcX_s);
		//		recognize_pcY_s.push_back(pcY_s);
		//		recognize_pcZ_s.push_back(pcZ_s);
		//		recognize_instances.push_back(pointCloudPtr_s);
		//	}
		//}


		/**
		* 显示
		*/
		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.setBackgroundColor(0, 0, 0);
		pcl::visualization::PointCloudColorHandlerCustom<PointT> sceneColor(sceneSource.getKeypoint(),
			250, 250, 250);
		viewer.addPointCloud(sceneSource.getKeypoint(), sceneColor, "SceneCloud");

		for (int k = 0; k < recognizeClusterCorrs.size(); ++k)
		{
			PointCloudPtr model_keypoints(new PointCloud);
			pcl::copyPointCloud(*objectLibrary[k].getKeypoint(),
				*model_keypoints);
			Eigen::Matrix4f tran = Eigen::Matrix4f::Identity();
			tran(0, 3) = 130;
			if(k == 1)tran(1, 3) =  100 ;
			if (k == 2)tran(1, 3) = -100;
			pcl::transformPointCloud(*model_keypoints, *model_keypoints, tran);

			pcl::visualization::PointCloudColorHandlerCustom<PointT> model_color_handler(model_keypoints, 255, 255, 128);

			std::stringstream s;
			s << k;
			viewer.addPointCloud(model_keypoints, model_color_handler, "model_" + s.str());

			for (int n = 0; n < recognizeClusterCorrs[k].size(); ++n)
			{
				PointCloudPtr cloud_(new PointCloud);
				//pcl::copyPointCloud(*cloud_, *recognize_instances[k][n]);

				int r, g, b;
				if (k == 0)
				{
					r = 255; g = 0; b = 0;
				}
				else if (k == 1)
				{
					r = 0; g = 255; b = 0;
				}
				else if (k == 2)
				{
					r = 0; g = 0; b = 255;
				}
				std::stringstream ss;
				ss << k << "_" << n;

				std::string cloudName = "cluster_" + ss.str();
				std::string bboxName = "OBB_" + ss.str();

				//if (false)
				//{
				//	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_(cloud_, r, g, b);
				//	viewer.addPointCloud(cloud_, color_, cloudName);
				//	viewer.addCube(recognize_bboxT_s[k][n], recognize_bboxQuaternionf_s[k][n],
				//		recognize_bboxWHD_s[k][n](0),
				//		recognize_bboxWHD_s[k][n](1),
				//		recognize_bboxWHD_s[k][n](2),
				//		bboxName);

				//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
				//		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, bboxName);
				//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
				//		0.0, 1.0, 0.0, bboxName);
				//}

				if (true)
				{
					for (size_t j = 0; j < recognizeClusterCorrs[k][n].size(); ++j)
					{
						std::stringstream ss_line;
						ss_line << "correspondence_line" << k << "_" << n<<"_"<<j;
						PointT& model_point = model_keypoints->at(recognizeClusterCorrs[k][n][j].index_match);
						PointT& scene_point = sceneSource.getKeypoint()->at(recognizeClusterCorrs[k][n][j].index_query);
						if(j % 10 == 0)
							viewer.addLine<PointT, PointT>(model_point, scene_point, r, g, b, ss_line.str());
					}
				}
			}
		}

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(1);
		}
	}
}
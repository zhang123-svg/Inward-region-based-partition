#include "myself.h"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;    //PointXYZ 成员：float x，y，z;表示了xyz3D信息，可以通过points[i].data[0]或points[i].x访问点X的坐标值
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
using namespace pcl;

#define Random(x) (rand() % x)  

int main(int argc, char ** argv)   //argc和argv用于读取命令行指令
{
	// 输入原始点云input_cloud_ptr和模型点云. cloud_CAD    
	PointCloud<PointT>::Ptr input_cloud_ptr(new pcl::PointCloud<PointT>);
	PointCloud<PointT>::Ptr Objects(new pcl::PointCloud<PointT>);
	PointCloud<PointT>::Ptr cloud_CAD(new PointCloud<PointT>);								//CAD模型点云，待配准
	Load_xyz(input_cloud_ptr);				//调用Load_xyz函数，输入的txt文件，输出pcd文件												//读取，自写函数
	Objects = RemovePlain(input_cloud_ptr);
	//Display(Objects)


	//读取CAD

	//if (pcl::io::loadPCDFile("bottle.pcd", *cloud_CAD) == -1)
	//{
	//	PCL_ERROR("ERROR: Could not read CAD cloud ");
	//	return (3);
	//}

	PCL_INFO("Done loading cloud\n");
	//点云加载完毕
	
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;								//创建一个快速K近邻查询，查询的时候若该点在点云中，则第一个近邻点是其本身
	kdtree.setInputCloud(Objects);
	int k =2;
	float everagedistance =0;
	for (int i =0; i < Objects->size()/2;i++)
	{
	vector<int> nnh ;
	vector<float> squaredistance;
	//  pcl::PointXYZ p;
	//   p = cloud->points[i];
	kdtree.nearestKSearch(Objects->points[i],k,nnh,squaredistance);
	everagedistance += sqrt(squaredistance[1]);
	//   cout<<everagedistance<<endl;
	}

	everagedistance = everagedistance/(Objects->size()/2);
	cout<<"average distance is : "<<everagedistance<<endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal前三项是法向，最后一项是曲率
	normEst.setInputCloud(Objects);
	normEst.setSearchMethod(tree);
	// normEst.setRadiusSearch(2);  //法向估计的半径
	normEst.setKSearch(9);			//法向估计的点数
	normEst.compute(*normals);
	cout << "normal size is " << normals->size() << endl;

	//normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
	est.setInputCloud(Objects);
	est.setInputNormals(normals);
	//  est.setAngleThreshold(90);
	//   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	est.setSearchMethod(tree);
	est.setKSearch(100);  //一般这里的数值越高，最终边界识别的精度越好
						 //  est.setRadiusSearch(everagedistance);  //搜索半径
	est.compute(boundaries);

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> noBoundPoints;
	int countBoundaries = 0;
	for (int i = 0; i<Objects->size(); i++) {
		uint8_t x = (boundaries.points[i].boundary_point);				// boundary_point是一个flag，代表这个点是否在边界上

		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if (a == 1)
		{
			//  boundPoints.push_back(cloud->points[i]);
			(*boundPoints).push_back(Objects->points[i]);
  			countBoundaries++;
		}
		else
			noBoundPoints.push_back(Objects->points[i]);

	}
	std::cout << "boudary size is：" << countBoundaries << std::endl;

	//pcl::io::savePCDFileASCII("boudary.pcd", *boundPoints);

	//pcl::visualization::CloudViewer viewer("test");
	//viewer.showCloud(boundPoints);
	//while (!viewer.wasStopped())
	//{
	//}
	


	/*超体聚类*/


	float voxel_resolution = 1.f; //   3.5 
	float seed_resolution =  2.0f; //    11 
	float color_importance = 0.0f;
	float spatial_importance = 0.4f;
	float normal_importance = 4.0f;
	bool use_single_cam_transform = false;
	bool use_supervoxel_refinement = true;
	unsigned int k_factor = 0;
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);


	super.setUseSingleCameraTransform(use_single_cam_transform);
	super.setInputCloud(boundPoints);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	PCL_INFO("Extracting supervoxels\n");
	super.extract(supervoxel_clusters);
	PCL_INFO("Getting supervoxel adjacency\n");
	super.getSupervoxelAdjacency(supervoxel_adjacency);
	pcl::PointCloud<pcl::PointXYZL>::Ptr overseg = super.getLabeledCloud();

	int label_max1 = 0;
	for (int i = 0; i< overseg->size(); i++) {
		if (overseg->points[i].label>label_max1)
			label_max1 = overseg->points[i].label;
	}
	std::cout << "SuperVoxel Label is " << label_max1 << "\n" << endl;

	/*超体聚类的点云存储、可视化*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	ColoredCloud1->height = 1;
	ColoredCloud1->width = overseg->size();
	ColoredCloud1->resize(overseg->size());
	for (int i = 0; i < label_max1; i++) {
		int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);

		for (int j = 0; j < overseg->size(); j++) {
			if (overseg->points[j].label == i) {
				ColoredCloud1->points[j].x = overseg->points[j].x;
				ColoredCloud1->points[j].y = overseg->points[j].y;
				ColoredCloud1->points[j].z = overseg->points[j].z;
				ColoredCloud1->points[j].r = color_R;
				ColoredCloud1->points[j].g = color_G;
				ColoredCloud1->points[j].b = color_B;
			}
		}
	}
	// //显示超体分割的结果
	//Display(ColoredCloud1);




	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
	clock_t Super = clock();
	/* LCCP分割*/    
	float concavity_tolerance_threshold =15;
	float smoothness_threshold = 5;
	uint32_t min_segment_size = 0; 
	bool use_extended_convexity = true;
	bool use_sanity_criterion = true;
	PCL_INFO("Starting Segmentation\n");
	pcl::LCCPSegmentation<PointT> lccp;
	// 输入参数过程
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.setMinSegmentSize(min_segment_size);
	lccp.segment();
	PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
	// relabel的过程
	lccp.relabelCloud(* lccp_labeled_cloud);
	SuperVoxelAdjacencyList sv_adjacency_list;
	lccp.getSVAdjacencyList(sv_adjacency_list);

	/*LCCP显示与保存*/
	int label_max2 = 0;												    // 最大的点云label
	//LCCP_S_D(lccp_labeled_cloud, label_max2);

	/*提取单个边缘*/
	vector<PointCloud<PointT>::Ptr> LCCP_Extract_vec;
	int Extract_Num = 0;												// 边缘数目
	LCCP_Extract(lccp_labeled_cloud, LCCP_Extract_vec, Extract_Num);
	/*计算PCA特征的一些参数*/
	vector<Eigen::Vector4f> pcaCentroid(Extract_Num);
	vector<Eigen::Matrix3f> convariance(Extract_Num);
	vector<Eigen::SelfAdjointEigenSolver< Eigen::Matrix3f>> eigen_solver(Extract_Num);
	vector<Eigen::Matrix3f> eigenVectorsPCA(Extract_Num);
	vector<Eigen::Vector3f> eigenValuesPCA(Extract_Num);

	/*中间变量*/
	vector<PointT> min_p(Extract_Num), max_p(Extract_Num);
	vector<Eigen::Vector3f> whd(Extract_Num);
	vector<float> scale(Extract_Num);
	vector<Eigen::Vector3f> c(Extract_Num);

	/*方向包围盒*/
	vector<Eigen::Quaternionf> bboxQ(Extract_Num, Eigen::Quaternionf::Identity());
	vector<Eigen::Vector3f> bboxT(Extract_Num);

	vector<PointT> cp(Extract_Num);
	vector<PointT> pcX(Extract_Num);
	vector<PointT> pcY(Extract_Num);
	vector<PointT> pcZ(Extract_Num);

	for (int i = 0; i < Extract_Num; ++i) {
		// 计算特征值
		compute3DCentroid(*LCCP_Extract_vec[i], pcaCentroid[i]);
		computeCovarianceMatrixNormalized(*LCCP_Extract_vec[i], pcaCentroid[i], convariance[i]);
		eigen_solver[i]=Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f>(convariance[i],Eigen::ComputeEigenvectors);
		eigenVectorsPCA[i] = eigen_solver[i].eigenvectors();
		eigenValuesPCA[i] = eigen_solver[i].eigenvalues();
		//计算方向包围盒
		/*
		 * 计算方向包围盒
		 * whd(0)：宽；whd（1）：高；whd（2）：深度
		 * bboxT[i]：三坐标
		*/ 
		getMinMax3D(*LCCP_Extract_vec[i], min_p[i],max_p[i]);
		c[i] = 0.5f*(min_p[i].getVector3fMap() + max_p[i].getVector3fMap());
		whd[i] = max_p[i].getVector3fMap() - min_p[i].getVector3fMap();
		scale[i] = (whd[i](0) + whd[i](1) + whd[i](2)) / 3;

		
		bboxT[i] = Eigen::Vector3f(c[i]);
	}

	/* 筛选需要的边缘：基于包围盒的尺度以及长宽比
	 *
	 * Edge_vec：		筛选后的边缘的vector
	 * EdgeNum：		筛选后边缘的数目
	 * EdgeCentroid：	筛选后的长宽高vector
	 * EdgeRotation：	筛选后的旋转矩阵
	 * EdgeSize：	筛选后的边缘尺度
	 *
	*/
	vector<PointCloud<PointT>::Ptr> Edge_vec;
	int EdgeNum = 0;
	vector<Eigen::Vector3f> EdgeCentroid;
	vector<Eigen::Quaternionf> EdgeRotation;
	vector<Eigen::Vector3f> EdgeSize;
	for (int i = 0; i < Extract_Num; ++i) {
		// 1st标准：最小尺度不能过大，12.9设为50mm
		if (std::min(whd[i](0), whd[i](1))<10) {
			// 2nd标准：长宽比要大于1.5:1，保证边缘是细长型的
			if (std::max(whd[i](0), whd[i](1)) > 1.5 * std::min(whd[i](0), whd[i](1))) {
					Edge_vec.push_back(LCCP_Extract_vec[i]);
					EdgeNum++;
					EdgeCentroid.push_back(bboxT[i]);
					EdgeRotation.push_back(bboxQ[i]);
					EdgeSize.push_back(whd[i]);
			}
		}
	}
	cout << Extract_Num << " " << EdgeNum << endl;

	// 建立邻域包围盒的vector：用于计算密度梯度
	vector<Eigen::Vector3f> Neighbor1(EdgeNum), Neighbor2(EdgeNum);/* = bboxT[0];*/
	for (int i = 0; i < EdgeNum; ++i) {
		if (EdgeSize[i](0)>EdgeSize[i](1)) {
			float temp1 = EdgeCentroid[i](1) + EdgeSize[i](1);
			float temp2 = EdgeCentroid[i](1) - EdgeSize[i](1);
			Neighbor1[i] = EdgeCentroid[i]; Neighbor1[i](1) = temp1; Neighbor1[i](2) -= 3;	// -3ÎªÁË¸üºÃÓ­ºÏÇúÂÊ£¬Æ½Ãæ¾Í²»ÐèÒªÁË
			Neighbor2[i] = EdgeCentroid[i]; Neighbor2[i](1) = temp2; Neighbor2[i](2) -= 3;
		}
		else {
			float temp1 = EdgeCentroid[i](0) + EdgeSize[i](0);
			float temp2 = EdgeCentroid[i](0) - EdgeSize[i](0);
			Neighbor1[i] = EdgeCentroid[i]; Neighbor1[i](0) = temp1; Neighbor1[i](2) -= 3;
			Neighbor2[i] = EdgeCentroid[i]; Neighbor2[i](0) = temp2; Neighbor2[i](2) -= 3;
		}
	}

	// k-d tree搜索
	// FinalEdge 表示最终的一组变量，Edge是密度梯度之前的一
	vector<PointCloud<PointT>::Ptr> FinalEdge_vec;
	int FinalEdgeNum = 0;
	vector<Eigen::Vector3f> FinalEdgeCentroid;
	vector<Eigen::Quaternionf> FinalEdgeRotation;
	vector<Eigen::Vector3f> FinalEdgeSize;
	vector<Eigen::Vector3f> FinalOrientation;
	for (int i = 0; i < EdgeNum; ++i) {
		vector<int>PointRSearch1, PointRSearch2;
		vector<float>PointRSDistance1,PointRSDistance2;
		float radius = 2.0;
		PointXYZ searchPoint1,searchPoint2;
		searchPoint1.x = Neighbor1[i](0); searchPoint1.y = Neighbor1[i](1); searchPoint1.z = Neighbor1[i](2);
		searchPoint2.x = Neighbor2[i](0); searchPoint2.y = Neighbor2[i](1); searchPoint2.z = Neighbor2[i](2);
		kdtree.radiusSearch(searchPoint1, radius, PointRSearch1, PointRSDistance1);
		kdtree.radiusSearch(searchPoint2, radius, PointRSearch2, PointRSDistance2);
		if (std::max(PointRSearch1.size(), PointRSearch2.size()) > 1.5 * std::min(PointRSearch1.size(), PointRSearch2.size())) {
			FinalEdge_vec.push_back(Edge_vec[i]);
			FinalEdgeNum++;
			FinalEdgeCentroid.push_back(EdgeCentroid[i]);
			FinalEdgeRotation.push_back(EdgeRotation[i]);
			FinalEdgeSize.push_back(EdgeSize[i]);
			if(PointRSearch1.size()>PointRSearch2.size())
				FinalOrientation.push_back(Neighbor1[i]);
			else
				FinalOrientation.push_back(Neighbor2[i]);
		}
	}
	cout << Extract_Num << " " << EdgeNum << " " << FinalEdgeNum << endl;

	// 所有筛选的片段显示
	//PointCloud<PointT>::Ptr Extracts(new pcl::PointCloud<PointT>);					// 所有片段的叠加
	//for (int i = 0; i < FinalEdgeNum; i++)
	//{
	//	*Extracts = *Extracts + *FinalEdge_vec[i];
	//}
	//pcl::visualization::CloudViewer viewer("LCCP聚类结果");//创建viewer对象
	//viewer.showCloud(Extracts);
	//while (!viewer.wasStopped())
	//{
	//}



	/** \brief Add a cube from a set of given model coefficients
	* \bboxT	平移向量
	* \bboxQ    旋转矩阵
	* \whd(0)	宽
	* \whd(1)	¸高
	* \whd(2)	深度
	* id		the cube id/name (default: "cube")
	*/

// 可视化 包围盒
	//visualization::PCLVisualizer OBBviewer;
	//OBBviewer.addPointCloud<PointT>(boundPoints, "Objects");
	//for (int i = 0; i < FinalEdgeNum; i++) {
	//	OBBviewer.addCube(FinalEdgeCentroid[i], FinalEdgeRotation[i], FinalEdgeSize[i](0), FinalEdgeSize[i](1), FinalEdgeSize[i](2), std::to_string(i));
	//	OBBviewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(i));
	//	OBBviewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, std::to_string(i));

	//	OBBviewer.addCube(FinalOrientation[i], FinalEdgeRotation[i], FinalEdgeSize[i](0), FinalEdgeSize[i](1), FinalEdgeSize[i](2), std::to_string(i + 1000));
	//	OBBviewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, std::to_string(i + 1000));
	//	OBBviewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, std::to_string(i + 1000));
	//}

	//while (!OBBviewer.wasStopped())
	//{
	//	OBBviewer.spinOnce(100);
	//}

// 区域生长
/*
	* LabeledPointCloud:带有Label标记的点云
	* LabelNum：Label数量，初始化为FinalEdgeNum的数量，即是边缘的数量
*/
	pcl::PointCloud<pcl::PointXYZL>::Ptr LabeledPointCloud(new pcl::PointCloud<pcl::PointXYZL>);

// 给边缘的x坐标建个unordered_set；顺便将边缘（label赋值）放进去

	unordered_set<float> setx,sety,setz;
	for (int i = 0; i < FinalEdgeNum; ++i) {
		for (int j = 0; j < FinalEdge_vec[i]->size(); ++j) {
			setx.insert(FinalEdge_vec[i]->points[j].x);
			sety.insert(FinalEdge_vec[i]->points[j].y);
			sety.insert(FinalEdge_vec[i]->points[j].z);
			PointXYZL temp;
			temp.x = FinalEdge_vec[i]->points[j].x; temp.y = FinalEdge_vec[i]->points[j].y; temp.z = FinalEdge_vec[i]->points[j].z; temp.label = i + 1;
			LabeledPointCloud->push_back(temp);
		}
	}
	//kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance)

// 把除了边缘之外的其他点放进去
	for (int i = 0; i < Objects->size(); ++i) {
		PointXYZL temp;
		temp.x = Objects->points[i].x;
		temp.y = Objects->points[i].y;
		temp.z = Objects->points[i].z;
		temp.label = 0;
		if( (setx.find(temp.x)==setx.end()) && (sety.find(temp.y) == sety.end()) && (setz.find(temp.z) == setz.end()) )
		// 在存边缘的哈希表中没有找到当前点时，说明它不是边缘点
			LabeledPointCloud->push_back(temp);

	}

	pcl::KdTreeFLANN<pcl::PointXYZL> LabeledKdtree;								// 分割点云的KD
	LabeledKdtree.setInputCloud(LabeledPointCloud);

// 把包围盒的方向的点的label设为对应的label
	for (int i = 0; i < FinalEdgeNum; ++i) {
		PointXYZL SearchCenter; 
		float x, y, z;
		x = FinalEdgeCentroid[i](0);
		y = FinalEdgeCentroid[i](1);
		z = FinalEdgeCentroid[i](2);
		SearchCenter.x = x;
		SearchCenter.y = y;
		SearchCenter.z = z;
		SearchCenter.label = i + 1;

		vector<int>PointRSearch;
		vector<float>PointRSDistance;
		float radius = std::max(FinalEdgeSize[i](0), FinalEdgeSize[i](1));				// 这里max不合理，暂时debug可以
		LabeledKdtree.radiusSearch(SearchCenter, radius, PointRSearch, PointRSDistance);
		for (int j = 0; j < PointRSearch.size(); j++) {
			if (LabeledPointCloud->points[PointRSearch[j]].label == 0) {
				LabeledPointCloud->points[PointRSearch[j]].label = SearchCenter.label;
			}
		}
	}


// 尝试对label为0的点都生长一下
	for (int i = 0; i < LabeledPointCloud->size(); ++i) {
		if (LabeledPointCloud->points[i].label == 0) {
			// 空白点的最近label
			vector<int> nearestLabelIndex;
			vector<float> nearestLabelDistance;
			int nearestLabel = 0;
			int count = 1;
			while (nearestLabel==0)
			{
				// 循环是针对0点周围的最近点进行查找，从最近的1个点开始找
				// 找不到就接着找第2个点，直到出现第count个最近点的label不等于0
				LabeledKdtree.nearestKSearch(LabeledPointCloud->points[i], count, nearestLabelIndex, nearestLabelDistance);
				nearestLabel = LabeledPointCloud->points[nearestLabelIndex[count-1]].label;
				count++;
			}
			LabeledPointCloud->points[i].label = nearestLabel;

		}
	}
	//int maxLabeled = 0;
	//LCCP_S_D(LabeledPointCloud,maxLabeled);	


// 对邻接的label进行合并
// 对所有的label点云建kd树，计算其他点云到当前点云的欧式距离，若小于阈值，则合并两点云，重建树继续查，直到所有遍历
/*
	* SegmentCloud: 初始化为各个Label的点云，最后处理的结果应该是分割结果
	* SegmentPointNum: SegmentCloud中各点云的点数
*/
// 初始化各个SegmentCloud
	vector<PointCloud<PointT>::Ptr> SegmentCloud(FinalEdgeNum);
	vector<int> SegmentPointNum(FinalEdgeNum);
	vector<pcl::KdTreeFLANN<pcl::PointXYZ>> SegementKdTree(FinalEdgeNum);
	for (int i = 0; i < LabeledPointCloud->size(); ++i) {
		SegmentPointNum[LabeledPointCloud->points[i].label-1]++;								// label从1开始的，最大是FinalEdge的size大小
	}
	for (int i = 0; i < SegmentCloud.size(); ++i) {
		SegmentCloud[i] = boost::make_shared <PointCloud<PointT>>();							// 点云对象的智能指针要初始化
		SegmentCloud[i]->height = 1;
		SegmentCloud[i]->width = SegmentPointNum[i];
		SegmentCloud[i]->resize((SegmentCloud[i]->height)*(SegmentCloud[i]->width));
		int k = 0;																				// k是各点云片段的索引
		for (int j = 0; j < LabeledPointCloud->size(); ++j) {
			if ((LabeledPointCloud->points[j].label-1) == i) {
				SegmentCloud[i]->points[k].x = LabeledPointCloud->points[j].x;
				SegmentCloud[i]->points[k].y = LabeledPointCloud->points[j].y;
				SegmentCloud[i]->points[k].z = LabeledPointCloud->points[j].z;
				k = k + 1;
			}
		}
	}

// 建立各个SegmentCloud的K-D tree距离小于阈值就将点云累加
// 初始化：对SegmentCloud中所有点云建立kd树
	for (int i = 0; i < SegmentCloud.size(); ++i) {
		SegementKdTree[i].setInputCloud(SegmentCloud[i]);
	}
// 查找最小距离：计算SegementKdTree[0]对SegementKdTree[1]~SegementKdTree[size-1]的最小距离
	for (int i = 0; i < SegmentCloud.size(); ++i) {
		// 遍历所有label点云，相当于表格的竖排
		for (int j = i+1; j < SegmentCloud.size();) {
			// 遍历当前label点云之后的所有label，计算当前与其后label的最小距离，相当于表格的横排
			float minDistance = INT_MAX;
			for (int k = 0; k < SegmentCloud[j]->size(); ++k) {
				// 遍历当前点云i到其后各个点云j，得到最小距离
				vector<int> nearestIndex;
				vector<float> nearestDistance;
				SegementKdTree[i].nearestKSearch(SegmentCloud[j]->points[k], 1, nearestIndex, nearestDistance);
				minDistance = std::min(minDistance, nearestDistance[0]);
			}
			// 最小距离小于设定阈值，则合并SegementKdTree[i]和SegementKdTree[j]，并且删除SegementKdTree[j]，更新kd树
			if (minDistance < 1) {
				*SegmentCloud[i] = *SegmentCloud[i] + *SegmentCloud[j];
				SegmentCloud.erase(SegmentCloud.begin() + j);

				SegementKdTree.erase(SegementKdTree.begin() + i);
				SegementKdTree.insert(SegementKdTree.begin() + i, pcl::KdTreeFLANN<pcl::PointXYZ>());
				SegementKdTree[i].setInputCloud(SegmentCloud[i]);


				SegementKdTree.erase(SegementKdTree.begin() + j);
				j--;
			}
			j++;
		}
	}

	for (int i = 0; i < SegmentCloud.size(); ++i) {
		// 遍历所有label点云，相当于表格的竖排
		for (int j = i + 1; j < SegmentCloud.size();) {
			// 遍历当前label点云之后的所有label，计算当前与其后label的最小距离，相当于表格的横排
			float minDistance = INT_MAX;
			for (int k = 0; k < SegmentCloud[j]->size(); ++k) {
				// 遍历当前点云i到其后各个点云j，得到最小距离
				vector<int> nearestIndex;
				vector<float> nearestDistance;
				SegementKdTree[i].nearestKSearch(SegmentCloud[j]->points[k], 1, nearestIndex, nearestDistance);
				minDistance = std::min(minDistance, nearestDistance[0]);
			}
			// 最小距离小于设定阈值，则合并SegementKdTree[i]和SegementKdTree[j]，并且删除SegementKdTree[j]，更新kd树
			if (minDistance < 1) {
				*SegmentCloud[i] = *SegmentCloud[i] + *SegmentCloud[j];
				SegmentCloud.erase(SegmentCloud.begin() + j);

				SegementKdTree.erase(SegementKdTree.begin() + i);
				SegementKdTree.insert(SegementKdTree.begin() + i, pcl::KdTreeFLANN<pcl::PointXYZ>());
				SegementKdTree[i].setInputCloud(SegmentCloud[i]);


				SegementKdTree.erase(SegementKdTree.begin() + j);
				j--;
			}
			j++;
		}
	}
	cout <<"合并后的label数量" <<SegmentCloud.size() << endl;

	vector<PointCloud<pcl::PointXYZRGB>::Ptr> ColoredSementCloud(SegmentCloud.size());
	PointCloud<pcl::PointXYZRGB>::Ptr Result(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int i = 0; i < ColoredSementCloud.size(); i++) {
		ColoredSementCloud[i] = boost::make_shared <PointCloud<pcl::PointXYZRGB>>();
		ColoredSementCloud[i]->height = 1;
		ColoredSementCloud[i]->width = SegmentCloud[i]->size();
		ColoredSementCloud[i]->resize((ColoredSementCloud[i]->height)*(ColoredSementCloud[i]->width));
		int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);
		for (int j = 0; j < ColoredSementCloud[i]->size(); j++) {
			ColoredSementCloud[i]->points[j].x = SegmentCloud[i]->points[j].x;
			ColoredSementCloud[i]->points[j].y = SegmentCloud[i]->points[j].y;
			ColoredSementCloud[i]->points[j].z = SegmentCloud[i]->points[j].z;

			ColoredSementCloud[i]->points[j].r = color_R;
			ColoredSementCloud[i]->points[j].g = color_G;
			ColoredSementCloud[i]->points[j].b = color_B;
		}
		*Result = *Result + *ColoredSementCloud[i];
	}

	Display(Result);

	//for (int i = 0; i < SegmentCloud.size(); i++) {
	//	Display(SegmentCloud[i]);
	//}

	//int maxSize = 0, index = 0;
	//for (int i = 0; i < SegmentCloud.size(); i++) {
	//	if (SegmentCloud[i]->size()>maxSize) {
	//		maxSize = SegmentCloud[i]->size();
	//		index = i;
	//	}

	//}
	//Display(SegmentCloud[index]);



	system("pause");
	return 0;
}
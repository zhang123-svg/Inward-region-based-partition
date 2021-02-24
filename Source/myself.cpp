#include "myself.h"

typedef struct tagPOINT_3D
{
	double x;  //mm world coordinate x  
	double y;  //mm world coordinate y  
	double z;  //mm world coordinate z  
	double r;
}POINT_WORLD;

void Load_xyz(PointCloud<PointT>::Ptr &input_cloud_ptr)          //加载数据的函数Load_xyz
{
	/////加载txt数据
	int number_Txt;
	FILE *fp_txt;
	tagPOINT_3D TxtPoint;
	vector<tagPOINT_3D> m_vTxtPoints;
	fp_txt = fopen("/Users/zhangmeng/分割算法zm/点云txt", "r");  //r表示只读打开

	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
		cout << "txt数据加载失败！" << endl;
	number_Txt = m_vTxtPoints.size();
	PointCloud<PointT>::Ptr cloud = input_cloud_ptr;


	// Fill in the cloud data              // 每行有number_Txt个点
	input_cloud_ptr->width = number_Txt;   //width(int) 指定了点云数据中的宽度。//可以指定点云的数量，但是只是对于*无序点云而言*，指定有序点云中，一行点云的数量
	input_cloud_ptr->height = 1;           //height（int）指定了点云数据中的高度 //有序点云：点云行的数量 //无序点云：将height设为1（它的width即为点云的大小），以此来区分点云是否有序
	input_cloud_ptr->is_dense = false;     //is_dense(bool) 指定点云中所有数据都是有限的（true），还有其他一些不是有限的，他们的XYZ值可能包含inf/NaN这样的值（false）
	input_cloud_ptr->points.resize(input_cloud_ptr->width * input_cloud_ptr->height);  
 	//points（std：：vector） points是存储类型为PointT的点的向量


	for (size_t i = 0; i < input_cloud_ptr->points.size(); ++i)
	{
		input_cloud_ptr->points[i].x = m_vTxtPoints[i].x;
		input_cloud_ptr->points[i].y = m_vTxtPoints[i].y;
		input_cloud_ptr->points[i].z = m_vTxtPoints[i].z;
	}
	pcl::io::savePCDFileASCII("box_zm.pcd", *input_cloud_ptr);  //向pcd文件写入点云数据
	std::cerr << "Saved " << input_cloud_ptr->points.size() << " data points to box_zm.pcd." << std::endl;  //txt2pcd.pcd
}

PointCloud<PointT>::Ptr RemovePlain(PointCloud<PointT>::Ptr &input_cloud_ptr) {   //RemovePlain函数  高于阈值的点去掉
	/*
	 *  HeightThresh： 高度阈值，暂时设为640，高于此值去掉   去掉桌子上，只想要桌子上面的东西
	*/
	int HeightThresh = 640;
	PointCloud<PointT>::Ptr Objects(new pcl::PointCloud<PointT>);
		for (int i = 0; i < input_cloud_ptr->size(); ++i) {
			if (input_cloud_ptr->points[i].z < HeightThresh)
				Objects->push_back(input_cloud_ptr->points[i]);
		}
		return Objects;


}

// 生成随机数用于给点云染色
#define Random(x) (rand() % x)   //rand() % x  生成随机数函数



void voxel_filter(PointCloud<PointT>::Ptr & input_cloud_ptr,float leaf_size)  //voxel_filter函数
{
	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud(input_cloud_ptr);	//设置输入点云为input_cloud_ptr指针指向的
	vg.setLeafSize(leaf_size, leaf_size, leaf_size);	
	vg.filter(*input_cloud_ptr);		        //执行滤波，结果覆盖input_cloud_ptr

}

void Display(PointCloud<PointT>::Ptr cloud) 
{
	pcl::visualization::CloudViewer viewer("View");//创建viewer对象
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
}

void Display(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::visualization::CloudViewer viewer("View");//创建viewer对象
	viewer.showCloud(cloud);
	while (!viewer.wasStopped())
	{
	}
}

void LCCP_S_D(PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud,int & label_max2)      //(LCCP基于局部凸性的分割方法，显示结果)
{


	for (int i = 0; i< lccp_labeled_cloud->size(); i++) {
		if (lccp_labeled_cloud->points[i].label>label_max2)
			label_max2 = lccp_labeled_cloud->points[i].label;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	ColoredCloud2->height = 1;
	ColoredCloud2->width = lccp_labeled_cloud->size();
	ColoredCloud2->resize(lccp_labeled_cloud->size());
	for (int i = 0; i < label_max2; i++) {									// 大循环是label
		int color_R, color_G, color_B;
		// 边缘随机颜色，中间的东西都是白色
		if (i != 0) {
			color_R = Random(255);
			color_G = Random(255);
			color_B = Random(255);
		}
		else {
			color_R = 1;
			color_G = 1;
			color_B = 1;
		}
		for (int j = 0; j < lccp_labeled_cloud->size(); j++) {				// 小循环是点云的遍历
			if (lccp_labeled_cloud->points[j].label == i) {
				ColoredCloud2->points[j].x = lccp_labeled_cloud->points[j].x;
				ColoredCloud2->points[j].y = lccp_labeled_cloud->points[j].y;
				ColoredCloud2->points[j].z = lccp_labeled_cloud->points[j].z;
				ColoredCloud2->points[j].r = color_R;
				ColoredCloud2->points[j].g = color_G;
				ColoredCloud2->points[j].b = color_B;
			}
		}
	}


	//pcl::io::savePCDFileASCII("分割结果Label.pcd", *lccp_labeled_cloud);


	cout << "XYZL的最大Label是" << label_max2 << endl;
	cout << endl;


	 //显示LCCP的结果
	pcl::visualization::CloudViewer viewer("Cloud Viewer2");//创建viewer对象
	viewer.showCloud(ColoredCloud2);
	while (!viewer.wasStopped())
	{
	}

}

void LCCP_Extract(PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud, vector<PointCloud<PointT>::Ptr> & LCCP_Extract_vec, int & Extract_Num) {
	
	int label_matrix[10000] = { 0 };			//	存储各label的对应点数
	int edge[10000] = { 0 };					//  存储edge的label数组
											/* 遍历点云，将各个label对应的点云数目保存进label_matrix数组中 */
	for (int i = 0; i < lccp_labeled_cloud->size(); i++)
	{
		label_matrix[lccp_labeled_cloud->points[i].label]++;
	}

	int e_indice = 0;							// 计算边缘点云的索引
	cout << endl;
	for (int j = 0; j < 10000; j++)				//筛选点云数目大雨2500的label
	{
		if (label_matrix[j]>5/* && label_matrix[j]<100*/)		// 上限要过滤掉拐弯的线，下限要去掉过短的线（下限没什么意义了）
		{
			edge[e_indice] = j;
			//cout << "片段" << e_indice << "的label为" << edge[e_indice] << "该label中点数为" << label_matrix[j] << endl;
			e_indice = e_indice + 1;
		}
	}


	Extract_Num = e_indice;
	vector <PointCloud<PointT>::Ptr> vec(e_indice);							// vec是片段的容器，最后要将vec赋给LCCP_Extract_vec引用

	int n = vec.size();
	cout <<"共计"<< n<<"个边缘片段" << endl;
	for (unsigned i = 0; i < e_indice; i = i + 1)										// 1st循环：每个片段
	{

		vec[i] = boost::make_shared <PointCloud<PointT>>();							// 点云对象的智能指针要初始化
		vec[i]->height = 1;
		vec[i]->width = label_matrix[edge[i]];
		vec[i]->resize((vec[i]->height)*(vec[i]->width));
		int k = 0;																	// k是各点云片段的索引
		for (int j = 0; j < lccp_labeled_cloud->size(); j = j + 1)						// 2nd循环：对带有label的点云进行遍历
		{
			if (lccp_labeled_cloud->points[j].label == edge[i])					// 3rd判断：若2nd循环遇到的点的label与当前edge数组中存的label一致，则对当前片段进行写入
			{
				vec[i]->points[k].x = lccp_labeled_cloud->points[j].x;
				vec[i]->points[k].y = lccp_labeled_cloud->points[j].y;
				vec[i]->points[k].z = lccp_labeled_cloud->points[j].z;
				k = k + 1;
			}

		}

	}

	LCCP_Extract_vec = vec;															// 赋值给引用，返回给主函数



																					// 点云相加时要注意是指向的对象相加，不要误以为是指针相加

	PointCloud<PointT>::Ptr Extracts(new pcl::PointCloud<PointT>);					// 所有片段的叠加
	for (int i = 0; i < Extract_Num; i++)
	{
		*Extracts = *Extracts + *vec[i];
	}


	//pcl::visualization::CloudViewer viewer("LCCP聚类结果");        //创建viewer对象
	//viewer.showCloud(Extracts);
	//while (!viewer.wasStopped())
	//{
	//}
}





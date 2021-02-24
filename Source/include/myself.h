#ifndef MYSELF_H
#define MYSELF_H

#include <sys/stat.h>
#include <stdio.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>  
#include <pcl/impl/point_types.hpp>
#include <stdlib.h>    
#include <cmath>    
#include <limits.h>    
#include <boost/format.hpp>    
#include <fstream>   
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/console/parse.h>    
#include <pcl/io/pcd_io.h>    
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/visualization/point_cloud_color_handlers.h>    
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/filters/passthrough.h>    
#include <pcl/segmentation/supervoxel_clustering.h>    
#include <pcl/segmentation/lccp_segmentation.h>   
#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include <time.h>
#include <vector>
#include <Winsock2.h>     //win中
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <pcl/features/boundary.h>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <pcl/features/eigen.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/feature.h>
#include <pcl/features/boundary.h>
#include <unordered_set>

#pragma comment(lib,"ws2_32.lib")


typedef pcl::PointXYZ PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
using namespace pcl;
using std::vector;
using std::string;
using std::unordered_set;

void Load_xyz(PointCloud<PointT>::Ptr &);
PointCloud<PointT>::Ptr RemovePlain(PointCloud<PointT>::Ptr &);
void voxel_filter(PointCloud<PointT>::Ptr & input_cloud_ptr,float leaf_size);
void Display(PointCloud<PointT>::Ptr cloud);
void Display(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void LCCP_S_D(PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud,int &);
void LCCP_Extract(PointCloud<pcl::PointXYZL>::Ptr, vector<PointCloud<PointT>::Ptr> &, int &);
#endif

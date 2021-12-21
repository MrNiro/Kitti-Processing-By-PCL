#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <io.h>
#include <time.h>

using namespace std;
using namespace pcl;
typedef PointXYZI PointT;

pcl::PointCloud<PointT>::Ptr ground_cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr non_ground_cloud(new pcl::PointCloud<PointT>());

float depth_cal(PointCloud<PointT>::Ptr c, pcl::octree::OctreePointCloudSearch<PointT> *oc, PointT point, std::vector<int> &PointIdx)
{
	if (oc->voxelSearch(point, PointIdx))			//执行体素近邻搜索
	{
		float min_z = 10.0, max_z = -10.0;
		for (vector<int>::const_iterator idx = PointIdx.begin(); idx != PointIdx.end(); idx++)
		{
			if (c->points[*idx].z < min_z)
				min_z = c->points[*idx].z;
			if (c->points[*idx].z > max_z)
				max_z = c->points[*idx].z;
		}
		return max_z - min_z;
	}
	return 0;
}

void voxel_grid_filter(PointCloud<PointT>::Ptr c, float size = 0.1, int silent = 1)
{
	pcl::VoxelGrid<PointT> vgf;
	vgf.setInputCloud(c);
	vgf.setLeafSize(size, size, size);					//set the filter size
	vgf.filter(*c);

	if (silent == 0)
		std::cout << "Remain "
		<< c->points.size()
		<< " data points after voxel grid filter"
		<< std::endl;
}

void sor_filter(PointCloud<PointT>::Ptr c, int meanK, float thresh, int silent = 1)
{
	pcl::StatisticalOutlierRemoval <PointT> sor;
	sor.setInputCloud(c);
	sor.setMeanK(meanK);
	sor.setStddevMulThresh(thresh);
	sor.filter(*c);

	if (silent == 0)
		std::cout << "Remain "
		<< c->points.size()
		<< " data points after SOR"
		<< std::endl;
}

PointCloud<PointT>::Ptr pass_through(PointCloud<PointT>::Ptr row_c, string field, float m, float n, int silent = 0, int rowclear = 1)
{
	pcl::PointCloud<PointT>::Ptr c(new pcl::PointCloud<PointT>());
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(row_c);
	pass.setFilterFieldName(field);
	pass.setFilterLimits(m, n);
	pass.filter(*c);
	if (silent == 0)
		std::cout << "Remain "
		<< c->points.size()
		<< " data points after passThrough"
		<< std::endl;
	if (rowclear == 1)
		row_c->clear();
	return c;
}

void height_seg(PointCloud<PointT>::Ptr c, int silent = 1)
{
	ground_cloud = pass_through(c, "z", -5, -1, 1, 0);
	non_ground_cloud = pass_through(c, "z", -1, 5, 1, 0);

	if (silent == 0)
		std::cout << "Remain "
		<< non_ground_cloud->points.size() << " non-ground data points, "
		<< ground_cloud->points.size() << " ground data points "
		<< "after height segmentation"
		//<< min_z + height << ")"
		<< std::endl;
}

void pmf_filter(PointCloud<PointT>::Ptr c, float height = 0.1f, int silent = 1)
{
	pcl::PointIndicesPtr ground(new pcl::PointIndices);
	pcl::ProgressiveMorphologicalFilter<PointT> pmf;
	pmf.setInputCloud(c);
	pmf.setMaxWindowSize(3);
	//pmf.setCellSize(0.5f);
	pmf.setSlope(0.5f);
	pmf.setInitialDistance(height);			//地面起伏越大，该值须设为越大
	pmf.setMaxDistance(1.0f);
	pmf.extract(ground->indices);

	PointCloud<PointT>::Ptr new_ground(new PointCloud<PointT>);
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(c);
	extract.setIndices(ground);
	extract.setNegative(true);
	extract.filter(*new_ground);
	extract.setNegative(false);
	extract.filter(*ground_cloud);

	*non_ground_cloud += *new_ground;
	if (silent == 0)
		std::cout << "Remain "
		<< non_ground_cloud->points.size() << " non-ground data points, "
		<< ground_cloud->points.size() << " ground data points after pmf"
		<< std::endl;
}

void ground_filter(PointCloud<PointT>::Ptr c)
{
	clock_t start, end;
	ground_cloud->clear();
	non_ground_cloud->clear();

	cout << "\nGround filter started!" << endl;
	start = clock();

	c = pass_through(c, "x", -15, 15, 1, 1);
	c = pass_through(c, "y", -20, 20, 1, 1);

	pcl::octree::OctreePointCloudSearch<PointT> oc_search(0.3);
	oc_search.setInputCloud(c);
	oc_search.addPointsFromInputCloud();

	height_seg(c, 1);
	//ground_cloud = pass_through(ground_cloud, "intensity", 0.3, 1.0);
	voxel_grid_filter(ground_cloud, 0.3, 1);
	pmf_filter(ground_cloud, 0.1f, 1);
	//sor_filter(ground_cloud, 5, 1.0, 1);

	int row_size = ground_cloud->size();
	for (int i = 0; i < row_size; i++)					//根据当前地面点反向查找
	{
		std::vector<int> PointIdx, PointIdx_H;
		PointT point = ground_cloud->points[i];
		if (depth_cal(c, &oc_search, point, PointIdx) > 0.1)
		{
			point.z += 0.2;
			if (oc_search.voxelSearch(point, PointIdx_H))
				continue;
		}
		for (int idx = 0; idx < PointIdx.size(); idx++)
			ground_cloud->push_back(c->points[PointIdx[idx]]);
	}
	sor_filter(ground_cloud, 10, 1.0, 1);

	/*row_size = non_ground_cloud->size();
	for (int i = 0; i < row_size; i++)
	{
		std::vector<int> PointIdx;
		PointT point = non_ground_cloud->points[i];
		if (oc_search.voxelSearch(point, PointIdx))
		{
			for (int idx = 0; idx < PointIdx.size(); idx++)
				non_ground_cloud->push_back(cloud->points[PointIdx[idx]]);
		}
	}*/

	end = clock();
	std::cout << "Remain "
		<< non_ground_cloud->points.size() << " non-ground data points, "
		<< ground_cloud->points.size() << " ground data points after Ground filter"
		<< std::endl;
	cout << "Ground filter finished! Running time: " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
}

void fog_filter(PointCloud<PointT>::Ptr c)
{
	float min_intensity = 0.1, max_intensity = 0;
	for (int i = 0; i < c->size(); i++)
		if (c->points[i].intensity < min_intensity)
			min_intensity = c->points[i].intensity;

	for (int i = 0; i < c->size(); i++)
	{
		c->points[i].intensity = (c->points[i].intensity - min_intensity) / 0.1 + min_intensity;
		if (c->points[i].intensity > max_intensity)
			max_intensity = c->points[i].intensity;
	}
	for (int i = 0; i < c->size(); i++)
	{
		c->points[i].intensity = c->points[i].intensity / max_intensity;
	}
}

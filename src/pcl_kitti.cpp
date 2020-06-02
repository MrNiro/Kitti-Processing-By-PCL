/*
***********************************************
1.代码须运行在已完成VS+PCL配置的环境下（在VS2017 + PCL 1.9.1测试通过）
2.代码中包含了整个项目过程用到的所有函数，但最终处理流程中并没有全部用到，包括文件处理及一些优化尝试。考虑到未来可能的用途，仍然予以保留。
3.pipeline函数对所有数据进行了从预处理到目标提取的流程
4.joint_cloud函数对同一数据集点云进行拼接。（必须先用kitti_oxt.py对oxt中文件进行处理得到相应的move_step.txt文件）
5.data_processing对拼接后的点云进行后续处理，得到最终的提取结果点云
***********************************************
*/

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <io.h>
#include <time.h>

using namespace std;
using namespace pcl;
typedef PointXYZI PointT;

string All_file[500];
float move_rotate[500][6];
int line_id = 0;

pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr ground_cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr non_ground_cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr xy_cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr bound_cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr pole_cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr tag_cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr track_cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr my_cloud(new pcl::PointCloud<PointT>());


bool comp(int a, int b)
{
	return a > b;
}

bool load_move_step(string filename)
{
	ifstream file(filename, ios::in);
	if (!file)
	{
		cout << filename << "is not exist!" << endl;
		return false;
	}

	std::vector<float> step;
	float t = 0;
	while (file >> t)
		step.push_back(t);

	int line = 0, pos = 0;
	for (int i = 0; i < step.size(); i++)
	{
		move_rotate[line][pos] = step[i];
		pos++;
		if (pos == 6)
		{
			pos = 0;
			line++;
		}
	}

	file.close();
	return true;
}

int get_filename(const char* to_search, int out_name = 0)
{
	struct _finddata_t fileinfo;
	intptr_t handle;
	handle = _findfirst(to_search, &fileinfo);
	All_file[0] = fileinfo.name;
	int num = 1;
	while (_findnext(handle, &fileinfo) == 0)
	{
		All_file[num] = fileinfo.name;
		if (out_name)
			cout << All_file[num] << endl;
		num++;
	}
	_findclose(handle);
	return num;
}

void load_bin_data(PointCloud<PointT>::Ptr c, string path, string filename)
{
	string infile = path + filename;
	fstream input(infile.c_str(), ios::in | ios::binary);
	if (!input.good()) {
		cerr << "Could not read file: " << infile << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);

	c->clear();
	for (int i = 0; input.good() && !input.eof(); i++) {
		PointT point;
		input.read((char *)&point.x, 3 * sizeof(float));
		input.read((char *)&point.intensity, sizeof(float));
		c->push_back(point);
	}
	input.close();

	cout << "Loaded " << c->size()
		<< " data points from " << filename << endl;
}

void load_pcd_data(PointCloud<PointT>::Ptr c, string path, string filename)
{
	c->clear();
	string infile = path + filename;
	cout << "Loading " << filename << " now..." << endl;
	if (pcl::io::loadPCDFile(infile, *c) < 0)
	{
		std::cout << "Error loading point cloud." << std::endl;
	}
	cout << "Loaded " << c->size()
		<< " data points from " << infile << endl;
}

void save_pcd_data_bin(PointCloud<PointT>::Ptr c, string path, string filename)
{
	pcl::io::savePCDFileBinary(path + filename, *c);
	cout << "Saved " << c->size() << " data points to " << path + filename << endl;
}

void add_line(PointCloud<PointT>::Ptr c, PointT v1, PointT v2)
{
	double step_x = (v2.x - v1.x) / 100;
	double step_y = (v2.y - v1.y) / 100;
	double step_z = (v2.z - v1.z) / 100;

	PointT point = v1;
	point.intensity = 0.001;
	c->push_back(point);
	for (int i = 0; i < 100; i++)
	{
		point.x += step_x;
		point.y += step_y;
		point.z += step_z;
		c->push_back(point);
	}
}

void add_cube_for_pole(PointCloud<PointT>::Ptr c, vector<double> &extremum)
{
	PointT v[8];
	int pos = 0;

	for (int i = 0; i < 2; i++)
		for (int j = 2; j < 4; j++)
			for (int k = 4; k < 6; k++)
			{
				v[pos].x = extremum[i];
				v[pos].y = extremum[j];
				v[pos].z = extremum[k];
				pos++;
			}

	for (int i = 0; i < 8; i++)
		for (int j = i + 1; j < 8; j++)
		{
			if (v[i].x == v[j].x)
			{
				if (v[i].y == v[j].y)
					add_line(c, v[i], v[j]);
				else if (v[i].z == v[j].z)
					add_line(c, v[i], v[j]);
			}
			else if (v[i].y == v[j].y)
			{
				if (v[i].z == v[j].z)
					add_line(c, v[i], v[j]);
			}
		}
}

/*
void add_cube_for_pole_visual(pcl::visualization::PCLVisualizer& viewer)
{
	PointT v[8];
	int pos = 0;
	viewer.setBackgroundColor(1.0, 1.0, 1.0);
	for (int i = 0; i < 2; i++)
		for (int j = 2; j < 4; j++)
			for (int k = 4; k < 6; k++)
			{
				v[pos].x = extremum[i];
				v[pos].y = extremum[j];
				v[pos].z = extremum[k];
				pos++;
			}

	for (int i = 0; i < 8; i++)
		for (int j = i + 1; j < 8; j++)
		{
			if (v[i].x == v[j].x)
			{
				if (v[i].y == v[j].y || v[i].z == v[j].z)
					viewer.addLine(v[i], v[j], 1.0, 0.0, 0.0, "line" + to_string(line_id++));
			}
			else if (v[i].y == v[j].y)
			{
				if (v[i].z == v[j].z)
					viewer.addLine(v[i], v[j], 1.0, 0.0, 0.0, "line" + to_string(line_id++));
			}
		}
}
*/

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 1.0, 1.0);
}

void cloud_visualization(PointCloud<PointT>::Ptr c)
{
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(c);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");
}

double circle_fitting(PointCloud<PointT>::Ptr c, double& center_x, double& center_y, double& error)
{
	double sum_x = 0.0f, sum_y = 0.0f;
	double sum_x2 = 0.0f, sum_y2 = 0.0f;
	double sum_x3 = 0.0f, sum_y3 = 0.0f;
	double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

	int N = c->points.size();
	for (int i = 0; i < N; i++)
	{
		double x = c->points[i].x;
		double y = c->points[i].y;

		sum_x += x;
		sum_y += y;
		sum_x2 += x * x;
		sum_y2 += y * y;
		sum_x3 += x * x * x;
		sum_y3 += y * y * y;
		sum_xy += x * y;
		sum_x1y2 += x * y * y;
		sum_x2y1 += x * x * y;
	}

	double C, D, E, G, H;
	double aa, bb, cc;

	C = N * sum_x2 - sum_x * sum_x;
	D = N * sum_xy - sum_x * sum_y;
	E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
	G = N * sum_y2 - sum_y * sum_y;
	H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;

	aa = (H * D - E * G) / (C * G - D * D);
	bb = (H * C - E * D) / (D * D - G * C);
	cc = -(aa * sum_x + bb * sum_y + sum_x2 + sum_y2) / N;

	center_x = aa / (-2);
	center_y = bb / (-2);
	double radius = sqrt(aa * aa + bb * bb - 4 * cc) / 2;


	for (int i = 0; i < N; i++)
	{
		double x = c->points[i].x;
		double y = c->points[i].y;

		error += abs(sqrt(pow(x - center_x, 2) + pow(y - center_y, 2)) - radius);
	}

	/*for (float x = center_x - radius; x <= center_x + radius; x += 0.01)
	{
		PointT p;
		p.x = x;
		p.z = 5;
		p.intensity = 0.01;
		p.y = center_y + sqrt(pow(radius, 2) - pow(x - center_x, 2));
		pole_cloud->push_back(p);
		p.y = center_y - sqrt(pow(radius, 2) - pow(x - center_x, 2));
		pole_cloud->push_back(p);
	}*/

	error /= N;
	return radius;
}

float line_fitting(PointCloud<PointT>::Ptr c, float &a, float &b)
{
	double sum_x = 0.0f, sum_y = 0.0f;
	double sum_xy = 0.0f, sum_x2 = 0.0f;
	double sum_z = 0.0f;

	int N = c->points.size();
	for (int i = 0; i < N; i++)
	{
		double x = c->points[i].x;
		double y = c->points[i].y;
		sum_x += x;
		sum_y += y;
		sum_x2 += x * x;
		sum_xy += x * y;
		sum_z += c->points[i].z;
	}
	double avg_x = sum_x / N, avg_y = sum_y / N;
	double avg_xy = sum_xy / N, avg_x2 = sum_x2 / N;

	b = (avg_xy - avg_x * avg_y) / (avg_x2 - pow(avg_x, 2));
	a = avg_y - b * avg_x;

	float error = 0;
	for (int i = 0; i < N; i++)
	{
		double x = c->points[i].x;
		double y = c->points[i].y;
		error += abs(y - b * x - a) / sqrt(1 + b * b);
	}
	return error / N;
}

/*
void poly_intensity_fitting(PointCloud<PointT>::Ptr c, vector<float>& a, int m, vector<float>& dt)
{
	
	int N = c->size();
	if (N > 100000)
	{
		PointCloud<PointT>::Ptr temp(new PointCloud<PointT>);
		pcl::copyPointCloud(*c, *temp);
		voxel_grid_filter(temp, 0.1);
		N = temp->size();
	}
	m = (m > 20) ? 20 : m;
	float z = 0.0, p, c, g, q, d1, d2, s[20], t[20], b[20];
	float x[100000] = { 0.0 };
	float y[100000] = { 0.0 };

	for (int i = 0; i < m; i++) 
		a.push_back(0.0);	
	
	for (int i = 0; i < N; i++)
	{
		x[i]
		z += x[i] / (1.0*n);
	}
	b[0] = 1.0; d1 = 1.0*n; p = 0.0; c = 0.0;
	for (i = 0; i <= n - 1; i++)
	{
		p = p + (x[i] - z); c = c + y[i];
	}
	c = c / d1; p = p / d1;
	a[0] = c * b[0];
	if (m > 1)
	{
		t[1] = 1.0; t[0] = -p;
		d2 = 0.0; c = 0.0; g = 0.0;
		for (i = 0; i <= n - 1; i++)
		{
			q = x[i] - z - p; d2 = d2 + q * q;
			c = c + y[i] * q;
			g = g + (x[i] - z)*q*q;
		}
		c = c / d2; p = g / d2; q = d2 / d1;
		d1 = d2;
		a[1] = c * t[1]; a[0] = c * t[0] + a[0];
	}
	for (j = 2; j <= m - 1; j++)
	{
		s[j] = t[j - 1];
		s[j - 1] = -p * t[j - 1] + t[j - 2];
		if (j >= 3)
			for (k = j - 2; k >= 1; k--)
				s[k] = -p * t[k] + t[k - 1] - q * b[k];
		s[0] = -p * t[0] - q * b[0];
		d2 = 0.0; c = 0.0; g = 0.0;
		for (i = 0; i <= n - 1; i++)
		{
			q = s[j];
			for (k = j - 1; k >= 0; k--)
				q = q * (x[i] - z) + s[k];
			d2 = d2 + q * q; c = c + y[i] * q;
			g = g + (x[i] - z)*q*q;
		}
		c = c / d2; p = g / d2; q = d2 / d1;
		d1 = d2;
		a[j] = c * s[j]; t[j] = s[j];
		for (k = j - 1; k >= 0; k--)
		{
			a[k] = c * s[k] + a[k];
			b[k] = t[k]; t[k] = s[k];
		}
	}
	dt[0] = 0.0; dt[1] = 0.0; dt[2] = 0.0;
	for (i = 0; i <= n - 1; i++)
	{
		q = a[m - 1];
		for (k = m - 2; k >= 0; k--)
			q = a[k] + q * (x[i] - z);
		p = q - y[i];
		if (fabs(p) > dt[2]) dt[2] = fabs(p);
		dt[0] = dt[0] + p * p;
		dt[1] = dt[1] + fabs(p);
	}
	return;
}
*/

void XOY_projection(PointCloud<PointT>::Ptr c)
{
	xy_cloud->clear();
	for (size_t i = 0; i < c->points.size(); i++)
	{
		PointT point;
		point = c->points[i];
		point.z = 0;
		xy_cloud->push_back(point);
	}
}

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

	PointT minPt, maxPt;
	getMinMax3D(*ground_cloud, minPt, maxPt);

	row_size = non_ground_cloud->size();
	for (int i = 0; i < row_size; i++)
	{
		std::vector<int> PointIdx;
		PointT point = non_ground_cloud->points[i];
		if (oc_search.voxelSearch(point, PointIdx))
		{
			for (int idx = 0; idx < PointIdx.size(); idx++)
				if(c->points[PointIdx[idx]].z - minPt.z > 0.05)
					non_ground_cloud->push_back(c->points[PointIdx[idx]]);
		}
	}

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

void modified_intensity(PointCloud<PointT>::Ptr c)
{
	cloud->clear();
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(c);

	for (int i = 0; i < c->size(); i++)
	{
		vector<int> idx;
		vector<float> sqr_dis;
		PointT p = c->points[i];
		int neighbor = tree->radiusSearch(p, 0.1, idx, sqr_dis);
		if (neighbor <= 10)
			continue;
		//cout << neighbor << endl;

		float min_delta = RAND_MAX;
		int min_idx = 0;
		for (int j = 0; j < idx.size(); j++)
		{
			float sum_delta = 0;
			for (int k = 0; k < idx.size(); k++)
			{
				if (j == k)
					continue;
				//cout << cloud->points[idx[j]].intensity << ", " << cloud->points[idx[k]].intensity << endl;
				sum_delta += abs(cloud->points[idx[j]].intensity - cloud->points[idx[k]].intensity);
			}
			if (sum_delta < min_delta)
			{
				min_delta = sum_delta;
				min_idx = idx[j];
			}
		}
		p.intensity = c->points[min_idx].intensity;
		cloud->push_back(p);
	}
}

vector<PointCloud<PointT>::Ptr>* euclidean_cluster(PointCloud<PointT>::Ptr c, float dis = 0.5, int min_size = 50, int ifwrite = 0)
{
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(c);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;

	ec.setClusterTolerance(dis);
	ec.setMinClusterSize(min_size);
	ec.setMaxClusterSize(10000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(c);
	ec.extract(cluster_indices);

	//cout << "Extracted " << cluster_indices.size() << " cluster" << endl;
	vector<pcl::PointCloud<PointT>::Ptr> *all_cluster(new vector<pcl::PointCloud<PointT>::Ptr>);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(c->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		//std::cout << "Cluster " << j << ": " << cloud_cluster->points.size() << " data points." << std::endl;
		//cloud_visualization(cloud_cluster);

		if (ifwrite == 1)
		{
			string outfile = "cluster/cluster_" + to_string(j) + ".pcd";
			pcl::PCDWriter writer;
			writer.write(outfile, *cloud_cluster);
		}

		all_cluster->push_back(cloud_cluster);
		j++;
	}
	return all_cluster;
}

void cal_smooth(PointCloud<PointT>::Ptr c, PointCloud<PointT>::Ptr new_c, pcl::search::KdTree<PointT>::Ptr tree, int idx, bool *idx_judge)
{
	if (idx_judge[idx])
		return;
	idx_judge[idx] = true;

	float radius = 0.3;

	std::vector<int> PointIdx;
	std::vector<float> square_dis;

	PointT point = c->points[idx];
	int neighbor = tree->radiusSearch(point, radius, PointIdx, square_dis);
	if (neighbor == 0)
		return;

	float sum_dis = 0;
	for (vector<float>::const_iterator pit = square_dis.begin(); pit != square_dis.end(); pit++)
		sum_dis += *pit;
	float smooth = sqrt(sum_dis) / neighbor / radius;
	//cout << smooth << endl;

	if (smooth > 0 && smooth < 0.15)
	{
		//idx_judge[idx] = true;
		new_c->push_back(point);
		//c->erase(c->begin() + idx);
		for (vector<int>::const_iterator pit = PointIdx.begin(); pit != PointIdx.end(); pit++)
			if (!idx_judge[*pit])
				cal_smooth(c, new_c, tree, *pit, idx_judge);
	}
}

vector<PointCloud<PointT>::Ptr>* re_cluster(PointCloud<PointT>::Ptr c)
{
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(c);
	vector<PointCloud<PointT>::Ptr>* all_cluster(new vector<PointCloud<PointT>::Ptr>);

	//pcl::visualization::CloudViewer viewer("Viewer");
	//viewer.runOnVisualizationThreadOnce(viewerOneOff);

	bool idx_judge[10000] = { false };
	int row_size = c->size();
	for (int i = 0; i < row_size; i++)
	{
		if (idx_judge[i])
			continue;
		PointCloud<PointT>::Ptr new_c(new PointCloud<PointT>);
		cal_smooth(c, new_c, tree, i, idx_judge);

		if (!new_c->empty() && new_c->size() > 30)
		{
			//viewer.showCloud(new_c);
			//system("pause");
			all_cluster->push_back(new_c);
		}
	}
	return all_cluster;
}

bool pole_fitting(PointCloud<PointT>::Ptr row_c)
{
	PointT minPt, maxPt;
	getMinMax3D(*row_c, minPt, maxPt);

	double min_x = minPt.x, max_x = maxPt.x;
	double min_y = minPt.y, max_y = maxPt.y;
	double min_z = minPt.z, max_z = maxPt.z;

	double height = max_z - min_z;
	if (height < 1.5 || max_x - min_x > 3 || max_y - min_y > 3)
		return false;

	double step = height / 8, step_z = min_z + 0.1;

	vector<double> center_x;
	vector<double> center_y;
	double radius = 0.0;
	double error = 0.0;
	int count = 0;
	while (step_z < max_z)
	{
		PointCloud<PointT>::Ptr c = pass_through(row_c, "z", step_z, step_z + step, 1, 0);
		step_z += step;

		if (c->size() > 8)
			sor_filter(c, 3, 2.0, 1);

		double e = 0.0, c_x, c_y;
		float r = circle_fitting(c, c_x, c_y, e);
		radius += r;
		error += e;
		center_x.push_back(c_x);
		center_y.push_back(c_y);

		count++;
	}
	if (count)
	{
		float avg_r = radius / count;
		float avg_e = error / count;
		//float bias = 0.0;

		//cloud_visualization(row_c);

		if (avg_r < 0.5 && avg_r > 0 && avg_e < 0.05)
		{
			//cout << "radis = " << avg_r << ", error = " << avg_e << endl;
			//cout << "*****averagre pole radius = " << avg_r << endl;
			/*extremum.clear();
			extremum.push_back(min_x);
			extremum.push_back(max_x);
			extremum.push_back(min_y);
			extremum.push_back(max_y);
			extremum.push_back(min_z);
			extremum.push_back(max_z);*/
			return true;
		}
	}
	return false;
}

void pole_detecting(PointCloud<PointT>::Ptr c)
{
	clock_t start, end;
	cout << "\nPole detector started" << endl;
	start = clock();

	//pole_cloud->clear();
	pole_cloud = pass_through(c, "intensity", 0.1, 1.0, 1, 0);
	
	voxel_grid_filter(pole_cloud, 0.1, 1);

	vector<PointCloud<PointT>::Ptr> *non_ground_clusters = euclidean_cluster(pole_cloud, 0.3, 50);
	pole_cloud->clear();
	//pcl::visualization::CloudViewer viewer("Viewer");
	//viewer.runOnVisualizationThreadOnce(viewerOneOff);

	int count = 0;
	for (int i = 0; i < non_ground_clusters->size(); i++)
	{
		PointCloud<PointT>::Ptr cluster = (*non_ground_clusters)[i];
		if (pole_fitting(cluster))
		{
			//add_cube_for_pole(pole_cloud);
			*pole_cloud += *cluster;
			count++;
			//cloud_visualization(pole_cloud);
		}
		else if (cluster->size() > 100)
		{
			//viewer.showCloud(cluster);
			//system("pause");
			std::vector<PointCloud<PointT>::Ptr>* new_clusters = re_cluster(cluster);
			for (int j = 0; j < new_clusters->size(); j++)
			{
				//viewer.showCloud((*new_clusters)[j]);
				//system("pause");
				if (pole_fitting((*new_clusters)[j]))
				{
					//add_cube_for_pole(pole_cloud);
					*pole_cloud += *(*new_clusters)[j];
					count++;
				}
			}
		}
	}
	cout << count << " pole detected!" << endl;

	end = clock();
	cout << "Pole detecting finished! Running time: " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
}

void bound_depth_fitting(PointCloud<PointT>::Ptr c)
{
	bound_cloud->clear();

	typedef std::vector<PointT, Eigen::aligned_allocator<PointT>> AlignedPointTVector;
	AlignedPointTVector voxel_centers;

	pcl::octree::OctreePointCloudSearch<PointT> oc_search(0.2f);
	oc_search.setInputCloud(c);
	oc_search.addPointsFromInputCloud();
	oc_search.getOccupiedVoxelCenters(voxel_centers);
	//cout << "Voxel found: " << voxel_centers.size() << endl;

	int count = 0;
	for (std::vector<PointT, Eigen::aligned_allocator<PointT>>::const_iterator pit = voxel_centers.begin(); pit != voxel_centers.end(); pit++)
	{
		if (pow(pit->x, 2) + pow(pit->y, 2) < 25)
			continue;

		PointT center;
		center.x = pit->x;
		center.y = pit->y;
		center.z = pit->z;

		std::vector<int> PointIdx;
		if (depth_cal(c, &oc_search, center, PointIdx) > 0.08)
		{
			for (vector<int>::const_iterator idx = PointIdx.begin(); idx != PointIdx.end(); idx++)
				bound_cloud->push_back(c->points[*idx]);
			count++;
		}
	}

	cout << "Bound voxel remain: " << count << endl;
}

PointCloud<PointT>::Ptr density_fitting(PointCloud<PointT>::Ptr c, float size, int low, int high)
{
	typedef std::vector<PointT, Eigen::aligned_allocator<PointT>> AlignedPointTVector;
	AlignedPointTVector voxel_centers;

	pcl::octree::OctreePointCloudSearch<PointT> oc_search(size);
	oc_search.setInputCloud(c);
	oc_search.addPointsFromInputCloud();
	oc_search.getOccupiedVoxelCenters(voxel_centers);
	//cout << "Voxel found: " << voxel_centers.size() << endl;

	PointCloud<PointT>::Ptr tmp_c(new PointCloud<PointT>);
	for (std::vector<PointT, Eigen::aligned_allocator<PointT>>::const_iterator pit = voxel_centers.begin(); pit != voxel_centers.end(); pit++)
	{
		PointT center;
		center.x = pit->x;
		center.y = pit->y;
		center.z = pit->z;

		std::vector<int> PointIdx;
		if (oc_search.voxelSearch(center, PointIdx))			//执行体素近邻搜索
		{
			if (PointIdx.size() > low && PointIdx.size() < high)
			{
				for (int idx = 0; idx < PointIdx.size(); idx++)
					tmp_c->push_back(c->points[PointIdx[idx]]);
			}
		}
	}
	return tmp_c;
}

void bound_fitting(PointCloud<PointT>::Ptr c)
{
	std::vector<PointCloud<PointT>::Ptr> *clusters = euclidean_cluster(c, 0.3, 10);
	c->clear();
	std::vector<float> all_a;
	float left_side = 15, right_side = -15;

	for (int i = 0; i < clusters->size(); ++i)
	{
		float a = 0.0, b = 0.0;
		line_fitting((*clusters)[i], a, b);
		//cout << "Fitting line: y = " << b << "x + " << a << endl;
		all_a.push_back(a);
		if (abs(b) > 0.2)
			continue;

		if (a > 0 && a < left_side)
			left_side = a;
		if (a < 0 && a > right_side)
			right_side = a;
	}
	for (int i = 0; i < all_a.size(); i++)
		if (abs(left_side - all_a[i]) < 1 || abs(right_side - all_a[i]) < 1)
			*c += *(*clusters)[i];

	//std::vector<double> bound_side;
	//bound_side.push_back(right_side);
	//bound_side.push_back(left_side);

	return ;
}

void bound_fitting(PointCloud<PointT>::Ptr c, float track_k, float track_b)
{
	std::vector<PointCloud<PointT>::Ptr> *clusters = euclidean_cluster(c, 0.3, 10);
	c->clear();
	
	float positive_side = 10, negative_side = -10;
	float base = sqrt(1 + track_k * track_k);
	float alpha = atan(track_k) * 180 / 3.14;
	for (int i = 0; i < clusters->size(); ++i)
	{
		float b = 0.0, k = 0.0;
		line_fitting((*clusters)[i], b, k);
		//cout << "Fitting line: y = " << b << "x + " << a << endl;
		if (abs(atan(k) * 180 / 3.14 - alpha) > 10)
			continue;

		float dis = ((*clusters)[i]->points[0].y - track_k * (*clusters)[i]->points[0].x - track_b) / base;

		if (dis > 0 && dis < positive_side)
			positive_side = dis;
		if (dis < 0 && dis > negative_side)
			negative_side = dis;
	}
	for (int i = 0; i < clusters->size(); i++)
	{
		for (int j = 0; j < (*clusters)[i]->size(); j++)
		{
			float dis = ((*clusters)[i]->points[j].y - track_k * (*clusters)[i]->points[j].x - track_b) / base;
			if (abs(positive_side - dis) < 2 || abs(negative_side - dis) < 2)
				c->push_back((*clusters)[i]->points[j]);
		}
	}
	return ;
}

void bound_detecting(PointCloud<PointT>::Ptr c)
{
	clock_t start, end;
	cout << "\nBound detector started! " << endl;
	start = clock();

	bound_depth_fitting(c);

	cloud_visualization(bound_cloud);
	//if(bound_cloud->size() > 100)
	//	sor_filter(bound_cloud, 10, 1);
	//else if(bound_cloud->size() > 30)
	//	sor_filter(bound_cloud, 5, 1);

	bound_fitting(bound_cloud);
	cloud_visualization(bound_cloud);

	end = clock();
	cout << "Bound size: " << bound_cloud->size() << endl;
	cout << "Bound detecting finished! " <<
		"  Running time: " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
}

void pipe_line(string data_num, int first, int last, int if_ground, int if_pole, int if_bound, int if_tag)
{
	for (int i = first; i < last; i++)
	{
		string filename = to_string(i) + ".pcd";
		cout << "-----------------------------------------------------------------" << endl;
		cout << "********** " << filename << ": Data Processing Started!**********\n" << endl;
		clock_t all_start, all_end;
		all_start = clock();

		//地面点云滤波
		if (if_ground == 1)
		{
			load_pcd_data(cloud, "row_data/data_" + data_num + "/", filename);
			ground_filter(cloud);
			if (non_ground_cloud->size() > 0)
				save_pcd_data_bin(non_ground_cloud, "exp_data/data_" + data_num + "/non_ground/", filename);
			if (ground_cloud->size() > 0)
				save_pcd_data_bin(ground_cloud, "exp_data/data_" + data_num + "/ground/", filename);
		}
		else
		{
			if(if_pole == 1)
				load_pcd_data(non_ground_cloud, "exp_data/data_" + data_num + "/non_ground/", filename);
			if(if_bound == 1 || if_tag == 1)
				load_pcd_data(ground_cloud, "exp_data/data_" + data_num + "/ground/", filename);
		}
		//杆状物识别
		if (if_pole == 1)
		{
			pole_detecting(non_ground_cloud);
			if (pole_cloud->size() > 0)
				save_pcd_data_bin(pole_cloud, "exp_data/data_" + data_num + "/pole/", filename);
		}
		//道路边界及标识线检测
		if (if_bound == 1)
		{
			bound_detecting(ground_cloud);
			if (bound_cloud->size() > 0)
				save_pcd_data_bin(bound_cloud, "exp_data/data_" + data_num + "/bound/", filename);
		}
		if (if_tag == 1)
		{
			tag_cloud->clear();
			tag_cloud = pass_through(ground_cloud, "intensity", 0.4, 1.0, 1, 0);
			//tag_cloud = pass_through(tag_cloud, "y", -10, 10, 1, 1);
			tag_cloud = pass_through(tag_cloud, "x", -10, 10, 1, 1);
			if (tag_cloud->size() > 0)
				save_pcd_data_bin(tag_cloud, "exp_data/data_" + data_num + "/tag/", filename);
		}

		all_end = clock();
		cout << "\n********** " << filename << ": Data Processing Finished! Running time: " 
			<< double(all_end - all_start) / CLOCKS_PER_SEC << "s **********" << endl;
		cout << "-----------------------------------------------------------------\n" << endl;
	}
}

void rotation(float alpha, float &x, float &y)
{
	float xx = cos(alpha) * x - sin(alpha) * y;
	float yy = sin(alpha) * x + cos(alpha) * y;

	x = xx;
	y = yy;
}

void joint_cloud(string type, string data_num, int first, int last, int ifwrite = 0)
{
	if (!load_move_step("D:/Mr.Niro/School/graduation project/Data/2011_09_26_drive_00" + data_num + "_sync/oxts/move_step.txt"))
		return;

	cloud->clear();
	for (int i = first; i < last; i++)
	{
		if (type == "track")
		{			
			PointT p0(1.0);
			cloud->push_back(p0);
		}
		else
		{
			string path, file;
			file = to_string(i) + ".pcd";
			path = "exp_data/data_" + data_num + "/" + type + "/";
			load_pcd_data(cloud, path, file);

			//if (type == "ground")
			//{
			//	cloud = pass_through(cloud, "intensity", 0.3, 1.0, 1, 1);
			//	cloud = pass_through(cloud, "x", -10, 10, 1, 1);
			//	cloud = pass_through(cloud, "y", -15, 15, 1, 1);
			//}
		}

		for (int idx = 0; idx < cloud->size(); idx++)
		{
			PointT p;
			p.x = cloud->points[idx].x;
			p.y = cloud->points[idx].y;
			p.z = cloud->points[idx].z;

			rotation(move_rotate[i][3], p.y, p.z);
			rotation(move_rotate[i][4], p.z, p.x);
			rotation(move_rotate[i][5], p.x, p.y);
			if (i > 0)
			{
				p.x += move_rotate[i][0];
				p.y += move_rotate[i][1];
				p.z += move_rotate[i][2];
			}
			p.intensity = cloud->points[idx].intensity;
			my_cloud->push_back(p);
		}
		//cout << "currently cloud size: " << my_cloud->size() << endl;
		cloud->clear();
	}

	if (ifwrite == 1)
	{
		save_pcd_data_bin(my_cloud, "exp_data/data_" + data_num + "/", 
			type + "_" + to_string(first) + "-" + to_string(last) + ".pcd");
		my_cloud->clear();
	}
	return;
}

void pole_processing(string data_num, int first, int last)
{
	load_pcd_data(cloud, "exp_data/data_" + data_num + "/", "pole_" + to_string(first) + "-" + to_string(last) + ".pcd");
	voxel_grid_filter(cloud, 0.1);

	vector<PointCloud<PointT>::Ptr> *pole_clusters = euclidean_cluster(cloud, 0.3, 30);
	load_pcd_data(cloud, "exp_data/data_" + data_num + "/", "non_ground_" + to_string(first) + "-" + to_string(last) + ".pcd");
	voxel_grid_filter(cloud, 0.1);

	int pole_num = pole_clusters->size();
	for (int i = 0; i < pole_num; i++)
	{
		PointCloud<PointT>::Ptr cluster = (*pole_clusters)[i];
		PointT minPt, maxPt;
		getMinMax3D(*cluster, minPt, maxPt);

		double min_x = minPt.x, max_x = maxPt.x;
		double min_y = minPt.y, max_y = maxPt.y;
		double min_z = minPt.z, max_z = maxPt.z;

		vector<double> extremum;
		extremum.push_back(min_x);
		extremum.push_back(max_x);
		extremum.push_back(min_y);
		extremum.push_back(max_y);
		extremum.push_back(min_z);
		extremum.push_back(max_z);

		add_cube_for_pole(cloud, extremum);
	}

	cout << "all pole detected in data " << data_num <<": " << pole_num << endl;
	//cloud_visualization(cloud);
	save_pcd_data_bin(cloud, "exp_data/data_" + data_num + "/", "pole_whole.pcd");
}

float cal_len_wid(PointCloud<PointT>::Ptr c, float k, float a)
{
	//计算线性点云的长宽之比
	PointT minPt, maxPt;
	getMinMax3D(*c, minPt, maxPt);
	float len = sqrt(pow(maxPt.y - minPt.y, 2) + pow(maxPt.x - minPt.x, 2));
	float base = sqrt(1 + k * k);
	float max_e = 0, min_e = 0;
	for (int j = 0; j < c->size(); j++)
	{
		float e = (c->points[j].y - c->points[j].x * k - a) / base;
		if (e > max_e)
			max_e = e;
		else if (e < min_e)
			min_e = e;
	}
	float width = max_e - min_e;

	return len / width;
}

void tag_line_detecting(string data_num, int first, int last, int if_write = 1)
{
	load_pcd_data(tag_cloud, "exp_data/data_" + data_num + "/", "tag_whole_clear.pcd");
	cloud_visualization(tag_cloud);
	//PointCloud<PointT>::Ptr tmp_c = density_fitting(tag_cloud, 0.1, 2, 30);
	//cout << "remain " << tmp_c->size() << " data points after density fitting" << endl;
	//cloud_visualization(tmp_c);
	
	//cloud_visualization(tag_cloud);
	sor_filter(tag_cloud, 15, 2.0, 0);
	//sor_filter(tag_cloud, 10, 2.0, 0);
	//sor_filter(tag_cloud, 5, 0);
	voxel_grid_filter(tag_cloud, 0.1, 0);
	cloud_visualization(tag_cloud);

	vector<PointCloud<PointT>::Ptr> *clusters = euclidean_cluster(tag_cloud, 0.3, 10);
	tag_cloud->clear();

	for (int i = 0; i < clusters->size(); i++)
	{
		PointCloud<PointT>::Ptr p = (*clusters)[i];

		float a, k;
		float error = line_fitting(p, a, k);
		if (error < 0.3)
		{
			//cout << "size = " << (*clusters)[i]->size() << " error = " << error << endl;
			if (cal_len_wid(p, k, a) >= 2.0)
				*tag_cloud += *p;
		}
		else if (p->size() < 800)
		{	//类内K近邻聚类再拟合
			pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
			while (p->size() > 20)
			{
				tree->setInputCloud(p);
				int j = 0;
				for (j = 0; j < p->size(); j++)
				{
					vector<int> idx;
					vector<float> dis;
					tree->nearestKSearch(p->points[j], 20, idx, dis);
					PointCloud<PointT>::Ptr temp_c(new PointCloud<PointT>);
					for (int q = 0; q < idx.size(); q++)
						temp_c->push_back(p->points[idx[q]]);
					if (line_fitting(temp_c, a, k) < 0.2)
					{
						if (cal_len_wid(temp_c, k, a) >= 2.0)
						{
							*tag_cloud += *temp_c;
							std::sort(idx.begin(), idx.end(), comp);		//若不排序 erase后点云索引将错乱 且会出现越界
							for (int q = 0; q < idx.size(); q++)
								p->erase(p->begin() + idx[q]);
							//cout << "Save!" << endl;
							break;
						}
					}
				}
				if (j >= p->size())
					break;
			}
			vector<PointCloud<PointT>::Ptr> *p_clu = euclidean_cluster(p, 0.3, 5);
			for (int m = 0; m < p_clu->size(); m++)
			{
				PointCloud<PointT>::Ptr q = (*p_clu)[m];
				if (line_fitting(q, a, k) < 0.2)
					if (cal_len_wid(q, k, a) >= 1.3)
						*tag_cloud += *q;
			}
			/*
			//尝试基于平滑度约束再聚类
			std::vector<PointCloud<PointT>::Ptr>* new_clusters = re_cluster(p);
			cout << "new clusters: " << new_clusters->size() << endl;
			for (int j = 0; j < new_clusters->size(); j++)
				if (line_fitting((*new_clusters)[j], a, k) < 0.3)
				{
					*tag_cloud += *p;
				}
			*/
		}
	}

	//sor_filter(tag_cloud, 10, 0.5, 0);

	if (if_write == 1)
	{
		load_pcd_data(bound_cloud, "exp_data/data_" + data_num + "/", "bound_whole_clear.pcd");
		*tag_cloud += *bound_cloud;
		save_pcd_data_bin(tag_cloud, "exp_data/data_" + data_num + "/", "tag_bound_whole_clear.pcd");
	}
	/*pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(gcloud);
	int row_size = tag_cloud->size();
	for (int i = 0; i < row_size; i++)
	{
		vector<int> idx;
		vector<float> dis;
		tree->radiusSearch(tag_cloud->points[i], 0.2, idx, dis, 10);
		for (int j = 0; j < idx.size(); j++)
			if (gcloud->points[idx[j]].intensity > 0.4)
				tag_cloud->push_back(gcloud->points[idx[j]]);
	}*/
}

void bound_track_clear(string data_num, int first, int last, float width = 0.3, int ifwrite = 1)
{
	joint_cloud("track", data_num, first, last, 1);
	load_pcd_data(track_cloud, "exp_data/data_" + data_num + "/", "track_" + to_string(first) + "-" + to_string(last) + ".pcd");
	load_pcd_data(cloud, "exp_data/data_" + data_num + "/", "bound_whole_plus.pcd");
	//cloud_visualization(cloud);
	bound_cloud->clear();

	typedef std::vector<PointT, Eigen::aligned_allocator<PointT>> AlignedPointTVector;
	AlignedPointTVector voxel_centers;
	pcl::octree::OctreePointCloudSearch<PointT> oc_search(0.2f);
	oc_search.setInputCloud(cloud);
	oc_search.addPointsFromInputCloud();
	oc_search.getOccupiedVoxelCenters(voxel_centers);

	int track_size = track_cloud->size();
	PointCloud<PointT>::Ptr track_line(new PointCloud<PointT>);
	for (int i = 0; i < track_size; i++)
	{
		track_line->push_back(track_cloud->points[i]);
		if (track_line->size() == 3)
		{
			float k, b;
			line_fitting(track_line, b, k);
			float kk, bb;
			if (k == 0)
				kk = 1000.0;
			else
				kk = 1 / k;
			bb = track_cloud->points[i - 1].y - kk * track_cloud->points[i - 1].x;
			float base = sqrt(1 + kk * kk);
			float track_base = sqrt(1 + k * k);

			//确定边界点提取范围
			vector<PointT> potential_centers;
			for (vector<PointT, Eigen::aligned_allocator<PointT>>::const_iterator pit = voxel_centers.begin(); pit != voxel_centers.end(); pit++)
			{
				PointT center;
				center.x = pit->x;
				center.y = pit->y;
				center.z = pit->z;
				float error = abs(center.y - center.x * kk - bb) / base;
				center.intensity = error;
				if (error < 3)
					potential_centers.push_back(center);
			}

			//寻找距离track最近的一系列边界点
			vector<PointT> low_error_points;
			float positive_error = 10;
			float negative_error = -10;
			for (vector<PointT>::const_iterator point = potential_centers.begin(); point != potential_centers.end(); point++)
			{
				PointT center = *point;
				center.intensity = (point->y - point->x * k - b) / track_base;   //利用intensity属性存储error
				if (center.intensity < 0 && center.intensity > negative_error)
					negative_error = center.intensity;
				if (center.intensity > 0 && center.intensity < positive_error)
					positive_error = center.intensity;
				low_error_points.push_back(center);
			}

			vector<PointT> clost_points;			
			for (vector<PointT>::const_iterator point = low_error_points.begin(); point != low_error_points.end(); point++)
			{
				if (abs(point->intensity - positive_error) < width || abs(point->intensity - negative_error) < width)
					clost_points.push_back(*point);
			}

			for (vector<PointT>::const_iterator point = clost_points.begin(); point != clost_points.end(); point++)
			{
				vector<int> PointIdx;
				if (oc_search.voxelSearch(*point, PointIdx))
				{
					sort(PointIdx.begin(), PointIdx.end(), comp);
					for (vector<int>::const_iterator idx = PointIdx.begin(); idx != PointIdx.end(); idx++)
					{
						bound_cloud->push_back(cloud->points[*idx]);
						//cloud->erase(cloud->begin() + *idx);
					}
				}
			}
			/*joint_cloud("bound", data_num, i - 4, i + 1, 0);
			bound_fitting(my_cloud, k, b);
			*bound_cloud += *my_cloud;
			my_cloud->clear();*/
			
			track_line->clear();
			i -= 1;
		}
	}
	//voxel_grid_filter(bound_cloud, 0.2);
	cout << "currently bound size: " << bound_cloud->size() << endl;
	//*bound_cloud += *track_cloud;
	if (ifwrite == 1)
		save_pcd_data_bin(bound_cloud, "exp_data/data_" + data_num + "/", "bound_whole_clear.pcd");
}

void tag_bound_clear(string data_num, int first, int last, float pass_i = 0, int ifwrite = 1)
{
	joint_cloud("track", data_num, first, last, 1);
	load_pcd_data(track_cloud, "exp_data/data_" + data_num + "/", "track_" + to_string(first) + "-" + to_string(last) + ".pcd");
	load_pcd_data(bound_cloud, "exp_data/data_" + data_num + "/", "bound_whole_clear.pcd");
	load_pcd_data(tag_cloud, "exp_data/data_" + data_num + "/", "tag_" + to_string(first) + "-" + to_string(last) + ".pcd");
	if(pass_i > 0)
		tag_cloud = pass_through(tag_cloud, "intensity", pass_i, 1.0, 0, 1);
	voxel_grid_filter(bound_cloud, 0.2);

	PointCloud<PointT>::Ptr temp_tag(new PointCloud<PointT>);
	PointCloud<PointT>::Ptr track_line(new PointCloud<PointT>);
	int track_size = track_cloud->size();
	for (int i = 0; i < track_size; i++)
	{
		track_line->push_back(track_cloud->points[i]);
		if (i % 10 == 9)
		{
			float k, b;
			line_fitting(track_line, b, k);
			float kk, bb;
			if (k == 0)
				kk = 1000.0;
			else
				kk = 1 / k;
			bb = track_cloud->points[i - 5].y - kk * track_cloud->points[i - 5].x;
			float base = sqrt(1 + kk * kk);
			float track_base = sqrt(1 + k * k);

			//确定边界点提取范围
			vector<PointT> potential_tag;
			vector<PointT> nearest_bound;
			for (int i = 0; i < bound_cloud->size(); i++)
			{
				PointT p = bound_cloud->points[i];
				float error = abs(p.y - p.x * kk - bb) / base;
				if (error < 10)
					nearest_bound.push_back(p);
			}
			for (int i = 0; i < tag_cloud->size(); i++)
			{
				PointT p = tag_cloud->points[i];
				float error = abs(p.y - p.x * kk - bb) / base;
				if (error < 10)
					potential_tag.push_back(p);
			}

			//寻找距离track最近的一系列边界点
			float positive_error = 10;
			float negative_error = -10;
			for (vector<PointT>::const_iterator point = nearest_bound.begin(); point != nearest_bound.end(); point++)
			{
				float error = (point->y - point->x * k - b) / track_base;   //利用intensity属性存储error
				if (error < 0 && error > negative_error)
					negative_error = error;
				if (error > 0 && error < positive_error)
					positive_error = error;
			}

			for (vector<PointT>::const_iterator point = potential_tag.begin(); point != potential_tag.end(); point++)
			{
				PointT p = *point;
				float dis = (point->y - point->x * k - b) / track_base;
				if (dis > 0 && dis < positive_error || dis < 0 && dis > negative_error)
				{
					p.intensity = 0.8;
					temp_tag->push_back(p);
				}
			}
			track_line->clear();
		}
	}
	if (ifwrite == 1)
	{
		save_pcd_data_bin(temp_tag, "exp_data/data_" + data_num + "/", "tag_whole_clear.pcd");
	}
}

void bound_breed(string data_num, int first, int last, float pass_i = 0, float height = 0.08, int ifwrite = 1)
{
	load_pcd_data(bound_cloud, "exp_data/data_" + data_num + "/", "bound_" + to_string(first) + "-" + to_string(last) + ".pcd");
	voxel_grid_filter(bound_cloud, 0.1, 1);
	sor_filter(bound_cloud, 20, 0);
	
	load_pcd_data(ground_cloud, "exp_data/data_" + data_num + "/", "ground_" + to_string(first) + "-" + to_string(last) + ".pcd");
	ground_cloud = pass_through(ground_cloud, "intensity", pass_i, 1.0, 0, 1);

	typedef std::vector<PointT, Eigen::aligned_allocator<PointT>> AlignedPointTVector;
	AlignedPointTVector voxel_centers;
	pcl::octree::OctreePointCloudSearch<PointT> oc_search(0.2f);
	oc_search.setInputCloud(ground_cloud);
	oc_search.addPointsFromInputCloud();
	oc_search.getOccupiedVoxelCenters(voxel_centers);

	pcl::search::KdTree<PointT>::Ptr tree_delete(new pcl::search::KdTree<PointT>);
	tree_delete->setInputCloud(bound_cloud);
	for (vector<PointT, Eigen::aligned_allocator<PointT>>::const_iterator pit = voxel_centers.begin(); pit != voxel_centers.end(); pit++)
	{	//排除已包含边界点的体素
		PointT center;
		center.x = pit->x;
		center.y = pit->y;
		center.z = pit->z;
		vector<int> idx;
		vector<float> dis;
		if (tree_delete->radiusSearch(center, 0.1, idx, dis) > 0)
			continue;
		center.intensity = 0.0;
		cloud->push_back(center);
	}

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	for (int i = 0; i < bound_cloud->size(); i++)
	{
		if (bound_cloud->points[i].intensity == 0.3)
			continue;
		bound_cloud->points[i].intensity = 0.3;
		vector<int> idx;
		vector<float>dis;
		tree->radiusSearch(bound_cloud->points[i], 0.2828, idx, dis);
		for (vector<int>::const_iterator j = idx.begin(); j != idx.end(); j++)
		{
			std::vector<int> PointIdx;
			if (cloud->points[*j].intensity == 0.0 && depth_cal(ground_cloud, &oc_search, cloud->points[*j], PointIdx) > height)
			{
				cloud->points[*j].intensity = 1.0;
				bound_cloud->push_back(cloud->points[*j]);
			}
		}
	}
	
	//voxel_grid_filter(bound_cloud, 0.2, 1);
	cout << "currently size: " << bound_cloud->size() << endl;
	cloud->clear();
	//sor_filter(bound_cloud, 10, 0);
	//cout << "currently size: " << bound_cloud->size() << endl;

	if(ifwrite == 1)
		save_pcd_data_bin(bound_cloud, "exp_data/data_" + data_num + "/", "bound_whole_plus.pcd");
}

void data_processing(string data_num, int first, int last, int if_pipe, int if_pole, int if_breed, int if_tag)
{
	if (if_pipe == 1)
	{	//数据集目标提取pipe line
		//pipe_line(data_num, first, last, 0, 1, 0, 0);
		//joint_cloud("bound", data_num, first, last, 1);
		//joint_cloud("tag", data_num, first, last, 1);
		//joint_cloud("ground", data_num, first, last, 1);
		//joint_cloud("pole", data_num, first, last, 1);
		//joint_cloud("non_ground", data_num, first, last, 1);
	}

	if (if_pole == 1)
	{
		pole_processing(data_num, first, last);							//得到pole_whole.pcd
		load_pcd_data(my_cloud, "exp_data/data_" + data_num + "/", "pole_whole.pcd");
		cloud_visualization(my_cloud);
	}

	if (if_breed == 1)
	{	//道路边界区域增长及精化
		//cloud_visualization(bound_cloud);	
		bound_breed(data_num, first, last, 0.35, 0.06, 1);				//得到bound_whole_plus.pcd
		cloud_visualization(bound_cloud);
		bound_track_clear(data_num, first, last, 0.8, 1);				//得到bound_whole_clear.pcd
		cloud_visualization(bound_cloud);
	}

	if (if_tag == 1)
	{	//标识线提取
		tag_bound_clear(data_num, first, last, 0.45, 1);					//得到tag_whole_clear.pcd
		tag_line_detecting(data_num, first, last, 1);					//得到tag_bound_whole_clear.pcd

		load_pcd_data(my_cloud, "exp_data/data_" + data_num + "/", "tag_bound_whole_clear.pcd");
		cloud_visualization(my_cloud);
	}
}

int main()
{
	/* 
	//原数据集转换
	int num = get_filename("D:/Mr.Niro/School/graduation project/Data/2011_09_26_drive_0084_sync/velodyne_points/data/*.bin");
	for (int i = 0; i < num; i++)
	{
		load_bin_data(cloud, "D:/Mr.Niro/School/graduation project/Data/2011_09_26_drive_0084_sync/velodyne_points/data/", All_file[i]);
		save_pcd_data_bin(cloud, "row_data/data_84/", to_string(i) + ".pcd");
	}
	*/

	data_processing("13", 0, 143, 0, 0, 1, 1);
	//data_processing("84", 60, 382, 0, 1, 1, 1);
	//data_processing("01", 0, 107, 0, 0, 1, 1);

	system("pause");
	return 0;
}

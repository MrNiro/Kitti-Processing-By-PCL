#include <Config.h>

#include <CloudVisual.h>
#include <GroundFilter.h>
#include <CloudIO.h>

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <io.h>
#include <time.h>

using namespace std;

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

/*
void poly_intensity_fitting(PointCloud<PointT>::Ptr cloud, vector<float>& a, int m, vector<float>& dt)
{
	
	int N = cloud->size();
	if (N > 100000)
	{
		PointCloud<PointT>::Ptr temp(new PointCloud<PointT>);
		pcl::copyPointCloud(*cloud, *temp);
		voxel_grid_filter(temp, 0.1);
		N = temp->size();
	}
	m = (m > 20) ? 20 : m;
	float z = 0.0, p, cloud, g, q, d1, d2, s[20], t[20], b[20];
	float x[100000] = { 0.0 };
	float y[100000] = { 0.0 };

	for (int i = 0; i < m; i++) 
		a.push_back(0.0);	
	
	for (int i = 0; i < N; i++)
	{
		x[i]
		z += x[i] / (1.0*n);
	}
	b[0] = 1.0; d1 = 1.0*n; p = 0.0; cloud = 0.0;
	for (i = 0; i <= n - 1; i++)
	{
		p = p + (x[i] - z); cloud = cloud + y[i];
	}
	cloud = cloud / d1; p = p / d1;
	a[0] = cloud * b[0];
	if (m > 1)
	{
		t[1] = 1.0; t[0] = -p;
		d2 = 0.0; cloud = 0.0; g = 0.0;
		for (i = 0; i <= n - 1; i++)
		{
			q = x[i] - z - p; d2 = d2 + q * q;
			cloud = cloud + y[i] * q;
			g = g + (x[i] - z)*q*q;
		}
		cloud = cloud / d2; p = g / d2; q = d2 / d1;
		d1 = d2;
		a[1] = cloud * t[1]; a[0] = cloud * t[0] + a[0];
	}
	for (j = 2; j <= m - 1; j++)
	{
		s[j] = t[j - 1];
		s[j - 1] = -p * t[j - 1] + t[j - 2];
		if (j >= 3)
			for (k = j - 2; k >= 1; k--)
				s[k] = -p * t[k] + t[k - 1] - q * b[k];
		s[0] = -p * t[0] - q * b[0];
		d2 = 0.0; cloud = 0.0; g = 0.0;
		for (i = 0; i <= n - 1; i++)
		{
			q = s[j];
			for (k = j - 1; k >= 0; k--)
				q = q * (x[i] - z) + s[k];
			d2 = d2 + q * q; cloud = cloud + y[i] * q;
			g = g + (x[i] - z)*q*q;
		}
		cloud = cloud / d2; p = g / d2; q = d2 / d1;
		d1 = d2;
		a[j] = cloud * s[j]; t[j] = s[j];
		for (k = j - 1; k >= 0; k--)
		{
			a[k] = cloud * s[k] + a[k];
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
		//cloud->erase(cloud->begin() + idx);
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

	//pcl::visualization::CloudVisual viewer("Viewer");
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

	NoiseFilter noise_filter = NoiseFilter(row_c, true);
	while (step_z < max_z)
	{
		PointCloud<PointT>::Ptr c = noise_filter.pass_through("z", step_z, step_z + step, false);
		step_z += step;

		if (c->size() > 8)
		{
			noise_filter.set_cloud(c);
			noise_filter.sor_filter(3, 2.0);
		}
		double e = 0.0, c_x, c_y;
		float r = MathTools::circle_fitting(c, c_x, c_y, e);
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
	NoiseFilter noise_filter = NoiseFilter(c, true);
	pole_cloud = noise_filter.pass_through("intensity", 0.1, 1.0, false);

	noise_filter.set_cloud(pole_cloud);
	noise_filter.voxel_grid_filter(0.1);

	vector<PointCloud<PointT>::Ptr> *non_ground_clusters = euclidean_cluster(pole_cloud, 0.3, 50);
	pole_cloud->clear();
	//pcl::visualization::CloudVisual viewer("Viewer");
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
		if (MathTools::depth_cal(c, &oc_search, center, PointIdx) > 0.08)
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
		MathTools::line_fitting((*clusters)[i], a, b);
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
		MathTools::line_fitting((*clusters)[i], b, k);
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

	CloudVisual* viewer = new CloudVisual(bound_cloud);
	viewer->cloud_visualization();
	//if(bound_cloud->size() > 100)
	//	sor_filter(bound_cloud, 10, 1);
	//else if(bound_cloud->size() > 30)
	//	sor_filter(bound_cloud, 5, 1);

	bound_fitting(bound_cloud);
	viewer->cloud_visualization();

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
		NoiseFilter noise_filter(ground_cloud, false);
		if (if_ground == 1)
		{
			CloudIO::load_pcd_data(cloud, "row_data/data_" + data_num + "/", filename);
			GroundFilter ground_filter = GroundFilter(cloud, false);
			ground_filter.ground_filter();
			if (non_ground_cloud->size() > 0)
				CloudIO::save_pcd_data_bin(non_ground_cloud, "exp_data/data_" + data_num + "/non_ground/", filename);
			if (ground_cloud->size() > 0)
				CloudIO::save_pcd_data_bin(ground_cloud, "exp_data/data_" + data_num + "/ground/", filename);
		}
		else
		{
			if(if_pole == 1)
				CloudIO::load_pcd_data(non_ground_cloud, "exp_data/data_" + data_num + "/non_ground/", filename);
			if(if_bound == 1 || if_tag == 1)
				CloudIO::load_pcd_data(ground_cloud, "exp_data/data_" + data_num + "/ground/", filename);
		}
		//杆状物识别
		if (if_pole == 1)
		{
			pole_detecting(non_ground_cloud);
			if (pole_cloud->size() > 0)
				CloudIO::save_pcd_data_bin(pole_cloud, "exp_data/data_" + data_num + "/pole/", filename);
		}
		//道路边界及标识线检测
		if (if_bound == 1)
		{
			bound_detecting(ground_cloud);
			if (bound_cloud->size() > 0)
				CloudIO::save_pcd_data_bin(bound_cloud, "exp_data/data_" + data_num + "/bound/", filename);
		}
		if (if_tag == 1)
		{
			tag_cloud->clear();
			noise_filter.set_cloud(ground_cloud);
			tag_cloud = noise_filter.pass_through("intensity", 0.4, 1.0, false);
			//tag_cloud = pass_through(tag_cloud, "y", -10, 10, 1, 1);
			noise_filter.set_cloud(tag_cloud);
			tag_cloud = noise_filter.pass_through("x", -10, 10, true);
			if (tag_cloud->size() > 0)
				CloudIO::save_pcd_data_bin(tag_cloud, "exp_data/data_" + data_num + "/tag/", filename);
		}

		all_end = clock();
		cout << "\n********** " << filename << ": Data Processing Finished! Running time: " 
			<< double(all_end - all_start) / CLOCKS_PER_SEC << "s **********" << endl;
		cout << "-----------------------------------------------------------------\n" << endl;
	}
}

void pole_processing(string data_num, int first, int last)
{
	CloudIO::load_pcd_data(cloud, "exp_data/data_" + data_num + "/", "pole_" + to_string(first) + "-" + to_string(last) + ".pcd");
	NoiseFilter noise_filter = NoiseFilter(cloud, false);
	noise_filter.voxel_grid_filter(0.1);

	vector<PointCloud<PointT>::Ptr> *pole_clusters = euclidean_cluster(cloud, 0.3, 30);
	CloudIO::load_pcd_data(cloud, "exp_data/data_" + data_num + "/", "non_ground_" + to_string(first) + "-" + to_string(last) + ".pcd");
	noise_filter.voxel_grid_filter(0.1);

	int pole_num = pole_clusters->size();
	CloudVisual* viewer = new CloudVisual(cloud);
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

		viewer->add_cube_for_pole(extremum);
	}

	cout << "all pole detected in data " << data_num <<": " << pole_num << endl;
	//cloud_visualization(cloud);
	CloudIO::save_pcd_data_bin(cloud, "exp_data/data_" + data_num + "/", "pole_whole.pcd");
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
	CloudVisual* viewer = new CloudVisual(tag_cloud);

	CloudIO::load_pcd_data(tag_cloud, "exp_data/data_" + data_num + "/", "tag_whole_clear.pcd");
	viewer->cloud_visualization();
	//PointCloud<PointT>::Ptr tmp_c = density_fitting(tag_cloud, 0.1, 2, 30);
	//cout << "remain " << tmp_c->size() << " data points after density fitting" << endl;
	//cloud_visualization(tmp_c);
	
	NoiseFilter noise_filter = NoiseFilter(tag_cloud, false);

	//cloud_visualization(tag_cloud);
	noise_filter.sor_filter(15, 2.0);
	//sor_filter(tag_cloud, 10, 2.0, 0);
	//sor_filter(tag_cloud, 5, 0);
	noise_filter.voxel_grid_filter(0.1);
	viewer->cloud_visualization();

	vector<PointCloud<PointT>::Ptr> *clusters = euclidean_cluster(tag_cloud, 0.3, 10);
	tag_cloud->clear();

	for (int i = 0; i < clusters->size(); i++)
	{
		PointCloud<PointT>::Ptr p = (*clusters)[i];

		float a, k;
		float error = MathTools::line_fitting(p, a, k);
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
					if (MathTools::line_fitting(temp_c, a, k) < 0.2)
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
				if (MathTools::line_fitting(q, a, k) < 0.2)
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
		CloudIO::load_pcd_data(bound_cloud, "exp_data/data_" + data_num + "/", "bound_whole_clear.pcd");
		*tag_cloud += *bound_cloud;
		CloudIO::save_pcd_data_bin(tag_cloud, "exp_data/data_" + data_num + "/", "tag_bound_whole_clear.pcd");
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
	my_cloud = CloudIO::joint_cloud("track", data_num, first, last, true);
	CloudIO::load_pcd_data(track_cloud, "exp_data/data_" + data_num + "/", "track_" + to_string(first) + "-" + to_string(last) + ".pcd");
	CloudIO::load_pcd_data(cloud, "exp_data/data_" + data_num + "/", "bound_whole_plus.pcd");
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
			MathTools::line_fitting(track_line, b, k);
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
		CloudIO::save_pcd_data_bin(bound_cloud, "exp_data/data_" + data_num + "/", "bound_whole_clear.pcd");
}

void tag_bound_clear(string data_num, int first, int last, float pass_i = 0, int ifwrite = 1)
{
	my_cloud = CloudIO::joint_cloud("track", data_num, first, last, true);
	CloudIO::load_pcd_data(track_cloud, "exp_data/data_" + data_num + "/", "track_" + to_string(first) + "-" + to_string(last) + ".pcd");
	CloudIO::load_pcd_data(bound_cloud, "exp_data/data_" + data_num + "/", "bound_whole_clear.pcd");
	CloudIO::load_pcd_data(tag_cloud, "exp_data/data_" + data_num + "/", "tag_" + to_string(first) + "-" + to_string(last) + ".pcd");

	NoiseFilter noise_filter = NoiseFilter(tag_cloud, false);
	if (pass_i > 0) 
	{
		tag_cloud = noise_filter.pass_through("intensity", pass_i, 1.0, true);
	}
	noise_filter.set_cloud(bound_cloud);
	noise_filter.voxel_grid_filter(0.2);

	PointCloud<PointT>::Ptr temp_tag(new PointCloud<PointT>);
	PointCloud<PointT>::Ptr track_line(new PointCloud<PointT>);
	int track_size = track_cloud->size();
	for (int i = 0; i < track_size; i++)
	{
		track_line->push_back(track_cloud->points[i]);
		if (i % 10 == 9)
		{
			float k, b;
			MathTools::line_fitting(track_line, b, k);
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
		CloudIO::save_pcd_data_bin(temp_tag, "exp_data/data_" + data_num + "/", "tag_whole_clear.pcd");
	}
}

void bound_breed(string data_num, int first, int last, float pass_i = 0, float height = 0.08, int ifwrite = 1)
{
	CloudIO::load_pcd_data(bound_cloud, "exp_data/data_" + data_num + "/", "bound_" + to_string(first) + "-" + to_string(last) + ".pcd");
	NoiseFilter noise_filter = NoiseFilter(bound_cloud, false);
	noise_filter.voxel_grid_filter(0.1);
	noise_filter.sor_filter(20, 0);
	
	CloudIO::load_pcd_data(ground_cloud, "exp_data/data_" + data_num + "/", "ground_" + to_string(first) + "-" + to_string(last) + ".pcd");
	noise_filter.set_cloud(ground_cloud);
	ground_cloud = noise_filter.pass_through("intensity", pass_i, 1.0, true);

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
			if (cloud->points[*j].intensity == 0.0 && MathTools::depth_cal(ground_cloud, &oc_search, cloud->points[*j], PointIdx) > height)
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
		CloudIO::save_pcd_data_bin(bound_cloud, "exp_data/data_" + data_num + "/", "bound_whole_plus.pcd");
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
	CloudVisual* viewer = new CloudVisual();
	if (if_pole == 1)
	{
		pole_processing(data_num, first, last);							//得到pole_whole.pcd
		CloudIO::load_pcd_data(my_cloud, "exp_data/data_" + data_num + "/", "pole_whole.pcd");
		viewer->set_cloud(my_cloud);
		viewer->cloud_visualization();
	}

	if (if_breed == 1)
	{	//道路边界区域增长及精化
		//cloud_visualization(bound_cloud);	
		viewer->set_cloud(bound_cloud);
		bound_breed(data_num, first, last, 0.35, 0.06, 1);				//得到bound_whole_plus.pcd
		viewer->cloud_visualization();
		bound_track_clear(data_num, first, last, 0.8, 1);				//得到bound_whole_clear.pcd
		viewer->cloud_visualization();
	}

	if (if_tag == 1)
	{	//标识线提取
		tag_bound_clear(data_num, first, last, 0.45, 1);					//得到tag_whole_clear.pcd
		tag_line_detecting(data_num, first, last, 1);					//得到tag_bound_whole_clear.pcd

		CloudIO::load_pcd_data(my_cloud, "exp_data/data_" + data_num + "/", "tag_bound_whole_clear.pcd");
		viewer->set_cloud(my_cloud);
		viewer->cloud_visualization();
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

	CloudIO::load_pcd_data(my_cloud, "data/exp_data/data_01/", "tag_bound_whole_clear.pcd");
	CloudVisual* viewer = new CloudVisual(my_cloud);
	viewer->cloud_visualization();

	//data_processing("84", 60, 382, 0, 1, 1, 1);
	//data_processing("13", 0, 143, 0, 0, 1, 1);
	//data_processing("01", 0, 107, 0, 0, 1, 1);

	/*load_pcd_data(cloud, "exp_data/data_84/", "tag_bound_whole_clear.pcd");
	cloud_visualization(cloud);*/

	/*pcl::visualization::CloudVisual viewer("Viewer");
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	for (int i = 10; i < 108; i++)
	{
		load_pcd_data(cloud, "exp_data/data_01/pole/", to_string(i) + ".pcd");
		viewer.showCloud(cloud);
		system("pause");
	}*/
	system("pause");
	return 0;
}

#include <CloudCluster.h>


CloudCluster::CloudCluster(bool silent) {
	this->silent = silent;
}

void cal_smooth(PointCloud<PointT>::Ptr row_c, PointCloud<PointT>::Ptr new_c, 
	pcl::search::KdTree<PointT>::Ptr tree, int idx, bool* idx_judge)
{
	if (idx_judge[idx])
		return;
	idx_judge[idx] = true;

	float radius = 0.3;

	std::vector<int> PointIdx;
	std::vector<float> square_dis;

	PointT point = row_c->points[idx];
	// Calculate the number of neighbor points near to point[idx], 
	int neighbor = tree->radiusSearch(point, radius, PointIdx, square_dis);
	if (neighbor == 0)
		return;

	float sum_dis = 0;
	for (float each: square_dis)
		sum_dis += each;
	float smooth = sqrt(sum_dis) / neighbor / radius;
	//cout << smooth << endl;

	if (smooth > 0 && smooth < 0.15)
	{
		new_c->push_back(point);
		//cloud->erase(cloud->begin() + idx);
		for (int i: PointIdx)
			if (!idx_judge[i])
				cal_smooth(row_c, new_c, tree, i, idx_judge);
	}
}

vector<PointCloud<PointT>::Ptr>* CloudCluster::euclidean_cluster(PointCloud<PointT>::Ptr cloud, 
	float dis, int min_size, bool ifwrite)
{
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;

	ec.setClusterTolerance(dis);
	ec.setMinClusterSize(min_size);
	ec.setMaxClusterSize(10000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	//cout << "Extracted " << cluster_indices.size() << " cluster" << endl;
	std::vector<pcl::PointCloud<PointT>::Ptr>* all_cluster = new vector<pcl::PointCloud<PointT>::Ptr>;

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(cloud->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		//std::cout << "Cluster " << j << ": " << cloud_cluster->points.size() << " data points." << std::endl;
		//cloud_visualization(cloud_cluster);

		if (ifwrite)
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

vector<PointCloud<PointT>::Ptr>* CloudCluster::advanced_euclidean_cluster(PointCloud<PointT>::Ptr cloud)
{
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);
	vector<PointCloud<PointT>::Ptr>* all_cluster(new vector<PointCloud<PointT>::Ptr>);

	//pcl::visualization::CloudVisual viewer("Viewer");
	//viewer.runOnVisualizationThreadOnce(viewerOneOff);

	const int row_size = cloud->size();

	// to record if each point has been assigned to a cluster
	bool* idx_judge = new bool[row_size]{ false };
	
	// go through each point and make them a single cluster at begining, 
	// extend the cluster according to "smooth" value of every point in the clustr,
	// including new points added into the cluster.
	for (int i = 0; i < row_size; i++)
	{
		if (idx_judge[i])
			continue;
		PointCloud<PointT>::Ptr new_c(new PointCloud<PointT>);
		cal_smooth(cloud, new_c, tree, i, idx_judge);

		if (!new_c->empty() && new_c->size() > 30)
		{
			//viewer.showCloud(new_c);
			//system("pause");
			all_cluster->push_back(new_c);
		}
	}
	return all_cluster;
}
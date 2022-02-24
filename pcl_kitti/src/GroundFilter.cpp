#include <GroundFilter.h>


GroundFilter::GroundFilter(PointCloud<PointT>::Ptr c, bool silent) {
	this->row_cloud = c;
	this->silent = silent;
}

void GroundFilter::height_segmentation()
{
	NoiseFilter noise_filter = NoiseFilter(row_cloud, this->silent);
	ground_cloud = noise_filter.pass_through("z", -5, -1, false);
	non_ground_cloud = noise_filter.pass_through("z", -1, 5, false);

	if (silent == 0)
		std::cout << "Remain "
		<< non_ground_cloud->points.size() << " non-ground data points, "
		<< ground_cloud->points.size() << " ground data points "
		<< "after height segmentation"
		//<< min_z + height << ")"
		<< std::endl;
}

void GroundFilter::pmf_filter(float height)
{
	pcl::PointIndicesPtr ground(new pcl::PointIndices);
	pcl::ProgressiveMorphologicalFilter<PointT> pmf;
	pmf.setInputCloud(ground_cloud);
	pmf.setMaxWindowSize(3);
	//pmf.setCellSize(0.5f);
	pmf.setSlope(0.5f);
	pmf.setInitialDistance(height);			//地面起伏越大，该值须设为越大
	pmf.setMaxDistance(1.0f);
	pmf.extract(ground->indices);

	PointCloud<PointT>::Ptr new_non_ground(new PointCloud<PointT>);
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(row_cloud);
	extract.setIndices(ground);
	extract.setNegative(true);
	extract.filter(*new_non_ground);
	extract.setNegative(false);
	extract.filter(*ground_cloud);

	*non_ground_cloud += *new_non_ground;
	if (silent == 0)
		std::cout << "Remain "
		<< non_ground_cloud->points.size() << " non-ground data points, "
		<< ground_cloud->points.size() << " ground data points after PMF filter"
		<< std::endl;
}

void GroundFilter::ground_filter()
{
	clock_t start, end;
	ground_cloud->clear();
	non_ground_cloud->clear();

	cout << "\nGround filter started!" << endl;
	start = clock();


	NoiseFilter noise_filter = NoiseFilter(row_cloud, this->silent);
	row_cloud = noise_filter.pass_through("x", -15, 15, true);
	row_cloud = noise_filter.pass_through("y", -20, 20, true);

	pcl::octree::OctreePointCloudSearch<PointT> oc_search(0.3);
	oc_search.setInputCloud(row_cloud);
	oc_search.addPointsFromInputCloud();

	height_segmentation();
	//ground_cloud = pass_through(ground_cloud, "intensity", 0.3, 1.0);
	
	noise_filter.set_cloud(ground_cloud);
	noise_filter.voxel_grid_filter(0.3);
	//ground_cloud = noise_filter.get_cloud();

	pmf_filter(0.1f);
	//sor_filter(ground_cloud, 5, 1.0, 1);

	int row_size = ground_cloud->size();
	for (int i = 0; i < row_size; i++)					//根据当前地面点反向查找
	{
		std::vector<int> PointIdx, PointIdx_H;
		PointT point = ground_cloud->points[i];
		if (MathTools::depth_cal(row_cloud, &oc_search, point, PointIdx) > 0.1)
		{
			point.z += 0.2;
			if (oc_search.voxelSearch(point, PointIdx_H))
				continue;
		}
		for (int idx = 0; idx < PointIdx.size(); idx++)
			ground_cloud->push_back(row_cloud->points[PointIdx[idx]]);
	}
	noise_filter.sor_filter(10, 1.0);

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
				if (row_cloud->points[PointIdx[idx]].z - minPt.z > 0.05)
					non_ground_cloud->push_back(row_cloud->points[PointIdx[idx]]);
		}
	}

	end = clock();
	std::cout << "Remain "
		<< non_ground_cloud->points.size() << " non-ground data points, "
		<< ground_cloud->points.size() << " ground data points after Ground filter"
		<< std::endl;
	cout << "Ground filter finished! Running time: " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
}

#include <NoiseFilter.h>


NoiseFilter::NoiseFilter(PointCloud<PointT>::Ptr c, bool silent) {
	this->cloud = c;
	this->silent = silent;
}

void NoiseFilter::set_cloud(PointCloud<PointT>::Ptr c) {
	this->cloud = c;
}

PointCloud<PointT>::Ptr NoiseFilter::get_cloud() {
	return this->cloud;
}

void NoiseFilter::voxel_grid_filter(float size)
{
	pcl::VoxelGrid<PointT> vgf;
	vgf.setInputCloud(cloud);
	vgf.setLeafSize(size, size, size);					//set the voxel grid size
	vgf.filter(*cloud);

	if (!silent)
		std::cout << "Remain "
		<< cloud->points.size()
		<< " data points after voxel grid filter"
		<< std::endl;
}

void NoiseFilter::sor_filter(int meanK, float thresh)
{
	pcl::StatisticalOutlierRemoval <PointT> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(meanK);
	sor.setStddevMulThresh(thresh);
	sor.filter(*cloud);

	if (!silent)
		std::cout << "Remain "
		<< cloud->points.size()
		<< " data points after SOR"
		<< std::endl;
}

void NoiseFilter::fog_filter()
{
	float min_intensity = 0.1, max_intensity = 0;
	for (int i = 0; i < cloud->size(); i++)
		if (cloud->points[i].intensity < min_intensity)
			min_intensity = cloud->points[i].intensity;

	for (int i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].intensity = (cloud->points[i].intensity - min_intensity) / 0.1 + min_intensity;
		if (cloud->points[i].intensity > max_intensity)
			max_intensity = cloud->points[i].intensity;
	}
	for (int i = 0; i < cloud->size(); i++)
	{
		cloud->points[i].intensity = cloud->points[i].intensity / max_intensity;
	}
}

PointCloud<PointT>::Ptr NoiseFilter::pass_through(string field, float m, float n, bool rowclear)
{
	pcl::PointCloud<PointT>::Ptr new_cloud(new pcl::PointCloud<PointT>());
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName(field);
	pass.setFilterLimits(m, n);
	pass.filter(*new_cloud);

	if (silent == 0)
		std::cout << "Remain "
		<< new_cloud->points.size()
		<< " data points after passThrough"
		<< std::endl;

	if (rowclear)
		cloud->clear();
	return new_cloud;
}


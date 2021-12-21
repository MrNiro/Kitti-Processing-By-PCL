#ifndef NOISEFILTER_H
#define NOISEFILTER_H

#include <Config.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

class NoiseFilter {

private:
	bool silent;
	PointCloud<PointT>::Ptr cloud;

public:
    NoiseFilter(PointCloud<PointT>::Ptr c, bool silent);

	void set_cloud(PointCloud<PointT>::Ptr c);

	PointCloud<PointT>::Ptr get_cloud();

	void voxel_grid_filter(float size = 0.1);

	void sor_filter(int meanK, float thresh);

	void fog_filter();

	PointCloud<PointT>::Ptr pass_through(string field, float m, float n, bool rowclear = false);
};

#endif //!NOISEFILTER_H

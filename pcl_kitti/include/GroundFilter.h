#ifndef GROUNDFILTER_H
#define GROUNDFILTER_H

#include <Config.h>
#include <NoiseFilter.h>
#include <MathTools.h>

#include <pcl/octree/octree.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>


class GroundFilter {

private:
	bool silent;
	PointCloud<PointT>::Ptr row_cloud;
	PointCloud<PointT>::Ptr ground_cloud;
	PointCloud<PointT>::Ptr non_ground_cloud;

public:
    GroundFilter(PointCloud<PointT>::Ptr c, bool silent);

	void height_segmentation();

	void pmf_filter(float height = 0.1f);

	void ground_filter();

};

#endif // !GROUNDFILTER_H

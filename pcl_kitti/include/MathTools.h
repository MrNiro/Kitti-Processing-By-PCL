#ifndef MATHTOOLS
#define MATHTOOLS

#include <Config.h>

class MathTools {

public:

	static double circle_fitting(PointCloud<PointT>::Ptr c, double& center_x, double& center_y, double& error);

	static float line_fitting(PointCloud<PointT>::Ptr c, float& a, float& b);

	static float depth_cal(PointCloud<PointT>::Ptr c, 
		pcl::octree::OctreePointCloudSearch<PointT>* oc, 
		PointT point, 
		std::vector<int>& PointIdx);

};

#endif // !MATHTOOLS

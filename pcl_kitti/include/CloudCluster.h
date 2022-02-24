#ifndef CLOUDCLUSTER_H
#define CLOUDCLUSTER_H

#include <Config.h>

#include <pcl/segmentation/extract_clusters.h>


class CloudCluster {

public:
    CloudCluster(bool silent = true);

    static std::vector<PointCloud<PointT>::Ptr>* euclidean_cluster(PointCloud<PointT>::Ptr cloud,
        float dis = 0.5, int min_size = 50, bool ifwrite = false);

    // Improve the typical euclidean clustering, to separate close objects by "smooth" value. 
    // This method will be slower than typical clustering(calulating smooth value could be time consuming)
    // Theoretically, this an O(n) algorithm, but it is implemented by recursion and can't be parallel
    // Best practice is invoke this after typical euclidean clustering and visual analysis.
    static std::vector<PointCloud<PointT>::Ptr>* advanced_euclidean_cluster(PointCloud<PointT>::Ptr cloud);

private:
    bool silent;

};

#endif // !CLOUDCLUSTER_H

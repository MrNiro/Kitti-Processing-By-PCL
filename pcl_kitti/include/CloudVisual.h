#ifndef CLOUDVIEWER_H
#define CLOUDVIEWER_H

#include <Config.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;


class CloudVisual {
public:
    CloudVisual();

    CloudVisual(PointCloud<PointT>::Ptr c);

    PointCloud<PointT>::Ptr get_cloud();

    void set_cloud(PointCloud<PointT>::Ptr c);

    void add_line(PointT v1, PointT v2);

    void add_cube_for_pole(vector<double>& extremum);

    void cloud_visualization();

private:
    PointCloud<PointT>::Ptr cloud;
};

#endif // !CLOUDVIEWER_H

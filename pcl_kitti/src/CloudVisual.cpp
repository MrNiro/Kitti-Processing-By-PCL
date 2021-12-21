#include "CloudVisual.h"


CloudVisual::CloudVisual() {
	this->cloud = nullptr;
}

CloudVisual::CloudVisual(PointCloud<PointT>::Ptr cloud) {
	this->cloud = cloud;
}

PointCloud<PointT>::Ptr CloudVisual::get_cloud() {
	return this->cloud;
}

void CloudVisual::set_cloud(PointCloud<PointT>::Ptr c) {
	this->cloud = c;
}

//

void CloudVisual::add_line(PointT v1, PointT v2)
{
	double step_x = (v2.x - v1.x) / 100;
	double step_y = (v2.y - v1.y) / 100;
	double step_z = (v2.z - v1.z) / 100;

	PointT point = v1;
	point.intensity = 0.001;
	cloud->push_back(point);
	for (int i = 0; i < 100; i++)
	{
		point.x += step_x;
		point.y += step_y;
		point.z += step_z;
		cloud->push_back(point);
	}
}

void CloudVisual::add_cube_for_pole(vector<double>& extremum)
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
					add_line(v[i], v[j]);
				else if (v[i].z == v[j].z)
					add_line(v[i], v[j]);
			}
			else if (v[i].y == v[j].y)
			{
				if (v[i].z == v[j].z)
					add_line(v[i], v[j]);
			}
		}
}

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 1.0, 1.0);
}

void CloudVisual::cloud_visualization()
{
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	system("pause");
}

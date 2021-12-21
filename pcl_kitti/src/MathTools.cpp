#include <MathTools.h>


double MathTools::circle_fitting(PointCloud<PointT>::Ptr c, double& center_x, double& center_y, double& error)
{
	double sum_x = 0.0f, sum_y = 0.0f;
	double sum_x2 = 0.0f, sum_y2 = 0.0f;
	double sum_x3 = 0.0f, sum_y3 = 0.0f;
	double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

	int N = c->points.size();
	for (int i = 0; i < N; i++)
	{
		double x = c->points[i].x;
		double y = c->points[i].y;

		sum_x += x;
		sum_y += y;
		sum_x2 += x * x;
		sum_y2 += y * y;
		sum_x3 += x * x * x;
		sum_y3 += y * y * y;
		sum_xy += x * y;
		sum_x1y2 += x * y * y;
		sum_x2y1 += x * x * y;
	}

	double C, D, E, G, H;
	double aa, bb, cc;

	C = N * sum_x2 - sum_x * sum_x;
	D = N * sum_xy - sum_x * sum_y;
	E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
	G = N * sum_y2 - sum_y * sum_y;
	H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;

	aa = (H * D - E * G) / (C * G - D * D);
	bb = (H * C - E * D) / (D * D - G * C);
	cc = -(aa * sum_x + bb * sum_y + sum_x2 + sum_y2) / N;

	center_x = aa / (-2);
	center_y = bb / (-2);
	double radius = sqrt(aa * aa + bb * bb - 4 * cc) / 2;


	for (int i = 0; i < N; i++)
	{
		double x = c->points[i].x;
		double y = c->points[i].y;

		error += abs(sqrt(pow(x - center_x, 2) + pow(y - center_y, 2)) - radius);
	}

	/*for (float x = center_x - radius; x <= center_x + radius; x += 0.01)
	{
		PointT p;
		p.x = x;
		p.z = 5;
		p.intensity = 0.01;
		p.y = center_y + sqrt(pow(radius, 2) - pow(x - center_x, 2));
		pole_cloud->push_back(p);
		p.y = center_y - sqrt(pow(radius, 2) - pow(x - center_x, 2));
		pole_cloud->push_back(p);
	}*/

	error /= N;
	return radius;
}

float MathTools::line_fitting(PointCloud<PointT>::Ptr c, float& a, float& b)
{
	double sum_x = 0.0f, sum_y = 0.0f;
	double sum_xy = 0.0f, sum_x2 = 0.0f;
	double sum_z = 0.0f;

	int N = c->points.size();
	for (int i = 0; i < N; i++)
	{
		double x = c->points[i].x;
		double y = c->points[i].y;
		sum_x += x;
		sum_y += y;
		sum_x2 += x * x;
		sum_xy += x * y;
		sum_z += c->points[i].z;
	}
	double avg_x = sum_x / N, avg_y = sum_y / N;
	double avg_xy = sum_xy / N, avg_x2 = sum_x2 / N;

	b = (avg_xy - avg_x * avg_y) / (avg_x2 - pow(avg_x, 2));
	a = avg_y - b * avg_x;

	float error = 0;
	for (int i = 0; i < N; i++)
	{
		double x = c->points[i].x;
		double y = c->points[i].y;
		error += abs(y - b * x - a) / sqrt(1 + b * b);
	}
	return error / N;
}

float MathTools::depth_cal(PointCloud<PointT>::Ptr c, pcl::octree::OctreePointCloudSearch<PointT>* oc, PointT point, std::vector<int>& PointIdx)
{
	if (oc->voxelSearch(point, PointIdx))			//Ö´ÐÐÌåËØ½üÁÚËÑË÷
	{
		float min_z = 10.0, max_z = -10.0;
		for (vector<int>::const_iterator idx = PointIdx.begin(); idx != PointIdx.end(); idx++)
		{
			if (c->points[*idx].z < min_z)
				min_z = c->points[*idx].z;
			if (c->points[*idx].z > max_z)
				max_z = c->points[*idx].z;
		}
		return max_z - min_z;
	}
	return 0;
}

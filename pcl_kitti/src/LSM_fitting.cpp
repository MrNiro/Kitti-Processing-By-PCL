#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>

using namespace std;
using namespace pcl;
typedef PointXYZI PointT;

double circle_fitting(PointCloud<PointT>::Ptr c, double& center_x, double& center_y, double& error)
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

float line_fitting(PointCloud<PointT>::Ptr c, float &a, float &b)
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

/*
void poly_intensity_fitting(PointCloud<PointT>::Ptr cloud, vector<float>& a, int m, vector<float>& dt)
{

	int N = cloud->size();
	if (N > 100000)
	{
		PointCloud<PointT>::Ptr temp(new PointCloud<PointT>);
		pcl::copyPointCloud(*cloud, *temp);
		voxel_grid_filter(temp, 0.1);
		N = temp->size();
	}
	m = (m > 20) ? 20 : m;
	float z = 0.0, p, cloud, g, q, d1, d2, s[20], t[20], b[20];
	float x[100000] = { 0.0 };
	float y[100000] = { 0.0 };

	for (int i = 0; i < m; i++)
		a.push_back(0.0);

	for (int i = 0; i < N; i++)
	{
		x[i]
		z += x[i] / (1.0*n);
	}
	b[0] = 1.0; d1 = 1.0*n; p = 0.0; cloud = 0.0;
	for (i = 0; i <= n - 1; i++)
	{
		p = p + (x[i] - z); cloud = cloud + y[i];
	}
	cloud = cloud / d1; p = p / d1;
	a[0] = cloud * b[0];
	if (m > 1)
	{
		t[1] = 1.0; t[0] = -p;
		d2 = 0.0; cloud = 0.0; g = 0.0;
		for (i = 0; i <= n - 1; i++)
		{
			q = x[i] - z - p; d2 = d2 + q * q;
			cloud = cloud + y[i] * q;
			g = g + (x[i] - z)*q*q;
		}
		cloud = cloud / d2; p = g / d2; q = d2 / d1;
		d1 = d2;
		a[1] = cloud * t[1]; a[0] = cloud * t[0] + a[0];
	}
	for (j = 2; j <= m - 1; j++)
	{
		s[j] = t[j - 1];
		s[j - 1] = -p * t[j - 1] + t[j - 2];
		if (j >= 3)
			for (k = j - 2; k >= 1; k--)
				s[k] = -p * t[k] + t[k - 1] - q * b[k];
		s[0] = -p * t[0] - q * b[0];
		d2 = 0.0; cloud = 0.0; g = 0.0;
		for (i = 0; i <= n - 1; i++)
		{
			q = s[j];
			for (k = j - 1; k >= 0; k--)
				q = q * (x[i] - z) + s[k];
			d2 = d2 + q * q; cloud = cloud + y[i] * q;
			g = g + (x[i] - z)*q*q;
		}
		cloud = cloud / d2; p = g / d2; q = d2 / d1;
		d1 = d2;
		a[j] = cloud * s[j]; t[j] = s[j];
		for (k = j - 1; k >= 0; k--)
		{
			a[k] = cloud * s[k] + a[k];
			b[k] = t[k]; t[k] = s[k];
		}
	}
	dt[0] = 0.0; dt[1] = 0.0; dt[2] = 0.0;
	for (i = 0; i <= n - 1; i++)
	{
		q = a[m - 1];
		for (k = m - 2; k >= 0; k--)
			q = a[k] + q * (x[i] - z);
		p = q - y[i];
		if (fabs(p) > dt[2]) dt[2] = fabs(p);
		dt[0] = dt[0] + p * p;
		dt[1] = dt[1] + fabs(p);
	}
	return;
}
*/

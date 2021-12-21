#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <fstream>
#include <string>
#include <io.h>

using namespace std;
using namespace pcl;
typedef PointXYZI PointT;

string All_file[500];

int get_filename(const char* to_search, int out_name = 0)
{
	struct _finddata_t fileinfo;
	intptr_t handle;
	handle = _findfirst(to_search, &fileinfo);
	All_file[0] = fileinfo.name;
	int num = 1;
	while (_findnext(handle, &fileinfo) == 0)
	{
		All_file[num] = fileinfo.name;
		if (out_name)
			cout << All_file[num] << endl;
		num++;
	}
	_findclose(handle);
	return num;
}

void load_bin_data(PointCloud<PointT>::Ptr c, string path, string filename)
{
	string infile = path + filename;
	fstream input(infile.c_str(), ios::in | ios::binary);
	if (!input.good()) {
		cerr << "Could not read file: " << infile << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);

	c->clear();
	for (int i = 0; input.good() && !input.eof(); i++) {
		PointT point;
		input.read((char *)&point.x, 3 * sizeof(float));
		input.read((char *)&point.intensity, sizeof(float));
		c->push_back(point);
	}
	input.close();

	cout << "Loaded " << c->size()
		<< " data points from " << filename << endl;
}

void load_pcd_data(PointCloud<PointT>::Ptr c, string path, string filename)
{
	c->clear();
	string infile = path + filename;
	cout << "Loading " << filename << " now..." << endl;
	if (pcl::io::loadPCDFile(infile, *c) < 0)
	{
		std::cout << "Error loading point cloud." << std::endl;
	}
	cout << "Loaded " << c->size()
		<< " data points from " << infile << endl;
}

void save_pcd_data_bin(PointCloud<PointT>::Ptr c, string path, string filename)
{
	pcl::io::savePCDFileBinary(path + filename, *c);
	cout << "Saved " << c->size() << " data points to " << path + filename << endl;
}

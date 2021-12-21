#include <CloudIO.h>


int CloudIO::get_filename(const char* to_search, vector<string>& filenames, bool silent)
{
	struct _finddata_t fileinfo;
	intptr_t handle;
	handle = _findfirst(to_search, &fileinfo);
	filenames.push_back(fileinfo.name);
	int num = 1;
	while (_findnext(handle, &fileinfo) == 0)
	{
		filenames.push_back(fileinfo.name);
		if (!silent) {
			cout << fileinfo.name << endl;
		}
		num++;
	}
	_findclose(handle);
	return num;
}

void CloudIO::load_bin_data(PointCloud<PointT>::Ptr c, string path, string filename)
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
		input.read((char*)&point.x, 3 * sizeof(float));
		input.read((char*)&point.intensity, sizeof(float));
		c->push_back(point);
	}
	input.close();

	cout << "Loaded " << c->size()
		<< " data points from " << filename << endl;
}

void CloudIO::load_pcd_data(PointCloud<PointT>::Ptr c, string path, string filename)
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

void CloudIO::save_pcd_data_bin(PointCloud<PointT>::Ptr c, string path, string filename)
{
	pcl::io::savePCDFileBinary(path + filename, *c);
	cout << "Saved " << c->size() << " data points to " << path + filename << endl;
}

bool load_move_step(string filename, vector<vector<float>>& move_rotate)
{
	move_rotate.clear();

	ifstream file(filename, ios::in);
	if (!file)
	{
		cout << filename << "is not exist!" << endl;
		return false;
	}

	std::vector<float> step;
	float t = 0;
	while (file >> t)
		step.push_back(t);

	int pos = 0;
	vector<float> temp_line;
	for (int i = 0; i < step.size(); i++)
	{
		temp_line.push_back(step[i]);
		pos++;
		if (pos == 6)
		{
			move_rotate.push_back(temp_line);
			pos = 0;
		}
	}

	file.close();
	return true;
}

void rotation(float alpha, float& x, float& y)
{
	float xx = cos(alpha) * x - sin(alpha) * y;
	float yy = sin(alpha) * x + cos(alpha) * y;

	x = xx;
	y = yy;
}

PointCloud<PointT>::Ptr CloudIO::joint_cloud(string type, string data_num, int first, int last, bool ifwrite)
{
	vector<vector<float>> move_rotate;
	if (!load_move_step("D:/Mr.Niro/School/graduation project/Data/2011_09_26_drive_00" + data_num + "_sync/oxts/move_step.txt",
		move_rotate))
		return nullptr;

	PointCloud<PointT>::Ptr cloud;
	PointCloud<PointT>::Ptr whole_cloud;
	cloud->clear();
	for (int i = first; i < last; i++)
	{
		if (type == "track")
		{
			PointT p0(1.0);
			cloud->push_back(p0);
		}
		else
		{
			string path, file;
			file = to_string(i) + ".pcd";
			path = "exp_data/data_" + data_num + "/" + type + "/";
			load_pcd_data(cloud, path, file);

			//if (type == "ground")
			//{
			//	cloud = pass_through(cloud, "intensity", 0.3, 1.0, 1, 1);
			//	cloud = pass_through(cloud, "x", -10, 10, 1, 1);
			//	cloud = pass_through(cloud, "y", -15, 15, 1, 1);
			//}
		}

		for (int idx = 0; idx < cloud->size(); idx++)
		{
			PointT p;
			p.x = cloud->points[idx].x;
			p.y = cloud->points[idx].y;
			p.z = cloud->points[idx].z;

			rotation(move_rotate[i][3], p.y, p.z);
			rotation(move_rotate[i][4], p.z, p.x);
			rotation(move_rotate[i][5], p.x, p.y);
			if (i > 0)
			{
				p.x += move_rotate[i][0];
				p.y += move_rotate[i][1];
				p.z += move_rotate[i][2];
			}
			p.intensity = cloud->points[idx].intensity;
			whole_cloud->push_back(p);
		}
		//cout << "currently cloud size: " << my_cloud->size() << endl;
		cloud->clear();
	}

	if (ifwrite)
	{
		save_pcd_data_bin(whole_cloud, "exp_data/data_" + data_num + "/",
			type + "_" + to_string(first) + "-" + to_string(last) + ".pcd");
		whole_cloud->clear();
	}
	return whole_cloud;
}

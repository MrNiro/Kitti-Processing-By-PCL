#ifndef CLOUDIO_H
#define CLOUDIO_H

#include <Config.h>

class CloudIO {

public:
    static int get_filename(const char* to_search, vector<string>& filenames, bool silent = true);

	static void load_bin_data(PointCloud<PointT>::Ptr c, string path, string filename);

	static void load_pcd_data(PointCloud<PointT>::Ptr c, string path, string filename);

	static void save_pcd_data_bin(PointCloud<PointT>::Ptr c, string path, string filename);

	static PointCloud<PointT>::Ptr joint_cloud(string type, string data_num, int first, int last, bool ifwrite = false);
};

#endif // !CLOUDIO_H

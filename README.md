# kitti-processing-with-pcl

(To be completed)

This is a Road Traffic Elements Extraction project for Vehicle Laser Point Cloud (KITTI dataset), using PCL(Point Cloud Lirary). 

In the main function, the processing pipeline are well designed for KITTI dataset, parameters are tested and set based on my best practices.

In addition to KITTI, there are several functions which can be easily invoked to process Point Cloud, including:

- Noise elimination
- Ground/Non-ground point cloud separation (Ground Pointclod Filter)
- Typical euclidean clustering
- Advanced euclidean clustering for close objects
- Extration of Pole, Road Boundary, Road Tag Line

利用PCL对KITTI数据进行处理与分析的工程，主要目标为道路交通要素的提取、分析与识别（杆状物，道路边界，道路标示线等）。针对KITTI数据集，在main函数中设计了处理流程，并基于个人实验尽可能选取了较好的参数。代码中对相关功能有较详细的注释



## Prerequisite
For Win64, the required libraries are included in the thirdParty folder(apart from boost)
```
    boost_1_74_0
    PCL 1.11.1
    Eigen
    FLANN
    OpenNI2
    Qhull
    VTK
```

Please contact me if you have any questions or suggestions.

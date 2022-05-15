# 使用Docker

## Pull from docker-hub (Option A)

```
$ docker pull dlopezmadrid/pcl-docker:latest
```

## Build it for yourself (Option B)

Build the image with

```
$ ./build_image.sh
```

## 运行

```bash
docker run -it --gpus all --name="pcl-docker" --cap-add sys_ptrace -p 127.0.0.1:2222:22 --user=pcl --volume=./example_project:/home/pcl/docker_dir/example_project dlopezmadrid/pcl-docker:latest
```



## 编译并运行项目

```bash
cd docker_dir/example_project/cloud_viewer   # 项目文件夹
mkdir build && cd build
cmake ..
make
```

在ROS下可以直接使用，相当于以上全部指令

```bash
catkin_make
```



# Basic Structures

```bash
cloud.width = 640; // there are 640 points per line
cloud.height = 480; // 表示有多少行

// 点云数据类
pcl::PointCloud<pcl::PointXYZ> cloud;
std::vector<pcl::PointXYZ> data = cloud.points;

// 检查是否有序
cloud.isOrganized ()

// 检查数据点的值是否为有限的(false)，若值为无限的则包括Inf/NaN
cloud.is_dense

```



## PCD文件

```powershell
# .PCD v.7 - Point Cloud Data file format
VERSION .7								# 版本号
FIELDS x y z rgb				#  指定一个点可以有的每一个维度和字段的名字
SIZE 4 4 4 4								# 用字节数指定每一个维度的大小。
TYPE F FFF									# 用一个字符指定每一个维度的类型 int uint folat
COUNT 1 1 1 1						# 指定每一个维度包含的元素数目，默认为1
WIDTH 640    						   # 像图像一样的有序结构，有640行和480列，
HEIGHT 480    					  # 这样该数据集中共有640*480=307200个点
# 1)它表示有序点云数据集的高度（行的总数）；
# 2)HEIGHT对于无序数据集它被设置成1（被用来检查一个数据集是有序还是无序）。
VIEWPOINT 0 0 0 1 0 0 0			# 指定数据集中点云的获取视点 视点信息被指定为平移（txtytz）+四元数（qwqxqyqz）
POINTS 307200						# 指定点云中点的总数。从0.7版本开始，该字段就有点多余了
DATA ascii										# 指定存储点云数据的数据类型。支持两种数据类型：ascii和二进制
0.93773 0.33763 0 4.2108e+06
0.90805 0.35641 0 4.2108e+06
```



### 读取PCD文件

```c++
#include <iostream>          //标准C++库中的输入输出的头文件
#include <pcl/io/pcd_io.h>   //PCD读写类相关的头文件
#include <pcl/point_types.h> //PCL中支持的点类型的头文件

int main(int argc, char **argv)
{
    //创建一个PointCloud<pcl::PointXYZ>    boost共享指针并进行实例化
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../test_pcd.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    //默认就是而二进制块读取转换为模块化的PointCLoud格式里pcl::PointXYZ作为点类型  然后打印出来
    std::cout << "Loaded "
              << cloud->width * cloud->height // 宽*高
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;
    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " " << cloud->points[i].y
                  << " " << cloud->points[i].z << std::endl;

    return (0);
}
```



### 将点云数据写入PCD文件

```c++
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main ()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width    = 5;
    cloud.height   = 1;
    cloud.is_dense = false;
    cloud.resize (cloud.width * cloud.height);
    for (auto& point: cloud)
    {
        // RAND_MAX是随机数能达到的最大值，这里相当于生成0-1的浮点数
        point.x = 1024 * rand () / (RAND_MAX + 1.0f);
        point.y = 1024 * rand () / (RAND_MAX + 1.0f);
        point.z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;

    for (const auto& point: cloud)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    return (0);
}
```

==cmake==

```cmake
cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(MY_GRAND_PROJECT)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcd_write pcd_write.cpp)  # 需要编译成可执行文件的源文件，空格之前是可执行文件的名字
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})
```

## **可用Point类型**

### sensor_msgs::PointCloud2（ROS点云数据）

==转成PointXYZI（PointT）==

```c++
void elevatedCloudCallback(const  sensor_msgs::PointCloud2::ConstPtr & input_cloud)  // PointCloud2
{
    // 将接收到的消息打印出来
    // ROS_INFO(  input_cloud->header);
    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>());   // input_cloud转换成pcl数据类型laser_cloud
    pcl::fromROSMsg(*input_cloud, *output_cloud);//转换成pcl数据类型
    std::cout <<output_cloud->header<< std::endl;
    std::cout << output_cloud->points[1].x<< std::endl;
}
```

其格式为：

```
$ rostopic echo /airsim_node/Drone0/lidar/LidarSensor1
header: 
  seq: 2116
  stamp: 
    secs: 1586919439
    nsecs: 448866652
  frame_id: "LidarSensor1"
height: 1
width: 3
fields: 
  - 
    name: "x"
    offset: 0
    datatype: 7
    count: 1
  - 
    name: "y"
    offset: 4
    datatype: 7
    count: 1
  - 
    name: "z"
    offset: 8
    datatype: 7
    count: 1
is_bigendian: False
point_step: 12
row_step: 36
data: [143, 194, 117, 53, 10, 215, 163, 53, 222, 238, 165, 64, 143, 194, 117, 53, 10, 215, 163, 53, 222, 238, 165, 64, 143, 194, 117, 53, 10, 215, 163, 53, 222, 238, 165, 64]
is_dense: True
```

### PointXYZ–成员变量: float x, y, z

PointXYZ是使用最常见的一个点数据类型，因为它只包含三维xyz坐标信息，这三个浮点数附加一个浮点数来满足存储对齐，用户可利用points[i].data[0]，或者points[i].x访问点的x坐标值。

```
union
{
    float data[4];
    struct
    {
        float x;
        float y;
        float z;
    };
};
```

### PointXYZI–成员变量: float x, y, z, intensity

PointXYZI是一个简单的XYZ坐标加intensity的point类型，理想情况下，这四个变量将新建单独一个结构体，并且满足存储对齐，然而，由于point的大部分操作会把data[4]元素设置成0或1（用于变换），不能让intensity与xyz在同一个结构体中，如果这样的话其内容将会被覆盖。例如，两个点的点积会把他们的第四个元素设置成0，否则该点积没有意义，等等。因此，对于兼容存储对齐，用三个额外的浮点数来填补intensity，这样在存储方面效率较低，但是符合存储对齐要求，运行效率较高。

```
union
{
    float data[4];
    struct
    {
        float x;
        float y;
        float z;
    };
};
union
{
    struct
    {
        float intensity;
    };
    float data_c[4];
};
```

### PointXYZRGBA

成员变量: float x, y, z; uint32_t rgba


除了rgba信息被包含在一个整型变量中，其它的和PointXYZI类似。

```
union
{
    float data[4];
    struct
    {
        float x;
        float y;
        float z;
    };
};
union
{
    struct
    {
        uint32_t rgba;
    };
    float data_c[4];
};

```

* **PointXYZRGB - float x, y, z, rgb;** <span id = "PointXYZRGB"/>

```
union
{
    float data[4];
    struct
    {
        float x;
        float y;
        float z;
    };
};
union
{
    struct
    {
        float rgb;
    };
    float data_c[4];
};
```

### PointXY-float x, y

简单的二维x-y point结构

```
struct
{
    float x;
    float y;
};
```

### **InterestPoint-float x, y, z, strength;**

除了strength表示关键点的强度的测量值，其它的和PointXYZI类似。

```
union
{
    float data[4];
    struct
    {
        float x;
        float y;
        float z;
    };
};
union
{
    struct
    {
        float strength;
    };
    float data_c[4];
};
```

### **Normal - float normal[3], curvature;**

另一个最常用的数据类型，Normal结构体表示给定点所在样本曲面上的法线方向，以及对应曲率的测量值（通过曲面块特征值之间关系获得——查看NormalEstimation类API以便获得更多信息，后续章节有介绍），由于在PCL中对曲面法线的操作很普遍，还是用第四个元素来占位，这样就兼容SSE和高效计算，例如，用户访问法向量的第一个坐标，可以通过points[i].data\_n[0]或者points[i].normal[0]或者points[i].normal\_x，再一次强调，曲率不能被存储在同一个结构体中，因为它会被普通的数据操作覆盖掉。

```
union
{
    float data_n[4];
    float normal[3];
    struct
    {
        float normal_x;
        float normal_y;
        float normal_z;
    };
}
union
{
    struct
    {
        float curvature;
    };
    float data_c[4];
};
```

### **PointNormal - float x, y, z; float normal[3], curvature;**


PointNormal是存储XYZ数据的point结构体，并且包括采样点对应法线和曲率。

```
union
{
    float data[4];
    struct
    {
        float x;
        float y;
        float z;
    };
};
union
{
    float data_n[4];
    float normal[3];
    struct
    {
        float normal_x;
        float normal_y;
        float normal_z;
    };
};
union
{
    struct
    {
        float curvature;
    };
    float data_c[4];
};
```

### **PointXYZRGBNormal - float x, y, z, rgb, normal[3], curvature;**


PointXYZRGBNormal存储XYZ数据和RGB颜色的point结构体，并且包括曲面法线和曲率

```
union
{
    float data[4];
    struct
    {
        float x;
        float y;
        float z;
    };
};
union
{
    float data_n[4];
    float normal[3];
    struct
    {
        float normal_x;
        float normal_y;
        float normal_z;
    };
}
union
{
    struct
    {
        float rgb;
        float curvature;
    };
    float data_c[4];
};

```

### **PointXYZINormal - float x, y, z, intensity, normal[3], curvature;**

PointXYZINormal存储XYZ数据和强度值的point结构体，并且包括曲面法线和曲率。

```
union
{
    float data[4];
    struct
    {
        float x;
        float y;
        float z;
    };
};
union
{
    float data_n[4];
    float normal[3];
    struct
    {
        float normal_x;
        float normal_y;
        float normal_z;
    };
}
union
{
    struct
    {
        float intensity;
        float curvature;
    };
    float data_c[4];
};
```

## Point转换和使用

### **pcl::PointXYZI转换成pcl::PointXYZ**

```c++
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
  // keypoints_Harris( cloud_xyz, keypoints);

  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>); 
  // 将原点云的字段复制进去
  copyPointCloud (*keypoints , *keypoints_ptr); 
```

### pcl::PointCloud::Ptr和pcl::PointCloud的两个类相互转换

```c++
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> cloud;
cloud = *cloudPtr;//由Ptr转变为另一种类型
cloudPtr = cloud.makeShared();//转变为Ptr类型
```

### pcl::PointCloud::Ptr赋值

```c++
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
for (int i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
```





## KdTree

```c++
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>

int
main ()
{
  // 设置随机数种子（以当前时间为种子）
  srand (time (NULL));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Generate pointcloud data
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  // 初始化，std::size_t为了匹配多平台
  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    (*cloud)[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    (*cloud)[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }

  // 将已有的点云构建KdTree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);
  
  // 初始化一个随机点
  pcl::PointXYZ searchPoint;
  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  // K nearest neighbor search

  int K = 10;

  std::vector<int> pointIdxKNNSearch(K);
  std::vector<float> pointKNNSquaredDistance(K);

  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;
    
  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
  {
    for (std::size_t i = 0; i < pointIdxKNNSearch.size (); ++i)
      std::cout << "    "  <<   (*cloud)[ pointIdxKNNSearch[i] ].x 
                << " " << (*cloud)[ pointIdxKNNSearch[i] ].y 
                << " " << (*cloud)[ pointIdxKNNSearch[i] ].z 
                << " (squared distance: " << pointKNNSquaredDistance[i] << ")" << std::endl;
  }

  // Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

  std::cout << "Neighbors within radius search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;


  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      std::cout << "    "  <<   (*cloud)[ pointIdxRadiusSearch[i] ].x 
                << " " << (*cloud)[ pointIdxRadiusSearch[i] ].y 
                << " " << (*cloud)[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  }


  return 0;
}
```



## Octree

3

## Random Sample Consensus model

```c++
#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h> // for PointCloud
#include <pcl/common/io.h> // for copyPointCloud
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

int
main(int argc, char** argv)
{
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

  // populate our PointCloud with points
  cloud->width    = 500;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for (pcl::index_t i = 0; i < static_cast<pcl::index_t>(cloud->size ()); ++i)
  {
    if (pcl::console::find_argument (argc, argv, "-s") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    {
      (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if (i % 5 == 0)
        (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else if(i % 2 == 0)
        (*cloud)[i].z =  sqrt( 1 - ((*cloud)[i].x * (*cloud)[i].x)
                                      - ((*cloud)[i].y * (*cloud)[i].y));
      else
        (*cloud)[i].z =  - sqrt( 1 - ((*cloud)[i].x * (*cloud)[i].x)
                                        - ((*cloud)[i].y * (*cloud)[i].y));
    }
    else
    {
      (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if( i % 2 == 0)
        (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else
        (*cloud)[i].z = -1 * ((*cloud)[i].x + (*cloud)[i].y);
    }
  }

  std::vector<int> inliers;

  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
  if(pcl::console::find_argument (argc, argv, "-f") >= 0)
  {
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }
  else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 )
  {
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    //  
    ransac.getInliers(inliers);
  }

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud (*cloud, inliers, *final);

  // creates the visualization object and adds either our original cloud or all of the inliers
  // depending on the command line arguments specified.
  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    viewer = simpleVis(final);
  else
    viewer = simpleVis(cloud);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  return 0;
 }
```


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("data/test_cloud.pcd", *cloud) == -1)
    {
        PCL_ERROR("无法读取点云文件\n");
        return -1;
    }
    std::cout << "点云加载成功，点数: " << cloud->points.size() << std::endl;

    pcl::visualization::PCLVisualizer viewer("点云查看器");
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
    return 0;
}

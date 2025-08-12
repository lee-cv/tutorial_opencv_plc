PCL 初学阶段学习计划
1. 安装与环境配置
确认 PCL 已正确安装（Ubuntu 推荐用 apt 或源码编译）

配置 CMake，确保能正确找到 PCL

编写第一个简单程序，读取并显示点云文件

2. 点云基础操作
读取和保存点云（PCD/PLY格式）

点云数据结构了解（pcl::PointXYZ、pcl::PointCloud）

遍历访问点云数据

3. 点云可视化
使用 pcl::visualization::PCLVisualizer 展示点云

添加点云颜色、坐标系、文本标注等

4. 点云滤波
体素网格滤波（VoxelGrid）：降采样

直通滤波（PassThrough）：裁剪区域

条件滤波（ConditionalRemoval）

5. 点云法线估计
计算点云法线（NormalEstimation）

调整参数对结果影响

6. 点云分割
RANSAC 平面分割（SACSegmentation）

通过平面模型提取感兴趣区域

7. 点云聚类
欧式聚类（EuclideanClusterExtraction）

识别点云中不同物体

8. 点云配准
ICP（IterativeClosestPoint）基础

点云配准思想及应用
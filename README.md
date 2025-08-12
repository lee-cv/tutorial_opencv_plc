# PCL & OpenCV 系统学习路线 — tutorial_opencv_pcl

欢迎使用本项目作为学习 PCL（点云库）和 OpenCV（计算机视觉库）的教程和代码示范仓库。  
本 README 文件整理了系统化学习路线，方便按照步骤学习、实践并结合项目实际应用。

---

## 目录结构简介

- `src/opencv_tutorial/` ：OpenCV 基础及进阶示例代码  
- `src/pcl_tutorial/` ：PCL 点云处理示例代码  
- `src/combined_opencv_pcl/` ：OpenCV 与 PCL 结合应用示例  
- `include/` ：公共工具函数头文件  
- `data/` ：示例图片和点云数据  
- `docs/` ：学习笔记与心得文档  

---

## 学习路线

### 一、OpenCV 部分

1. **基础图像操作**  
   - 读写图像（`imread`，`imwrite`）  
   - 显示窗口（`imshow`，`waitKey`）  
   - 像素访问，ROI（感兴趣区域）操作  
   - 通道分离与合并  

2. **绘图与几何变换**  
   - 绘制基本图形：线条、圆、矩形、文字等（`line`, `circle`, `rectangle`, `putText`）  
   - 图像缩放、旋转、平移  
   - 仿射变换与透视变换  

3. **滤波与边缘检测**  
   - 均值滤波、高斯滤波、中值滤波  
   - 边缘检测：Sobel、Laplacian、Canny算子  

4. **特征点检测与匹配**  
   - Harris、Shi-Tomasi角点检测  
   - ORB、SIFT、SURF等描述子（注意 SIFT/SURF 需要非自由许可）  

5. **摄像机模型与标定**  
   - 相机内参标定与畸变校正（`calibrateCamera`）  
   - 了解相机坐标系和像素坐标系转换  

6. **视频与实时处理**  
   - 视频读取与显示（`VideoCapture`）  
   - 实时图像处理框架搭建  

---

### 二、PCL 部分

1. **点云 I/O**  
   - 读取与保存 PCD、PLY 文件（`pcl::io::loadPCDFile`，`pcl::io::savePCDFile`）  

2. **点云可视化**  
   - 使用 `pcl::visualization::PCLVisualizer` 进行点云展示  

3. **滤波器**  
   - 体素网格滤波（`VoxelGrid`）  
   - 直通滤波（`PassThrough`）  
   - 条件滤波  

4. **法线估计**  
   - `pcl::NormalEstimation` 的使用  

5. **分割与聚类**  
   - RANSAC 平面分割（`SACSegmentation`）  
   - 欧式聚类（`EuclideanClusterExtraction`）  

6. **点云配准**  
   - ICP 算法（`IterativeClosestPoint`）  
   - 特征匹配（FPFH特征等）  

7. **高级点云处理**  
   - 表面重建  
   - 深度图转点云  

---

### 三、OpenCV 与 PCL 结合应用示例

- RGB-D 图像到点云转换  
- 使用图像分割结果做点云区域筛选  
- 点云投影回图像，实现点云与图像的联合分析  

---

## 如何使用本项目

1. 按照目录中的分模块示例学习代码，运行、调试，理解每个示例的输入输出及核心函数。  
2. 编写并添加你自己的学习代码，保持代码风格和目录结构一致。  
3. 在 `docs/` 目录写学习笔记和心得，结合代码示例附截图与说明。  
4. 如遇问题，可以随时查阅官方文档或网络资料，项目中遇到的关键代码也可保存并整理。  
5. 后续在教学或演示时，可直接使用本项目示例，方便快捷。  

---

## 推荐资源

- OpenCV 官方文档: https://docs.opencv.org  
- PCL 官方文档: https://pointclouds.org/documentation/  
- OpenCV 教程（中英文均有）  
- PCL GitHub 及示例  

---

祝你学习顺利！  
欢迎持续完善本项目，做成你自己的学习宝典！

---


cmake_minimum_required(VERSION 3.10)
project(tutorial_opencv_pcl)  # 你这里拼写我帮你修正了

# 设置默认构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# OpenCV 配置
set(OpenCV_DIR /home/lee/Cpp/template_utils/cmakeTemplate/3rdparty/opencv-cuda)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# PCL配置
find_package(PCL REQUIRED COMPONENTS common io filters segmentation surface visualization) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# 头文件目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 主程序，可选保留
add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/temp.cpp
)

# PCL 01_io 示例程序
add_executable(pcl_io_tutorial
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pcl_tutorial/01_io.cpp
)

# 设置 C++17 标准
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
set_property(TARGET pcl_io_tutorial PROPERTY CXX_STANDARD 17)

# 链接库
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(pcl_io_tutorial ${PCL_LIBRARIES})

cmake_minimum_required(VERSION 3.0)
project(simple_camera)

# 查找OpenCV包
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加项目本地inc文件夹
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)

# 添加相机SDK的库文件路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 查看SDK库是否存在于项目目录中
file(GLOB SDK_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/lib/*.so)
if(SDK_LIBS)
  message(STATUS "Found SDK libraries: ${SDK_LIBS}")
endif()

# 添加所有源文件到可执行文件
add_executable(simple_camera 
    src/main.cpp
    src/simple_camera.cpp
    src/camera_ui.cpp
    src/config_manager.cpp
)

# 将库路径添加到编译标志（有助于查找库）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,-rpath,${CMAKE_CURRENT_SOURCE_DIR}/lib")

# 链接OpenCV库和相机SDK库
# 直接链接到MVSDK库
target_link_libraries(simple_camera ${OpenCV_LIBS} MVSDK)

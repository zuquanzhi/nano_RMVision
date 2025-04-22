cmake_minimum_required(VERSION 3.0)
project(simple_camera)

# 查找OpenCV包
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
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

# Set the build type to Release for optimization
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Enable compiler optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

# Enable parallel compilation if supported
if(CMAKE_GENERATOR MATCHES "Makefiles|Ninja")
    set(CMAKE_BUILD_FLAGS "-j$(nproc)")
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
target_link_libraries(simple_camera PRIVATE ${OpenCV_LIBS} MVSDK openvino::runtime)

cmake_minimum_required(VERSION 3.22)
project(hikcamera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV 4.5 REQUIRED)

# 把 sdk 的库目录加进来
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/lib)

set(PROJECT_SOURCES
  src/image_capturer.cpp
)

# 编译成一个库 target
ament_auto_add_library(${PROJECT_NAME} SHARED ${PROJECT_SOURCES})

# BUILD_INTERFACE 可以让本地开发时找到头文件
# INSTALL_INTERFACE 可以让 install 后的包也把 include/ 目录加进来
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/sdk/include>
    $<INSTALL_INTERFACE:include>
)

# 链接 OpenCV 和 SDK 库
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  MvCameraControl
)

# —— 新增：把 SDK 的头文件安装到 ${prefix}/include —— 
install(
  DIRECTORY src/sdk/include/
  DESTINATION include
)

# 把 SDK 的 .so 也安装到 ${prefix}/lib
install(
  DIRECTORY src/sdk/lib/
  DESTINATION lib
)

ament_auto_package()

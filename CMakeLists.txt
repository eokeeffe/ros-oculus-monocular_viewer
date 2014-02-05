cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(oculus_monocular_fpv src/fp_viewer.cpp)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(LibOVR/Include)
include_directories(LibOVR/Src)
include_directories(LibOVR/Src/Kernel)
include_directories(LibOVR/Src/Util)
include_directories(CommonSrc/Platform)
include_directories(CommonSrc/Render)
link_directories(LibOVR/Lib/Linux/Release/x86_64)

# Oculus FPV
#External Controllers
rosbuild_add_library(Platform_Default CommonSrc/Platform/Platform.cpp)
rosbuild_add_library(Linux_Platform CommonSrc/Platform/Linux_Platform.cpp)
rosbuild_add_library(Linux_Gamepad CommonSrc/Platform/Linux_Gamepad.cpp)
rosbuild_add_library(Render_Device CommonSrc/Render/Render_Device.cpp)
rosbuild_add_library(Render_GL_Device CommonSrc/Render/Render_GL_Device.cpp)

#-DGL_GLEXT_PROTOTYPES
rosbuild_add_compile_flags(Linux_Platform -DGL_GLEXT_PROTOTYPES)
rosbuild_add_compile_flags(Render_GL_Device -DGL_GLEXT_PROTOTYPES)
rosbuild_add_compile_flags(oculus_monocular_fpv -DGL_GLEXT_PROTOTYPES)

target_link_libraries(Platform_Default ovr GL X11 Xinerama)
target_link_libraries(Linux_Platform ovr GL X11 Xinerama)
target_link_libraries(Linux_Gamepad ovr GL X11 Xinerama)
target_link_libraries(Render_Device ovr GL X11 Xinerama)
target_link_libraries(Render_GL_Device ovr GL X11 Xinerama)
target_link_libraries(Linux_Platform Render_Device Render_GL_Device Linux_Gamepad Platform_Default)

target_link_libraries(oculus_monocular_fpv ovr udev pthread GL X11 Xinerama  Platform_Default Linux_Gamepad Render_Device Render_GL_Device Linux_Platform ${OpenCV_LIBRARIES})

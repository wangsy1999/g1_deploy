cmake_minimum_required(VERSION 3.16)
project(g1_deploy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(COMMON_INCLUDE "${REPOSITORY_ROOT}/common/include")
set(TENSORRT_ROOT "$ENV{TENSORRT_ROOT}" CACHE PATH "TensorRT SDK install prefix")
set(CUDAToolkit_ROOT "$ENV{CUDAToolkit_ROOT}" CACHE PATH "CUDA Toolkit install prefix")
find_path(TENSORRT_INCLUDE_DIR NvInfer.h HINTS "${TENSORRT_ROOT}"
    PATH_SUFFIXES include "include/${CMAKE_LIBRARY_ARCHITECTURE}")
find_path(TENSORRT_ONNX_INCLUDE_DIR NvOnnxParser.h HINTS "${TENSORRT_ROOT}"
    PATH_SUFFIXES include "include/${CMAKE_LIBRARY_ARCHITECTURE}")
find_library(TENSORRT_LIBRARY nvinfer HINTS "${TENSORRT_ROOT}"
    PATH_SUFFIXES lib lib64 "lib/${CMAKE_LIBRARY_ARCHITECTURE}")
find_library(TENSORRT_ONNX_LIBRARY nvonnxparser HINTS "${TENSORRT_ROOT}"
    PATH_SUFFIXES lib lib64 "lib/${CMAKE_LIBRARY_ARCHITECTURE}")
if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_ONNX_INCLUDE_DIR OR
   NOT TENSORRT_LIBRARY OR NOT TENSORRT_ONNX_LIBRARY)
    message(FATAL_ERROR "Install TensorRT 8.5–10.x C++ and ONNX parser development files or set TENSORRT_ROOT")
endif()

# FindCUDAToolkit was added in CMake 3.17. Keep system CMake 3.16 usable on Ubuntu 20.04.
find_path(G1_CUDA_INCLUDE_DIR cuda_runtime_api.h HINTS "${CUDAToolkit_ROOT}" /usr/local/cuda
    PATH_SUFFIXES include "targets/${CMAKE_SYSTEM_PROCESSOR}-linux/include" targets/sbsa-linux/include)
find_library(G1_CUDART_LIBRARY cudart HINTS "${CUDAToolkit_ROOT}" /usr/local/cuda
    PATH_SUFFIXES lib64 lib "targets/${CMAKE_SYSTEM_PROCESSOR}-linux/lib" targets/sbsa-linux/lib)
if(NOT G1_CUDA_INCLUDE_DIR OR NOT G1_CUDART_LIBRARY)
    message(FATAL_ERROR "Install CUDA Toolkit development files or set CUDAToolkit_ROOT")
endif()
add_library(g1_tensorrt STATIC common/src/g1_tensorrt.cpp)
target_include_directories(g1_tensorrt PUBLIC "${COMMON_INCLUDE}")
target_include_directories(g1_tensorrt SYSTEM PRIVATE
    "${TENSORRT_INCLUDE_DIR}" "${TENSORRT_ONNX_INCLUDE_DIR}" "${G1_CUDA_INCLUDE_DIR}")
target_link_libraries(g1_tensorrt PRIVATE "${TENSORRT_LIBRARY}" "${TENSORRT_ONNX_LIBRARY}" "${G1_CUDART_LIBRARY}")
find_package(Threads REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/usr/local" "/opt/unitree_robotics")
find_package(unitree_sdk2 REQUIRED)
get_target_property(SDK_LIBRARY unitree_sdk2 IMPORTED_LOCATION)
if(NOT SDK_LIBRARY)
    get_target_property(SDK_LIBRARY unitree_sdk2 IMPORTED_LOCATION_RELEASE)
endif()
if(NOT SDK_LIBRARY)
    message(FATAL_ERROR "The installed unitree_sdk2 package must expose its native library location")
endif()
get_filename_component(SDK_RUNTIME_DIR "${SDK_LIBRARY}" DIRECTORY)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc photo)

# Native camera support is detected automatically. Robot builds require it;
# workstations without the SDK can still run simulation and offline policy checks.
option(G1_REQUIRE_REALSENSE "Fail configuration when native RealSense SDK is missing" OFF)
find_package(realsense2 CONFIG QUIET)
if(G1_REQUIRE_REALSENSE AND NOT realsense2_FOUND)
    message(FATAL_ERROR "Install librealsense2 development files or set realsense2_DIR")
endif()
add_library(g1_depth_camera STATIC apps/realsense/src/g1_depth_camera.cpp)
target_include_directories(g1_depth_camera PUBLIC apps/realsense/include)
target_link_libraries(g1_depth_camera PUBLIC Threads::Threads)
if(realsense2_FOUND)
    target_compile_definitions(g1_depth_camera PRIVATE G1_WITH_REALSENSE)
    if(TARGET realsense2::realsense2)
        target_link_libraries(g1_depth_camera PUBLIC realsense2::realsense2)
    elseif(TARGET realsense2)
        target_link_libraries(g1_depth_camera PUBLIC realsense2)
    else()
        target_include_directories(g1_depth_camera PRIVATE ${realsense2_INCLUDE_DIR})
        target_link_libraries(g1_depth_camera PUBLIC ${realsense2_LIBRARY})
    endif()
else()
    message(STATUS "RealSense SDK not found: simulation/offline only; hardware camera startup will report an error")
endif()
add_executable(g1_camera_check apps/realsense/src/camera_check.cpp)
target_link_libraries(g1_camera_check PRIVATE g1_depth_camera)
set_target_properties(g1_camera_check PROPERTIES INSTALL_RPATH "$ORIGIN/../lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS g1_camera_check RUNTIME DESTINATION bin)

add_executable(g1_dwaq
    "${REPOSITORY_ROOT}/apps/dwaq/src/main.cpp"
    "${REPOSITORY_ROOT}/apps/dwaq/src/options.cpp"
    "${REPOSITORY_ROOT}/apps/dwaq/src/controller.cpp")
target_include_directories(g1_dwaq PRIVATE "${REPOSITORY_ROOT}/apps/dwaq/include")
target_link_libraries(g1_dwaq PRIVATE g1_tensorrt unitree_sdk2 Boost::boost yaml-cpp Threads::Threads)
target_compile_definitions(g1_dwaq PRIVATE G1_WITH_POLICY
    G1_POLICY_DEFAULT_CONFIG="${REPOSITORY_ROOT}/apps/dwaq/config/g1_locomotion.json")
add_executable(g1_parkour
    "${REPOSITORY_ROOT}/apps/parkour/src/main.cpp"
    "${REPOSITORY_ROOT}/apps/parkour/src/options.cpp"
    "${REPOSITORY_ROOT}/apps/parkour/src/controller.cpp")
target_include_directories(g1_parkour PRIVATE "${REPOSITORY_ROOT}/apps/parkour/include" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(g1_parkour PRIVATE g1_tensorrt g1_depth_camera unitree_sdk2 Boost::boost yaml-cpp ${OpenCV_LIBS} Threads::Threads)
target_compile_definitions(g1_parkour PRIVATE
    G1_PARKOUR_DEFAULT_CONFIG="${REPOSITORY_ROOT}/apps/parkour/config/g1_parkour.json")
add_executable(g1_hybrid
    apps/hybrid/src/main.cpp apps/hybrid/src/options.cpp apps/hybrid/src/controller.cpp
    apps/parkour/src/options.cpp apps/parkour/src/controller.cpp)
target_include_directories(g1_hybrid PRIVATE apps/hybrid/include apps/dwaq/include apps/parkour/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(g1_hybrid PRIVATE g1_tensorrt g1_depth_camera unitree_sdk2 Boost::boost yaml-cpp ${OpenCV_LIBS} Threads::Threads)
target_compile_definitions(g1_hybrid PRIVATE
    G1_PARKOUR_DEFAULT_CONFIG="${REPOSITORY_ROOT}/apps/hybrid/config/g1_hybrid.json")
set_target_properties(g1_dwaq g1_parkour g1_hybrid PROPERTIES
    INSTALL_RPATH "$ORIGIN/../lib;${SDK_RUNTIME_DIR}"
    INSTALL_RPATH_USE_LINK_PATH TRUE)
if(UNIX AND NOT APPLE)
    target_link_options(g1_dwaq PRIVATE "-Wl,--disable-new-dtags")
    target_link_options(g1_parkour PRIVATE "-Wl,--disable-new-dtags")
    target_link_options(g1_hybrid PRIVATE "-Wl,--disable-new-dtags")
endif()

install(TARGETS g1_dwaq g1_parkour g1_hybrid RUNTIME DESTINATION bin)
install(FILES apps/hybrid/config/g1_hybrid.json DESTINATION apps/hybrid/config)
install(PROGRAMS apps/hybrid/hybrid.sh DESTINATION . RENAME hybrid)
install(DIRECTORY "${REPOSITORY_ROOT}/apps/parkour/models" DESTINATION apps/parkour)
install(FILES "${REPOSITORY_ROOT}/apps/parkour/config/g1_parkour.json" "${REPOSITORY_ROOT}/apps/parkour/config/realsense_parkour.json" DESTINATION apps/parkour/config)
install(FILES "${REPOSITORY_ROOT}/apps/realsense/realsense_web.py" "${REPOSITORY_ROOT}/apps/realsense/requirements.txt" DESTINATION apps/realsense)
install(FILES "${REPOSITORY_ROOT}/apps/realsense/static/realsense.html" DESTINATION apps/realsense/static)
install(PROGRAMS "${REPOSITORY_ROOT}/apps/realsense/realsense.sh" DESTINATION . RENAME realsense)
install(PROGRAMS "${REPOSITORY_ROOT}/apps/parkour/parkour.sh" DESTINATION . RENAME parkour)
install(FILES "${REPOSITORY_ROOT}/docs/PARKOUR.md" DESTINATION docs)
install(DIRECTORY "${REPOSITORY_ROOT}/apps/dwaq/models" DESTINATION apps/dwaq)
install(FILES "${REPOSITORY_ROOT}/apps/dwaq/config/g1_locomotion.json" DESTINATION apps/dwaq/config)
install(PROGRAMS "${REPOSITORY_ROOT}/g1" DESTINATION .)
install(PROGRAMS "${REPOSITORY_ROOT}/apps/monitor/monitor.sh" DESTINATION . RENAME monitor)
install(FILES "${REPOSITORY_ROOT}/apps/monitor/joint_monitor.py" "${REPOSITORY_ROOT}/apps/monitor/requirements.txt" DESTINATION apps/monitor)
install(FILES "${REPOSITORY_ROOT}/apps/monitor/static/joint_monitor.html" DESTINATION apps/monitor/static)
install(FILES "${REPOSITORY_ROOT}/README.md" "${REPOSITORY_ROOT}/ORIGIN.md" "${REPOSITORY_ROOT}/CHANGELOG.md" DESTINATION .)
install(FILES "${REPOSITORY_ROOT}/docs/HARDWARE.md" DESTINATION docs)
install(FILES "${REPOSITORY_ROOT}/docs/WEB.md" DESTINATION docs)
install(FILES "${REPOSITORY_ROOT}/LICENSE" DESTINATION licenses/project)
configure_file("${REPOSITORY_ROOT}/cmake/BUILD_INFO.in" "${CMAKE_CURRENT_BINARY_DIR}/BUILD_INFO.txt" @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/BUILD_INFO.txt" DESTINATION .)
set(CPACK_GENERATOR TGZ)
set(CPACK_PACKAGE_NAME "g1-deploy")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_FILE_NAME "g1-deploy-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")
include(CPack)

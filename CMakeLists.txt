cmake_minimum_required(VERSION 3.19)
project(CamIMUCalib)

set(CMAKE_CXX_STANDARD 14)
set(use_hk true)

find_package(OpenCV 4 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

link_directories(${PROJECT_SOURCE_DIR}/lib/Camera)

# link_libraries用在add_executable之前，target_link_libraries用在add_executable之后
if (use_hk)
    message(STATUS "Linking HKROBOT camera lib on linux...")
    add_definitions("-DUSE_HK")
    link_libraries(-lMvCameraControl)
endif ()

file (GLOB_RECURSE camfiles
    "${PROJECT_SOURCE_DIR}/src/Camera/hk_cam_wrapper.cpp"
)
file (GLOB_RECURSE serialfiles
    "src/com/**/*.cc"
    "src/com/**/*.c"
    "src/com/**/*.cpp"
    "src/com/*.cpp"
)
file (GLOB_RECURSE calibfiles
    "src/CamIMUCalib/*.cpp"
)


add_executable(CamIMUCalib CamIMUCalib.cpp ${calibfiles})
add_executable(get_calib_data get_calib_data.cpp ${camfiles} ${serialfiles})
add_executable(Display Samples/Display.cpp)
add_executable(SetParam Samples/SetParam.cpp)
add_executable(serial_example Samples/serial_example.cc ${serialfiles})

target_link_libraries(get_calib_data ${OpenCV_LIBS} -lglog -lpthread)
target_link_libraries(CamIMUCalib ${OpenCV_LIBS})
target_link_libraries(Display X11)
target_link_libraries(SetParam X11)








cmake_minimum_required(VERSION 3.10)
project(BezierCurve)

# find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(./../../)
include_directories(./../../opencv/include)
file(GLOB OpenCV_LIBRARIES ${PROJECT_SOURCE_DIR}/../../opencv/x64/mingw/lib/*.dll.a)
file(GLOB dllfiles ${PROJECT_SOURCE_DIR}/../../opencv/x64/mingw/bin/*.dll)
# set(OpenCV_LIBRARIES ${PROJECT_SOURCE_DIR}/../opencv/x64/mingw/lib/*.dll.a)
file(COPY ${dllfiles} DESTINATION ./)

add_executable(BezierCurve main.cpp)

target_link_libraries(BezierCurve ${OpenCV_LIBRARIES})

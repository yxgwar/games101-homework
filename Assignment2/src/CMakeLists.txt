cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

# find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

option(SS "" OFF)

if (SS)
    add_definitions(-DSS)
endif (SS)

include_directories(./../../)
include_directories(./../../opencv/include)
file(GLOB OpenCV_LIBRARIES ${PROJECT_SOURCE_DIR}/../../opencv/x64/mingw/lib/*.dll.a)
file(GLOB dllfiles ${PROJECT_SOURCE_DIR}/../../opencv/x64/mingw/bin/*.dll)
# set(OpenCV_LIBRARIES ${PROJECT_SOURCE_DIR}/../opencv/x64/mingw/lib/*.dll.a)
file(COPY ${dllfiles} DESTINATION ./)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})

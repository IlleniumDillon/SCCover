cmake_minimum_required(VERSION 3.0)
project(proto)

set(CMAKE_CXX_STANDARD 17)

include(${CMAKE_CURRENT_SOURCE_DIR}/ros2support.cmake)

FIND_PACKAGES(
    OpenCV
    Eigen3
)

IMPORT_DIR(
    ${CMAKE_CURRENT_SOURCE_DIR}/Cover
    COVER_SRC
    COVER_DIR
)
IMPORT_DIR(
    ${CMAKE_CURRENT_SOURCE_DIR}/FileIO
    FILEIO_SRC
    FILEIO_DIR
)

include_directories(${FILEIO_DIR} ${COVER_DIR})

set(MAPFILE_PATH "${CMAKE_SOURCE_DIR}/map.png")
add_compile_definitions(MAPFILE_PATH="${MAPFILE_PATH}")

set(DATASET_PATH "${CMAKE_SOURCE_DIR}/indoor_dataset")
add_compile_definitions(DATASET_PATH="${DATASET_PATH}")

add_executable(proto main.cpp ${COVER_SRC} ${FILEIO_SRC})
target_link_libraries(proto ${OpenCV_LIBS} ${Eigen3_LIBS})
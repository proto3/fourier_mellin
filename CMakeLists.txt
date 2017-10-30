cmake_minimum_required(VERSION 2.8)
project(FMT_demo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

################################################################################
#PACKAGES
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

################################################################################
#INCLUDES
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

################################################################################
#SOURCES
file(GLOB_RECURSE FMT_SRC "src/FMT/*.cpp")
file(GLOB_RECURSE CALIBRATION_SRC "src/Calibration/*.cpp")

################################################################################
#LINK
LIST(APPEND LINK_LIBS
   ${OpenCV_LIBS}
   ${YAML_CPP_LIBRARIES}
   ${Boost_SYSTEM_LIBRARY}
   ${Boost_FILESYSTEM_LIBRARY}
)

################################################################################
#EXECUTABLE
add_executable(FMT_demo ${FMT_SRC} src/main.cpp)
target_link_libraries(FMT_demo ${LINK_LIBS})


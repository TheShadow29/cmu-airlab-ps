cmake_minimum_required(VERSION 2.8.3)
project(eigen_calc)

# find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	cmake_modules
)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${Eigen_LIBRARY_DIRS})

add_executable(eigen_calc eigen_calc.cpp)
target_link_libraries(eigen_calc)

cmake_minimum_required(VERSION 2.6)
project(point_set_processing)

find_package( CGAL REQUIRED )

include_directories( ${CGAL_INCLUDE_DIRS})


add_executable(point_set_processing main.cpp)
target_link_libraries(point_set_processing CGAL::CGAL)


install(TARGETS point_set_processing RUNTIME DESTINATION bin)

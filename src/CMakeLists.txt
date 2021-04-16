cmake_minimum_required(VERSION 2.8)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})
#add_subdirectory(kalman_filter)
#add_subdirectory(particle_filter)
set(SOURCES
    dummy.cpp
    )
add_executable(run main.cpp ${SOURCES})
target_link_libraries(run PUBLIC ${ARMADILLO_LIBRARIES})
target_link_libraries(run PUBLIC ${OpenCV_LIBS})
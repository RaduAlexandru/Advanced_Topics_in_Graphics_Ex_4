cmake_minimum_required(VERSION 2.8.12)

project(assignmentsheet4)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")

find_package(LIBIGL REQUIRED)
find_package(nanoflann)
find_package(TouchExpand REQUIRED)

if(MSVC)
  # Enable parallel compilation
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /bigobj")
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4244 /wd4267 /wd4101 /wd4800 /wd4305 /wd4018 /wd4838")
else()
  # Libigl requires a modern C++ compiler that supports c++11
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
endif()

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/externals/libigl/shared/cmake/" "libigl")

#set the libigl options
set(LIBIGL_WITH_OPENGL ON FORCE)
set(LIBIGL_WITH_OPENGL_GLFW ON FORCE)
set(LIBIGL_WITH_VIEWER ON FORCE)
set(LIBIGL_WITH_NANOGUI ON FORCE)

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/Grid3D.cpp
        lib/lib_convolution.h
        externals/TouchExpand/TouchExpand.cpp
        externals/TouchExpand/graph.cpp
        externals/TouchExpand/block.h
        externals/TouchExpand/maxflow.cpp
        lib/marching_cubes_table.h
        lib/lib_marching_cubes.cpp)

target_include_directories( ${PROJECT_NAME}  PUBLIC ${LIBIGL_INCLUDE_DIRS}
                                                    ${nanoflann_INCLUDE_DIRS}
						                            ${TouchExpand_INCLUDE_DIRS}
                                                    "${CMAKE_CURRENT_SOURCE_DIR}/lib"
                                                    "${CMAKE_CURRENT_SOURCE_DIR}/include" )
target_compile_definitions( ${PROJECT_NAME}  PUBLIC ${LIBIGL_DEFINITIONS})
target_link_libraries(      ${PROJECT_NAME}  PUBLIC ${LIBIGL_LIBRARIES}
                                                    ${LIBIGL_VIEWER_EXTRA_LIBRARIES}
                                                    ${LIBIGL_OPENGL_EXTRA_LIBRARIES}
                                                    ${LIBIGL_OPENGL_GLFW_EXTRA_LIBRARIES})



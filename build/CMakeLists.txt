cmake_minimum_required(VERSION 2.6)
project(senior)

add_definitions(-std=c++11)
#There are lots of scripts with cmake
#for finding external libraries. 
#see /usr/local/share/cmake-2.6/Modules/Find*.cmake for more examples
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
include_directories( ../public ${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIRS} "$ENV{EIGEN3_INCLUDE_DIR}" )

file(GLOB_RECURSE seniorproject_SOURCES "../source/*.cpp" "../source/*.cc")

set(CMAKE_CXX_FLAGS "-g -Wall -DGL_GLEXT_PROTOTYPES -lglut -lGL -lGLU")
add_executable(senior ${seniorproject_SOURCES} )

target_link_libraries( senior ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

cmake_minimum_required(VERSION 3.5)
project(Computer_Graphics_group_17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    src/BoundingBox.cpp
    src/BoundingBox.h
    src/imageWriter.h
    src/main.cpp
    src/matrix.h
    src/mesh.cpp
    src/mesh.h
    src/raytracing.cpp
    src/raytracing.h
    src/traqueboule.h
    src/Vec3D.h
    src/Vertex.h src/hsv.h)

add_executable(Computer_Graphics_group_17 ${SOURCE_FILES})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

include_directories( ${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} )

target_link_libraries(Computer_Graphics_group_17 ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )
cmake_minimum_required(VERSION 3.0)
project(Alice)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

INCLUDE_DIRECTORIES(
        ${OpenCV_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${GLUT_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
)

FILE(GLOB_RECURSE ALICE_SRCS "src/*")
add_executable(Alice ${ALICE_SRCS})

ADD_DEFINITIONS(-Wno-deprecated-declarations)

target_link_libraries(Alice
        ${OpenCV_LIBS}
        ${OPENGL_LIBRARIES}
        ${GLUT_LIBRARY}
)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)

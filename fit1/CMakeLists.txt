cmake_minimum_required(VERSION 3.14)

project(a_fit1 VERSION 1.0)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenMesh REQUIRED)




add_executable(a_fit1 src/main.cpp
src/visualization/src/canvas.cpp
src/core/src/discretefairer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
${CMAKE_CURRENT_SOURCE_DIR}/src/visualization/include/
${CMAKE_CURRENT_SOURCE_DIR}/src/core/include/
${CMAKE_CURRENT_SOURCE_DIR}/src/common/include/)

target_link_libraries(a_fit1 PRIVATE OpenGL::GL glfw ${OPENMESH_LIBRARIES})

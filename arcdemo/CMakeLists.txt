cmake_minimum_required(VERSION 3.12)

project(arcdemo)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)




add_executable(arcdemo main.cpp)

target_link_libraries(arcdemo PRIVATE OpenGL::GL glfw)

cmake_minimum_required(VERSION 3.12)

project(a_fit1)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)




add_executable(a_fit1 main.cpp)

target_link_libraries(a_fit1 PRIVATE OpenGL::GL glfw)

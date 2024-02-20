#include <GLFW/glfw3.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>




std::vector<Eigen::Vector3d> translatePoints(std::vector<Eigen::Vector3d>& cps)
{
    const auto t0_origo = cps[0];
    for (auto& cp : cps)
    {
        cp -= t0_origo;
    }
}





int main()
{

    if (!glfwInit())
    {
        return -1;
    }

    GLFWwindow *window = glfwCreateWindow(1500, 1000, "fit1 app", nullptr, nullptr);

    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    std::vector<Eigen::Vector3d> cps; //first element is the center point
    translatePoints(cps);



    while (!glfwWindowShouldClose(window))
    {

        glClear(GL_COLOR_BUFFER_BIT);

        
        



        glfwSwapBuffers(window);

        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

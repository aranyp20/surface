#include <GLFW/glfw3.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>


struct SurfaceParam
{
    double u = 0;
    double v = 0;
};


void translatePoints(std::vector<Eigen::Vector3d>& cps)
{
    const auto t0_origo = cps[0];
    for (auto& cp : cps)
    {
        cp -= t0_origo;
    }
}

std::vector<SurfaceParam> calcUVs(const std::vector<Eigen::Vector3d>& cps)
{
    std::vector<SurfaceParam> retval;


    return retval;
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

    std::vector<Eigen::Vector3d> cps //first element is the center point
    {
        {3,0,0},
        {4,2,1},
        {5,1,-1},
        {3.5,-2,-2},
        {6,-1, -4}
    };

    translatePoints(cps);

    const auto surface_params = calcUVs(cps);


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

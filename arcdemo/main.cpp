#include <GLFW/glfw3.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>

std::vector<Eigen::Vector2d> P{{-0.5, 0.0}, {-0.3, 0.8}, {0.1, -0.2}, {0.5, 0.0}};


int fact(int n)
{
    int res = 1;
    for (int i=1; i <= n; i++) {
        res *= i;
    }
    return res;
}


Eigen::Vector2d bezier(const double t)
{
    Eigen::Vector2d res(0,0);
    const auto& n = P.size() - 1;

    for (unsigned k = 0; k < P.size(); k++) {
        res += P[k] * (fact(n) / (fact(k) * fact(n-k))) * (std::pow(1-t, n-k)) * std::pow(t, k);
    }


    return res;
}

void drawBezier()
{
    const size_t tessellation_level = 100;

    glBegin(GL_LINE_STRIP);
    glColor3f(1.0f, 0.0f, 1.0f);


    for (size_t i = 0; i < tessellation_level; i++) {
        const auto ip = bezier((double)i / (tessellation_level-1));
        glVertex2f(ip[0], ip[1]);
    }

    glEnd();


    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0f, 1.0f, 1.0f);
    for (const auto& p : P) {
        glVertex2f(p[0], p[1]);
    }
    glEnd();
}


int main()
{

    if (!glfwInit())
    {
        return -1;
    }

    GLFWwindow *window = glfwCreateWindow(1500, 1000, "arcdemo app", nullptr, nullptr);

    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    while (!glfwWindowShouldClose(window))
    {

        glClear(GL_COLOR_BUFFER_BIT);

        drawBezier();

        glfwSwapBuffers(window);

        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

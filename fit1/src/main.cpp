#include <GLFW/glfw3.h>

#include "canvas.h"
#include "discretefairer.h"

int main()
{
    Canvas canvas;

    core::DiscreteFairer::MyMesh mesh;

    if (!OpenMesh::IO::read_mesh(mesh, "input1.obj"))
    {
        std::cout << "Error: Cannot read mesh from file." << std::endl;
    }

    core::DiscreteFairer df;
    df.execute(mesh);


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

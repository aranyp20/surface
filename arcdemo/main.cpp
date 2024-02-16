#include <GLFW/glfw3.h>

int main() {

    if (!glfwInit()) {
        return -1;
    }

    GLFWwindow* window = glfwCreateWindow(640, 480, "arcdemo app", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    while (!glfwWindowShouldClose(window)) {


        glClear(GL_COLOR_BUFFER_BIT);

        glBegin(GL_LINES);

        glColor3f(1.0f, 0.0f, 0.0f);  
        glVertex2f(-0.8f, -0.5f);
        glVertex2f(0.8f, -0.5f);

        
        glColor3f(0.0f, 1.0f, 0.0f);  
        glVertex2f(-0.8f, 0.0f);
        glVertex2f(0.8f, 0.0f);

        
        glColor3f(0.0f, 0.0f, 1.0f);  
        glVertex2f(-0.8f, 0.5f);
        glVertex2f(0.8f, 0.5f);

        glEnd();

        glPointSize(10.0f);

        glBegin(GL_POINTS);
        glColor3f(1.0f, 1.0f, 1.0f);
        glVertex2f(0.0f, 0.0f);
        glEnd();

        glfwSwapBuffers(window);

        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();


    return 0;
}

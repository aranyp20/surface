#include <QApplication>
#include <QStyleFactory>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <GLFW/glfw3.h>

#include "canvas.h"
#include "discretefairer.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
 
 
        // Set the Fusion style as the default
    QApplication::setStyle(QStyleFactory::create("Fusion"));

    // Set a custom color palette
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);

    a.setPalette(darkPalette);



    MainWindow w;

    w.show();

    return a.exec();

    gui::Canvas canvas;

    common::MyMesh mesh;


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

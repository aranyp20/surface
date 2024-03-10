#include "canvas.h"





Canvas::Canvas(QWidget *parent) : QOpenGLWidget(parent)
{}

Canvas::~Canvas()
{}

void Canvas::initializeGL()
{
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
}

void Canvas::paintGL()
{

}


void Canvas::resizeGL(int w, int h)
{
}

void Canvas::mousePressEvent(QMouseEvent *event)
{

}

void Canvas::mouseMoveEvent(QMouseEvent *event)
{

}



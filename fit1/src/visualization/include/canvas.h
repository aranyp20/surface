#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMouseEvent>


#include <iostream>

#include "common_defines.h"





class Canvas : public QOpenGLWidget
{
  Q_OBJECT

  QOpenGLBuffer vbo{QOpenGLBuffer::VertexBuffer};

  QOpenGLVertexArrayObject vao;
  QOpenGLShaderProgram *sp;


public:

  Canvas(QWidget *parent);

  virtual ~Canvas();


protected:

  void initializeGL()override;

  void resizeGL(int w, int h)override;

  void paintGL()override;

  void mousePressEvent(QMouseEvent *event)override;

  void mouseMoveEvent(QMouseEvent *event)override;


};





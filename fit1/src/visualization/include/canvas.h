#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMouseEvent>


#include <iostream>

#include "common_defines.h"
#include "camera.h"
#include "objectloader.h"





class Canvas : public QOpenGLWidget
{
  Q_OBJECT

  QOpenGLBuffer vbo{QOpenGLBuffer::VertexBuffer};

  QOpenGLVertexArrayObject vao;
  QOpenGLShaderProgram *sp;


  Camera camera;

  struct qGlVertex
  {
    QVector3D position;
    QVector3D color;
  };

public:

  Canvas(QWidget *parent);

  virtual ~Canvas();

  void setPrintable(const common::MyMesh* const _printable_mesh);

  void changeYaw(double diff);
  void changePitch(double diff);
  void changeRoll(double diff);

protected:

  void initializeGL()override;

  void resizeGL(int w, int h)override;

  void paintGL()override;

  void mousePressEvent(QMouseEvent *event)override;

  void mouseMoveEvent(QMouseEvent *event)override;

private:

  framework::ObjectLoader object_loader; //TODO replace

  std::shared_ptr<const common::MyMesh> printable_mesh = nullptr;
  double model_yaw = 0;
  double model_pitch = 4;
  double model_roll = 1;

  Eigen::Matrix4d modelRotMatrix() const;

  std::vector<qGlVertex> printableMeshToTriangles() const;

};





#include "canvas.h"
#include "discretefairer.h"
#include "organize.h"




Canvas::Canvas(QWidget *parent) : QOpenGLWidget(parent)
{
    
  printable_mesh = object_loader.loadFromWavefrontObj("input1.obj");


}

Canvas::~Canvas()
{}

//TODO Fix it, its not yaw-pitch-roll
Eigen::Matrix4d Canvas::modelRotMatrix() const
{

  Eigen::Matrix4d r_z;
  r_z << std::cos(model_yaw), -std::sin(model_yaw), 0, 0,
          std::sin(model_yaw), std::cos(model_yaw), 0, 0,
          0,0,1,0,
          0,0,0,1;

  Eigen::Matrix4d r_x;
  r_x << 1,0,0,0,
        0, std::cos(model_roll), -std::sin(model_roll), 0,
        0, std::sin(model_roll), std::cos(model_roll), 0,
        0,0,0,1;

  Eigen::Matrix4d r_y;
  r_y << std::cos(model_pitch), 0, std::sin(model_pitch), 0,
          0,1,0,0,
          -std::sin(model_pitch),0, std::cos(model_pitch),0,
          0,0,0,1;

  return r_x * r_y * r_z;
}

void Canvas::changeYaw(double diff)
{
  model_yaw += diff;
}

void Canvas::changePitch(double diff)
{
  model_pitch += diff;
}

void Canvas::changeRoll(double diff)
{
  model_roll += diff;
}

void Canvas::initializeGL()
{
    sp = new QOpenGLShaderProgram();
    sp->addShaderFromSourceCode(QOpenGLShader::Vertex,
                                "#version 450\n"

                                "uniform mat4  M, V, P;\n"

                                "in vec3 position;\n"
                                "in vec3 color;\n"
                                "out vec4 fragColor;\n"

                                "vec4 pos = vec4(position,1);\n"

                                "void main(){\n"
                                "fragColor = vec4(color,1.0);\n"
                                "gl_Position = P * V * M * pos;\n"
                                "}");
    sp->addShaderFromSourceCode(QOpenGLShader::Fragment,
                                "#version 450\n"
                                "in vec4 fragColor;\n"
                                "out vec4 finalColor;\n"
                                "void main(){\n"
                                "finalColor = fragColor;\n"
                                "}");
    sp->link();

    vao.create();
    vao.bind();
    vbo.create();
    vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    vao.release();
    vbo.release();

    glEnable(GL_DEPTH_TEST);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}


//TODO depr.
void Canvas::setPrintable(const common::MyMesh* const _printable_mesh)
{
  //printable_mesh = _printable_mesh;
}

//TODO optimize it to use TRIANGLE_STRIP
std::vector<Canvas::qGlVertex> Canvas::printableMeshToTriangles() const
{
  std::vector<Canvas::qGlVertex> retval;

  if (!printable_mesh) {
    std::cout<< "Canvas::printableMeshToTriangles called while printable_mesh was nullptr."<<std::endl;
    return retval;
  }


  common::MyMesh::FaceIter face_it, face_end(printable_mesh->faces_end());

  for (face_it = printable_mesh->faces_begin(); face_it != face_end; ++face_it)
  {
      common::MyMesh::FaceHandle fh = *face_it;

      common::MyMesh::HalfedgeHandle heh = printable_mesh->halfedge_handle(fh);
      
      for (common::MyMesh::ConstFaceHalfedgeIter fhe_it = printable_mesh->cfh_iter(fh); fhe_it.is_valid(); ++fhe_it)
      {
          common::MyMesh::VertexHandle vh = printable_mesh->to_vertex_handle(*fhe_it);

          common::MyMesh::Point vertex_position = printable_mesh->point(vh);

          OpenMesh::VPropHandleT<double> myprop;
          if(!printable_mesh->get_property_handle(myprop, "doubleValues")){
            std::cout<<"Prop not found."<<std::endl;
          }
          
          const auto curvature = printable_mesh->property(myprop, vh);
          const auto rgb_curvature = common::color::hsvToRgb({curvature/600 + 0.5, 0.8, 0.8});

          retval.push_back({{vertex_position[0] * 10, vertex_position[1] * 10 - 0.8, vertex_position[2] * 10}, {rgb_curvature[0], rgb_curvature[1], rgb_curvature[2]}});
      }
  }

  return retval;
}


void Canvas::paintGL()
{
  if (!printable_mesh) {
    return;
  }

    const auto& v = camera.V();
    const auto& p = camera.P();
    QMatrix4x4 q_v;
    QMatrix4x4 q_p;


    const auto m = modelRotMatrix();
    QMatrix4x4 q_m;


    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        q_v(i, j) = static_cast<float>(v(i, j));
      }
    }

    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        q_p(i, j) = static_cast<float>(p(i, j));
      }
    }

    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        q_m(i, j) = static_cast<float>(m(i, j));
      }
    }

    sp->bind();
    sp->setUniformValue("V", q_v);
    sp->setUniformValue("P", q_p);
    sp->setUniformValue("M", q_m);

    vao.bind();
    vbo.bind();

    std::vector<qGlVertex> pp = printableMeshToTriangles();
    const void *printable_data = pp.data();

    vbo.allocate(printable_data, sizeof(qGlVertex) * pp.size());

    sp->enableAttributeArray("position");
    sp->enableAttributeArray("color");

    sp->setAttributeBuffer(0, GL_FLOAT, offsetof(qGlVertex, position), 3, sizeof(qGlVertex));
    sp->setAttributeBuffer(1, GL_FLOAT, offsetof(qGlVertex, color), 3, sizeof(qGlVertex));

    glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glDrawArrays(GL_TRIANGLES, 0, pp.size());
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



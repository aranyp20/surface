#include "canvas.h"





Canvas::Canvas(QWidget *parent) : QOpenGLWidget(parent)
{
    const auto input_mesh = new common::MyMesh; //TODO temp solution


    if (!OpenMesh::IO::read_mesh(*input_mesh, "input1.obj"))
    {
        std::cout << "Error: Cannot read mesh from file." << std::endl;
    }
    else {
      printable_mesh = input_mesh;
    }

}

Canvas::~Canvas()
{}

void Canvas::initializeGL()
{
    sp = new QOpenGLShaderProgram();
    sp->addShaderFromSourceCode(QOpenGLShader::Vertex,
                                "#version 450\n"

                                "uniform mat4  V, P;\n"

                                "in vec3 position;\n"
                                "in vec3 color;\n"
                                "out vec4 fragColor;\n"

                                "vec4 pos = vec4(position,1);\n"

                                "void main(){\n"
                                "fragColor = vec4(color,1.0);\n"
                                "gl_Position = P * V * pos;\n"
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



void Canvas::setPrintable(const common::MyMesh* const _printable_mesh)
{
  printable_mesh = _printable_mesh;
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

          retval.push_back({{vertex_position[0] * 10, vertex_position[1] * 10 - 0.8, vertex_position[2] * 10}, {1.0, 0.0, 0.0}});
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


    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        q_v(i, j) = static_cast<float>(v(i, j));
      }
    }

    sp->bind();
    sp->setUniformValue("V", q_v);
    sp->setUniformValue("P", q_p);

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



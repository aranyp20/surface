#include "objectloader.h"
#include <iostream>

#include "discretefairer.h"

namespace framework {


void ObjectLoader::modelToShow(common::MyMesh& mesh) const
{
  if (mesh.vertices_empty()) {
    throw std::runtime_error("Cannot preprocess empty mesh.");
  }

  double max_x = std::numeric_limits<double>::min();
  double max_y = std::numeric_limits<double>::min();
  double max_z = std::numeric_limits<double>::min();
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();



  for (common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
  {
    const auto& vh = *v_it;
    const auto& vertexPos = mesh.point(vh);

    max_x = std::max(max_x, static_cast<double>(vertexPos[0]));
    max_y = std::max(max_y, static_cast<double>(vertexPos[1]));
    max_z = std::max(max_z, static_cast<double>(vertexPos[2]));

    min_x = std::min(min_x, static_cast<double>(vertexPos[0]));
    min_y = std::min(min_y, static_cast<double>(vertexPos[1]));
    min_z = std::min(min_z, static_cast<double>(vertexPos[2]));
  }

  Eigen::Matrix4d translate_to_origin = Eigen::Matrix4d::Identity();
  
  const Eigen::Vector3d original_center(common::average(max_x, min_x), common::average(max_y, min_y), common::average(max_z, min_z));
  translate_to_origin.block<3,1>(0,3) = -original_center;

  double normalize_scaler = 2 / std::max(max_x - min_x, std::max(max_y - min_y, max_z - min_z));

  Eigen::Matrix4d scale_to_normalized = Eigen::Matrix4d::Identity();

  scale_to_normalized(0,0) = normalize_scaler;
  scale_to_normalized(1,1) = normalize_scaler;  
  scale_to_normalized(2,2) = normalize_scaler;  
 

  const Eigen::Matrix4d T_show_model = scale_to_normalized * translate_to_origin;


  for (common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
  {
    auto& vh = *v_it;
    auto& vertexPos = mesh.point(vh);

    vertexPos = common::toOpenMeshV3(common::transformAsPoint(T_show_model, common::toEigenV3(vertexPos)));
  }

}


void ObjectLoader::preprocessMesh(common::MyMesh& mesh) const
{
  modelToShow(mesh);

  
  core::DiscreteFairer df;
  df.execute(mesh, 1, 1);

}

void aaaaa(common::MyMesh& mesh)
{

  common::MyMesh::VertexHandle v0 = mesh.add_vertex(common::MyMesh::Point(0, 0, 0)); // A
  common::MyMesh::VertexHandle v1 = mesh.add_vertex(common::MyMesh::Point(1, 0, 0)); // B
  common::MyMesh::VertexHandle v2 = mesh.add_vertex(common::MyMesh::Point(0.5, 1, 0)); // C

  common::MyMesh::FaceHandle f0 = mesh.add_face(v0, v1, v2);

return;
  std::vector<common::MyMesh::FaceHandle> original_faces = {f0};

  common::MyMesh::EdgeHandle e1 = mesh.edge_handle(mesh.find_halfedge(v1, v2)); // Edge AB
  auto v3 = mesh.split(e1, mesh.calc_edge_midpoint(e1));

  mesh.delete_face(f0);

  for (common::MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
  {
      common::MyMesh::FaceHandle fh = *f_it;

      std::cout<<"alma "<<(fh == original_faces[0])<<std::endl;
  }


/*
  // Add the original triangle face

  // Split each edge of the original triangle

  common::MyMesh::EdgeHandle e1 = mesh.edge_handle(mesh.find_halfedge(v1, v2)); // Edge BC
  common::MyMesh::Point midpoint1 = (mesh.point(mesh.to_vertex_handle(mesh.find_halfedge(v1, v2))) + mesh.point(mesh.from_vertex_handle(mesh.find_halfedge(v1, v2)))) / 2.0;
  common::MyMesh::VertexHandle v4 = mesh.split(e1, midpoint1); // E

  common::MyMesh::EdgeHandle e2 = mesh.edge_handle(mesh.find_halfedge(v2, v0)); // Edge CA
  common::MyMesh::Point midpoint2 = (mesh.point(mesh.to_vertex_handle(mesh.find_halfedge(v2, v0))) + mesh.point(mesh.from_vertex_handle(mesh.find_halfedge(v2, v0)))) / 2.0;

  common::MyMesh::VertexHandle v5 = mesh.split(e2, midpoint2); // F

  // Create new faces
  mesh.add_face(v3, v4, v5); // ADB
  mesh.add_face(v0, v3, v5); // BEC
  mesh.add_face(v1, v3, v4); // CFA
  mesh.add_face(v2, v4, v5); // DEF
*/




//  mesh.delete_face(*(mesh.faces_begin()), true);

  // Remove the original triangle face
  //mesh.delete_face(f0, false);


  /*
        common::MyMesh::VertexHandle v0 = mesh.add_vertex(common::MyMesh::Point(0, 0, 0)); // A
      common::MyMesh::VertexHandle v1 = mesh.add_vertex(common::MyMesh::Point(1, 0, 0)); // B
      common::MyMesh::VertexHandle v2 = mesh.add_vertex(common::MyMesh::Point(0.5, 1, 0)); // C

      auto fo = mesh.add_face(v0, v1, v2);


    for (common::MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
    {
        common::MyMesh::FaceHandle fh = *f_it;
        

        std::vector<common::MyMesh::EdgeHandle> ehs;
        // Iterate through edges of the face
        for (common::MyMesh::FaceEdgeIter fe_it = mesh.fe_iter(fh); fe_it.is_valid(); ++fe_it)
        {
            common::MyMesh::EdgeHandle eh = *fe_it;   
        }

        for (auto& eh : ehs) {

            common::MyMesh::HalfedgeHandle heh = mesh.halfedge_handle(eh, 0); // get the halfedge handle
            
            // Get the midpoint of the edge
            common::MyMesh::Point midpoint = (mesh.point(mesh.to_vertex_handle(heh)) + mesh.point(mesh.from_vertex_handle(heh))) / 2.0;

            // Split the edge at the midpoint
            common::MyMesh::VertexHandle new_vertex = mesh.split(eh, midpoint);
            
        }

    }
        mesh.delete_face(fo, true);
        mesh.garbage_collection();
  */

}


std::shared_ptr<common::MyMesh> ObjectLoader::loadFromFile(const std::string& path) const
{
  common::MyMesh input_mesh;

  if (!OpenMesh::IO::read_mesh(input_mesh, path))
  {
      std::cout << "Error: Cannot read mesh from file." << std::endl;
      return nullptr;
  }
  else {
    //preprocessMesh(input_mesh);

    common::MyMesh mesh;

    aaaaa(mesh);

    preprocessMesh(mesh);

    return std::make_shared<common::MyMesh>(std::move(mesh));
  }

  return nullptr;
}


}
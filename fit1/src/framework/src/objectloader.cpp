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
  df.execute(mesh);

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
    preprocessMesh(input_mesh);
    return std::make_shared<common::MyMesh>(std::move(input_mesh));
  }

  return nullptr;
}


}
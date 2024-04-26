#include "discretefairer.h"
#include <OpenMesh/Tools/Subdivider/Uniform/MidpointT.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include "organize.hpp"
#include <unordered_map>
#include <optional>


namespace core {

namespace {


common::MyMesh::EdgeHandle findEdgeConnectingVertices(common::MyMesh& mesh, common::MyMesh::VertexHandle v1, common::MyMesh::VertexHandle v2) {
    for (common::MyMesh::ConstEdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
        common::MyMesh::HalfedgeHandle heh0 = mesh.halfedge_handle(*e_it, 0);
        common::MyMesh::HalfedgeHandle heh1 = mesh.halfedge_handle(*e_it, 1);
        common::MyMesh::VertexHandle v0 = mesh.to_vertex_handle(heh0);
        common::MyMesh::VertexHandle v3 = mesh.to_vertex_handle(heh1);
        if ((v1 == v0 && v2 == v3) || (v1 == v3 && v2 == v0)) {
            // Found the edge connecting the two vertices
            return *e_it;
        }
    }
    // If no such edge found, return an invalid handle
    return common::MyMesh::EdgeHandle();
}

 

  // Vertexhandle not present as child in this map must be original vertices
  //TODO not global!!
  std::map<common::MyMesh::VertexHandle, std::array<common::MyMesh::VertexHandle,2>> child_parents_map; 
  
  //TODO refactor
void subdivide(common::MyMesh& mesh)
{
    std::vector<common::MyMesh::EdgeHandle> original_edges;
    for (common::MyMesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it)
    {
        original_edges.push_back(*e_it);
    }

    std::vector<common::MyMesh::VertexHandle> new_vertices;
    std::vector<common::MyMesh::EdgeHandle> flippable_edges;

    for(auto& original_edge_h : original_edges)
    {
        auto heh1 = mesh.halfedge_handle(original_edge_h, 0);
        auto heh2 = mesh.halfedge_handle(original_edge_h, 1);


        auto face1 = mesh.face_handle(heh1);
        auto face2 = mesh.face_handle(heh2);


        auto v1 = heh1.is_valid() ? mesh.from_vertex_handle(heh1) : mesh.from_vertex_handle(heh2);
        auto v2 = heh1.is_valid() ? mesh.to_vertex_handle(heh1) : mesh.to_vertex_handle(heh2);

        common::MyMesh::VertexHandle opposite_vertex1;
        common::MyMesh::VertexHandle opposite_vertex2;


        if (face1.is_valid()) {
            for (common::MyMesh::FaceVertexIter fv_it = mesh.fv_iter(face1); fv_it.is_valid(); ++fv_it) {
                common::MyMesh::VertexHandle v_face = *fv_it;

                if (v_face != v1 && v_face != v2) {
                    opposite_vertex1 = v_face;
                    break;
                }
            }
        }

        if (face2.is_valid()) {
            for (common::MyMesh::FaceVertexIter fv_it = mesh.fv_iter(face2); fv_it.is_valid(); ++fv_it) {
                common::MyMesh::VertexHandle v_face = *fv_it;

                if (v_face != v1 && v_face != v2) {
                    opposite_vertex2 = v_face;
                    break;
                }
            }
        }




        auto new_vertex = mesh.split(original_edge_h, mesh.calc_edge_midpoint(original_edge_h));

        new_vertices.push_back(new_vertex);

	child_parents_map.insert({new_vertex, {v1, v2}});
	

        if(face1.is_valid()){
            if(std::find(new_vertices.begin(), new_vertices.end(), opposite_vertex1) == new_vertices.end()){
                flippable_edges.push_back(findEdgeConnectingVertices(mesh, new_vertex, opposite_vertex1));     

            }
        }

        if(face2.is_valid()){
            if(std::find(new_vertices.begin(), new_vertices.end(), opposite_vertex2) == new_vertices.end()){
                flippable_edges.push_back(findEdgeConnectingVertices(mesh, new_vertex, opposite_vertex2));     

            }
        }
    }

    for (auto& flippable_edge : flippable_edges) {


            mesh.flip(flippable_edge);

    }
}
  
  void getEffectorsHelper(const common::MyMesh::VertexHandle& to, std::set<common::MyMesh::VertexHandle>& visited_vertices, std::set<common::MyMesh::VertexHandle>& effectors)
  {
    if(child_parents_map.count(to) == 0) {
      // Original vertex
      effectors.insert(to);
      return;
    }

    visited_vertices.insert(to);
    
    const auto& parents = child_parents_map.at(to);

    for(size_t i = 0; i < 2; i++){
      if(std::find(visited_vertices.begin(), visited_vertices.end(), parents[i]) == visited_vertices.end()){
	getEffectorsHelper(parents[i], visited_vertices, effectors);
      }
    }    
    
  }

  std::set<common::MyMesh::VertexHandle> getEffectors(const common::MyMesh::VertexHandle& to)
  {
    std::set<common::MyMesh::VertexHandle> result;
    std::set<common::MyMesh::VertexHandle> tmp_visited_vertices;

    getEffectorsHelper(to, tmp_visited_vertices, result);

    return result;
  }


  // TODO: optimize
  bool isOriginalVertex(const common::MyMesh::VertexHandle& vh)
  {
    return getEffectors(vh).size() == 1;
  }
  
  
  std::map<common::MyMesh::VertexHandle, double> vertex_curvature_map;
  
  Eigen::Vector3d iterateVertex(common::MyMesh& mesh, common::MyMesh::VertexHandle& iteratable)
{

  auto effectors = getEffectors(iteratable);

  //std::cout<<"Effectors size: "<< effectors.size()<<std::endl;

  std::vector<common::MyMesh::VertexHandle> neighbors;
  for (common::MyMesh::ConstVertexVertexIter vv_it = mesh.cvv_begin(iteratable); vv_it != mesh.cvv_end(iteratable); ++vv_it)
  {
    neighbors.push_back(*vv_it);
  }
  //std::cout<<"Neighbors size: "<<neighbors.size()<<std::endl;
  
  
  std::array<Eigen::Vector3d, 6> e_neighbors;
  for(size_t i = 0; i< 6; i++){
    const auto& p = mesh.point(neighbors[i]);
    e_neighbors[i] =  Eigen::Vector3d(p[0], p[1], p[2]);
  }
  

  CurvatureCalculator cc(mesh);
  cc.execute(iteratable);
  const auto normal = cc.getNormal();
  const auto fe = cc.getFundamentalElements();

  double H;
  if(effectors.size() == 2) {
    std::vector<common::MyMesh::VertexHandle> t_effectors;
    for(const auto& a : effectors) {
      t_effectors.push_back(a);
    }
    const auto p0 = mesh.point(iteratable);
    const auto p1 = mesh.point(t_effectors[0]);
    const auto p2 = mesh.point(t_effectors[1]);
    auto d1 = (p0 - p1).norm();
    auto d2 = (p0 - p2).norm();
    const auto normalizer = d2 + d1;
    d1 = d1 / normalizer;
    d2 = d2 / normalizer;
    H = vertex_curvature_map.at(t_effectors[0]) * d1 + vertex_curvature_map.at(t_effectors[1]) * d2;
  }
  else if (effectors.size() == 3) {
    std::vector<common::MyMesh::VertexHandle> t_effectors;
    for(const auto& a : effectors) {
      t_effectors.push_back(a);
    }
    const auto p0 = mesh.point(iteratable);
    const auto p1 = mesh.point(t_effectors[0]);
    const auto p2 = mesh.point(t_effectors[1]);
    const auto p3 = mesh.point(t_effectors[2]);
    auto d1 = (p0 - p1).norm();
    auto d2 = (p0 - p2).norm();
    auto d3 = (p0 - p3).norm();
    const auto normalizer = d3 + d2 + d1;
    d1 = d1 / normalizer;
    d2 = d2 / normalizer;
    d3 = d3 / normalizer;
    H = vertex_curvature_map.at(t_effectors[0]) * d1 + vertex_curvature_map.at(t_effectors[1]) * d2 + vertex_curvature_map.at(t_effectors[2]) * d3;
  }
  else {
    std::cout<<"Vertex iterate invalid effectors count."<<std::endl;
  }

  return DiscreteFairer::Q(e_neighbors, normal, H, fe);
  
}


} // namespace


void DiscreteFairer::execute(common::MyMesh& mesh, size_t face_split_count, size_t iteration_count)
{
  // Subdivide the base mesh
  // child_parents_map is filled
  for(size_t i = 0; i < face_split_count; i++) {
    //subdivide(mesh);
    
  }

  for(size_t i = 0; i < 0 ; i++){
    // Calculate the curvature for each vertex at the beginning of each iteration
    CurvatureCalculator mcc(mesh);
    //TODO range operator (smarthandle....)
    for(common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it){
      auto vh = *v_it;
      mcc.execute(vh);
      const auto curvature = mcc.getCurvature();
      vertex_curvature_map[vh] =  curvature;
    }
    // Calculate the new position of each new vertex
    std::vector<std::pair<common::MyMesh::VertexHandle, Eigen::Vector3d>> new_vertex_positions;
    for(common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it){
      auto vh = *v_it;
      if (!isOriginalVertex(vh)) {
	new_vertex_positions.emplace_back(vh, iterateVertex(mesh, vh));
      }
    }
    // Replace each vertex to its new position
    for (auto& vertex_with_new_pos : new_vertex_positions) {
      // TODO: conversion in common (new file for all of these)
      const auto& new_pos_e = vertex_with_new_pos.second;
      common::MyMesh::Point new_pos(new_pos_e[0], new_pos_e[1], new_pos_e[2]);
      mesh.point(vertex_with_new_pos.first) = new_pos;
    }

  }

  OpenMesh::IO::write_mesh(mesh, "result.obj");

    

    OpenMesh::VPropHandleT<double> doubleValues;
    mesh.add_property(doubleValues, "doubleValues");

    CurvatureCalculator cc(mesh); //TODO: only calculate it in the original vertices
    for (common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        common::MyMesh::VertexHandle vh = *v_it;

        cc.execute(vh);
	std::cout<<cc.getFundamentalElements()<<std::endl;
	
        mesh.property(doubleValues, vh) = cc.getCurvature();
    }

}

Eigen::Vector3d DiscreteFairer::Q(const std::array<Eigen::Vector3d, 6>& p,const Eigen::Vector3d& normal, double H,  const CurvatureCalculator::FundamentalElements& fe)
{

    const auto p_k = common::average(p[0], p[1], p[2], p[3], p[4], p[5]);

    const auto t = (2 * H * (fe.E * fe.G - fe.F * fe.F) + 2 * fe.M * fe.F - fe.G * normal.dot(p[0] + p[3])
                    - fe.E * (1/3) * normal.dot(-p[0] + 2 * p[1] + 2 * p[2] - p[3] + 2 * p[4] + 2 * p[5])) / (-2 * (fe.E + fe.G))
                    - normal.dot(p_k);

    return p_k + normal * t;
}

}

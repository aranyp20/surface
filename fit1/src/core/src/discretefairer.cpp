#include "discretefairer.h"
#include <OpenMesh/Tools/Subdivider/Uniform/MidpointT.hh>
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

void subdivide(common::MyMesh& mesh)
{
    std::vector<common::MyMesh::EdgeHandle> original_edges;
    for (common::MyMesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it)
    {
        original_edges.push_back(*e_it);
    }

    std::vector<common::MyMesh::FaceHandle> original_faces;
    for (common::MyMesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
    {
        original_faces.push_back(*f_it);
    }


    std::unordered_map<common::MyMesh::FaceHandle, common::MyMesh::EdgeHandle> face_memory;
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
            bool face_is_original = std::find(original_faces.begin(), original_faces.end(), face1) != original_faces.end();

            if(face_is_original) {
                for (common::MyMesh::FaceVertexIter fv_it = mesh.fv_iter(face1); fv_it.is_valid(); ++fv_it) {
                    common::MyMesh::VertexHandle v_face = *fv_it;

                    if (v_face != v1 && v_face != v2) {
                        opposite_vertex1 = v_face;
                        break;
                    }
                }
            }
        }
        //std::cout<<"a1 "<<opposite_vertex1.is_valid()<<std::endl;

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
        
        if (face1.is_valid()) {
            if (face_memory.count(face1) == 0) {
                face_memory.insert({face1, findEdgeConnectingVertices(mesh, opposite_vertex1, new_vertex)});
            }
        }

        if (face2.is_valid()) {
            if (face_memory.count(face2) == 0) {
                face_memory.insert({face2, findEdgeConnectingVertices(mesh, opposite_vertex2, new_vertex)});
            }
        }        
    }

    for (auto& face_and_edge : face_memory) {
        std::cout<<"Flip"<<std::endl;
        //mesh.flip(face_and_edge.second);
    }
}


void iterateVertex(common::MyMesh& mesh, const common::MyMesh::VertexHandle& iteratable)
{

}


}


void DiscreteFairer::execute(common::MyMesh& mesh, size_t face_split_count, size_t iteration_count)
{
    CurvatureCalculator cc(mesh);
    subdivide(mesh);

/*
    OpenMesh::VPropHandleT<double> doubleValues;
    mesh.add_property(doubleValues, "doubleValues");

    for (common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        common::MyMesh::VertexHandle vh = *v_it;

        cc.execute(vh);

        mesh.property(doubleValues, vh) = cc.getCurvature();
    }
*/



    return;

}

Eigen::Vector3d DiscreteFairer::Q(const std::array<Eigen::Vector3d, 6>& p,const Eigen::Vector3d& normal, double H,  const CurvatureCalculator::FundamentalElements& fe) const
{

    const auto p_k = common::average(p[0], p[1], p[2], p[3], p[4], p[5]);

    const auto t = (2 * H * (fe.E * fe.G - fe.F * fe.F) + 2 * fe.M * fe.F - fe.G * normal.dot(p[0] + p[3])
                    - fe.E * (1/3) * normal.dot(-p[0] + 2 * p[1] + 2 * p[2] - p[3] + 2 * p[4] + 2 * p[5])) / (-2 * (fe.E + fe.G))
                    - normal.dot(p_k);

    return p_k + normal * t;
}








}
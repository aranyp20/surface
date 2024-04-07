#include "discretefairer.h"
#include <OpenMesh/Tools/Subdivider/Uniform/MidpointT.hh>
#include "organize.hpp"


namespace core {

namespace {

void subdivide(common::MyMesh& mesh)
{
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


}


}


void DiscreteFairer::execute(common::MyMesh& mesh)
{
    CurvatureCalculator cc(mesh);
    //subdivide(mesh);


    OpenMesh::VPropHandleT<double> doubleValues;
    mesh.add_property(doubleValues, "doubleValues");

    for (common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        common::MyMesh::VertexHandle vh = *v_it;

        cc.execute(vh);

        mesh.property(doubleValues, vh) = cc.getCurvature();
    }



    return; //TODO
/*
    for (common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        common::MyMesh::VertexHandle vh = *v_it;
        double doubleValue = mesh.property(doubleValues, vh);
        //std::cout << "Vertex " << vh.idx() << ": Double Value = " << doubleValue << std::endl;
    }

    size_t original_vert_num = mesh.n_vertices();

    MySubdivider midpoint_subdivider;
    midpoint_subdivider.mySubdivideFunction(mesh);


    OpenMesh::VPropHandleT<double> doubleValues;
    mesh.add_property(doubleValues, "doubleValues");


    size_t processed_verts = 0;
    for (common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it, processed_verts++)
    {
        common::MyMesh::VertexHandle vh = *v_it;

        if(processed_verts < original_vert_num){
            mesh.property(doubleValues, vh) = 1;
        }
        else {
            mesh.property(doubleValues, vh) = 0;
        }
    }

*/

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
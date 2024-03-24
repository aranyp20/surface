#include "discretefairer.h"
#include <OpenMesh/Tools/Subdivider/Uniform/MidpointT.hh>
#include "organize.hpp"


namespace core {


class MySubdivider : public OpenMesh::Subdivider::Uniform::MidpointT<common::MyMesh> {
public:
    bool mySubdivideFunction(common::MyMesh& mesh) {
        return this->subdivide(mesh, 1);
    }
};


void DiscreteFairer::execute(common::MyMesh& mesh)
{
    CurvatureCalculator cc(mesh);


    OpenMesh::VPropHandleT<double> doubleValues;
    mesh.add_property(doubleValues, "doubleValues");

    for (common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        common::MyMesh::VertexHandle vh = *v_it;

        cc.execute(vh);
        

        mesh.property(doubleValues, vh) = cc.getCurvature();
    }

    for (common::MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        common::MyMesh::VertexHandle vh = *v_it;
        double doubleValue = mesh.property(doubleValues, vh);
        //std::cout << "Vertex " << vh.idx() << ": Double Value = " << doubleValue << std::endl;
    }


    return; //TODO
/*
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
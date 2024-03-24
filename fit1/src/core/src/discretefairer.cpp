#include "discretefairer.h"
#include <OpenMesh/Tools/Subdivider/Uniform/MidpointT.hh>
namespace core {


class MySubdivider : public OpenMesh::Subdivider::Uniform::MidpointT<common::MyMesh> {
public:
    bool mySubdivideFunction(common::MyMesh& mesh) {
        return this->subdivide(mesh, 1);
    }
};

void DiscreteFairer::execute(common::MyMesh& mesh)
{


    size_t original_vert_num = mesh.n_vertices();

    MySubdivider midpoint_subdivider;
    midpoint_subdivider.mySubdivideFunction(mesh);

    //cc.execute(mesh);

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


}








}
#include <GLFW/glfw3.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

#include "canvas.h"

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

struct DerResults
{
    Eigen::Vector3d Su{0, 0, 0};
    Eigen::Vector3d Sv{0, 0, 0};
    Eigen::Vector3d Suu{0, 0, 0};
    Eigen::Vector3d Suv{0, 0, 0};
    Eigen::Vector3d Svv{0, 0, 0};
};

struct SurfaceParam
{
    double u = 0;
    double v = 0;
};

// TODO u,v should be here
struct InputPoints
{
    Eigen::Vector3d center;
    std::vector<Eigen::Vector3d> P;

    void translatePoints()
    {
        for (auto &cp : P)
        {
            cp -= center;
        }
        center = {0, 0, 0};
    }
};

typedef std::array<Eigen::Vector3d, 3> Triangle3d;

// TODO before integrated: cps.P size check + origo center precondition
std::vector<SurfaceParam> calcUVs(const InputPoints &cps)
{
    std::vector<SurfaceParam> retval;

    //retval.push_back({0, 0});

    std::vector<double> alphas;
    double alpha_sum = 0;
    for (size_t i = 0; i < cps.P.size(); i++)
    {
        const auto &p_i = cps.P[i];
        const auto &p_i_next = i == cps.P.size() - 1 ? cps.P[0] : cps.P[i + 1];
        const auto alpha = std::acos(p_i.normalized().dot(p_i_next.normalized()));
        alphas.push_back(alpha);
        alpha_sum += alpha;
    }

    const auto alpha_normalizer = 2 * M_PI / alpha_sum;
    for (auto &alpha : alphas)
    {
        alpha *= alpha_normalizer;
    }

    for (size_t i = 0; i < cps.P.size(); i++)
    {
        const auto h_i = cps.P[i].norm();

        double sum_a = 0;
        if (i != 0)
        {
            for (size_t j = 0; j < i; j++)
            {
                sum_a += alphas[j];
            }
        }

        retval.push_back({h_i * std::cos(sum_a), h_i * std::sin(sum_a)});
    }

    return retval;
}

DerResults calcDer(const InputPoints &ipp, const std::vector<SurfaceParam> &uvs)
{
    const auto res_size = 5;
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(uvs.size(), res_size);
    Eigen::MatrixXd c = Eigen::MatrixXd::Random(uvs.size(), 3);

    for (int i = 0; i < uvs.size(); i++)
    {
        const auto &uv = uvs[i];
        A(i, 0) = uv.u;
        A(i, 1) = uv.v;
        A(i, 2) = 0.5 * uv.u * uv.u;
        A(i, 3) = uv.u * uv.v;
        A(i, 4) = 0.5 * uv.v * uv.v;


        c(i, 0) = ipp.P[i][0];
        c(i, 1) = ipp.P[i][1];
        c(i, 2) = ipp.P[i][2];
        
    }

    Eigen::MatrixXd b = Eigen::MatrixXd::Random(res_size, 3);

    if (uvs.size() >= 5)
    {
        //Eigen uses the least squares method for overdetermined systems by default

        b = A.colPivHouseholderQr().solve(c);
    }
    else {

        //Min norm methods

        b = A.transpose() * (A * A.transpose()).inverse() * c;
    }


    return {b.row(0), b.row(1), b.row(2), b.row(3), b.row(4)};
}

Eigen::Vector3d S(const double u, const double v, const DerResults &Ss)
{
    const Eigen::Vector3d S00{0, 0, 0};

    return S00 + u * Ss.Su + v * Ss.Sv + 0.5 * u * u * Ss.Suu + u * v * Ss.Suv + 0.5 * v * v * Ss.Svv;
}

std::vector<Triangle3d> tessellateSurface(const size_t resolution, const DerResults &Ss)
{

#define uvBOUND 20

    const double u_min = -uvBOUND;
    const double u_max = uvBOUND;
    const double v_min = -uvBOUND;
    const double v_max = uvBOUND;

    const double u_step_distance = (u_max - u_min) / resolution;
    const double v_step_distance = (v_max - v_min) / resolution;

    std::vector<Eigen::Vector3d> values;
    for (size_t i = 0; i < resolution; i++)
    {
        const double u = u_min + i * u_step_distance;
        for (size_t j = 0; j < resolution; j++)
        {
            const double v = v_min + j * v_step_distance;
            values.push_back(S(u, v, Ss));
        }
    }

    std::vector<Triangle3d> retval;

    // TODO refactor


    std::fstream fw("res1.obj", std::ios::out);



    fw << "# Vertices\n";


    for (const auto &vertex : values)
    {
        fw << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << std::endl;
    }




    fw << "\n# Faces\n";


    for (size_t i = 0; i < (resolution - 1) * (resolution - 1); i++)
    {
        const size_t left_right_index = std::floor(i / (resolution - 1)) + i;
        retval.push_back({values[left_right_index], values[left_right_index + resolution], values[left_right_index + resolution + 1]});
        retval.push_back({values[left_right_index], values[left_right_index + resolution + 1], values[left_right_index]}); // TODO check indices

        fw << "f " << left_right_index + 1 << " " << left_right_index + resolution + 1 << " " << left_right_index + resolution + 2 << std::endl;
        fw << "f " << left_right_index + 1 << " " << left_right_index + resolution + 2 << " " << left_right_index + 2 << std::endl;
    }


    return retval;
}


double calcCurvature(const InputPoints& ipp)
{
    auto ip = ipp;

    ip.translatePoints();

    const auto surface_params = calcUVs(ip);

    const auto eq = calcDer(ip, surface_params);


    const auto E = eq.Su.norm() * eq.Su.norm();
    const auto G = eq.Sv.norm() * eq.Sv.norm();
    const auto F = eq.Su.dot(eq.Sv);

    const auto n = eq.Su.cross(eq.Sv).normalized();
    const auto L = eq.Suu.dot(n);
    const auto M = eq.Suv.dot(n);
    const auto N = eq.Svv.dot(n);

    return (L * G - M * F + N * E - M * F) / (2 * E * G - F * F);

}

void calcCurvatures()
{


    typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

    MyMesh mesh;

    
    if (!OpenMesh::IO::read_mesh(mesh, "input1.obj")) {
        std::cout << "Error: Cannot read mesh from file." << std::endl;
    }

    
    OpenMesh::VPropHandleT<double> doubleValues;
    mesh.add_property(doubleValues);


    for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
        MyMesh::VertexHandle vh = *v_it;

        MyMesh::Point vertexPos = mesh.point(vh); //TODO ref?

        
        InputPoints cip;
        cip.center = Eigen::Vector3d(vertexPos[0], vertexPos[1], vertexPos[2]);
        for (MyMesh::VertexOHalfedgeIter voh_it = mesh.voh_iter(vh); voh_it.is_valid(); ++voh_it) {
            MyMesh::VertexHandle neighborVh = mesh.to_vertex_handle(*voh_it);

            const auto neighborPos = mesh.point(neighborVh); //TODO ref?
            cip.P.emplace_back(neighborPos[0], neighborPos[1], neighborPos[2]);
        }


        mesh.property(doubleValues, vh) = calcCurvature(cip);
    }

    // Now, you can access the double values for each vertex using the property
    for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
        MyMesh::VertexHandle vh = *v_it;
        double doubleValue = mesh.property(doubleValues, vh);
        // Do something with the calculated double value
        std::cout << "Vertex " << vh.idx() << ": Double Value = " << doubleValue << std::endl;
    }
}



int main()
{
    Canvas canvas;

    if (!glfwInit())
    {
        return -1;
    }

    GLFWwindow *window = glfwCreateWindow(1500, 1000, "fit1 app", nullptr, nullptr);

    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    InputPoints ip;

    ip.center = {4, 0, 0};
    
    /*
    ip.P =
        {
            {5, 4, 2},
            {2, 4, -3},
            {5, -1, -4},
            {1, -4, 10},
            {3,3,3},
            {-4,1,-2}};
    */

   ip.P = {
    //{4,5,6},
    //{4,9,-1},
    {4,1,1},
    {4,3,-2},
    {10, 2,5},
    {4,-2,-1},
    //{4,6,-1}
   };

    calcCurvatures();

/*
    std::cout << eq_params.Su << "\n"
              << eq_params.Sv << "\n"
              << eq_params.Suu << "\n"
              << eq_params.Suv << "\n"
              << eq_params.Svv << std::endl;
*/

    // for(auto a : surface_params){
    //     std::cout<<a.u<<" "<<a.v<<std::endl;
    // }

    while (!glfwWindowShouldClose(window))
    {

        glClear(GL_COLOR_BUFFER_BIT);

        glfwSwapBuffers(window);

        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

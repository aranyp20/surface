#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

#include "common_defines.h"

namespace core
{
    class CurvatureCalculator
    {

        struct SurfaceParam
        {
            double u = 0;
            double v = 0;
        };

        struct InputPoints
        {
            Eigen::Vector3d center{0, 0, 0};
            std::vector<Eigen::Vector3d> P;

            InputPoints translatePoints() const;

            std::vector<SurfaceParam> calcUVs() const;
        };

        struct DerResults
        {
            Eigen::Vector3d Su{0, 0, 0};
            Eigen::Vector3d Sv{0, 0, 0};
            Eigen::Vector3d Suu{0, 0, 0};
            Eigen::Vector3d Suv{0, 0, 0};
            Eigen::Vector3d Svv{0, 0, 0};
        };

        DerResults calcDer(const InputPoints &ipp) const;
        Eigen::Vector3d S(const double u, const double v, const DerResults &Ss) const;
        double calcCurvature(const InputPoints &ipp) const;
        void calcCurvatures(common::MyMesh &mesh) const;

    public:

        void execute(common::MyMesh &mesh);
    };

}

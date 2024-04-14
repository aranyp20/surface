#pragma once

#include "curvaturecalculator.h"


namespace core {


class DiscreteFairer
{

Eigen::Vector3d Q(const std::array<Eigen::Vector3d, 6>& p /* p[0] and p[3] (aka p1 and p4) are the original vertices*/,
                    const Eigen::Vector3d& normal, double H, const CurvatureCalculator::FundamentalElements& fe) const;

public:

void execute(common::MyMesh& mesh, size_t face_split_count, size_t iteration_count);



};

}
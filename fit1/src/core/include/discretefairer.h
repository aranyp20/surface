#pragma once

#include "curvaturecalculator.h"


namespace core {


class DiscreteFairer
{

CurvatureCalculator cc;

public:

void execute(common::MyMesh& mesh);

};

}
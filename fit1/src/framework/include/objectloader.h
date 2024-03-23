#pragma once

#include <OpenMesh/Core/IO/MeshIO.hh>
#include "common_defines.h"
#include <memory>
#include "organize.hpp"


namespace framework {

class ObjectLoader  
{



    void modelToShow(common::MyMesh& mesh) const;
    void preprocessMesh(common::MyMesh& mesh) const;

public:


    std::shared_ptr<common::MyMesh> loadFromFile(const std::string& path) const;


};
}
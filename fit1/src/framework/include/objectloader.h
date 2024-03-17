#pragma once

#include <OpenMesh/Core/IO/MeshIO.hh>
#include "common_defines.h"
#include <memory>


namespace framework {

class ObjectLoader
{

public:


    std::shared_ptr<common::MyMesh> loadFromWavefrontObj(const std::string& path) const;


};
}
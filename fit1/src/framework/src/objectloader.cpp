#include "objectloader.h"
#include <iostream>

#include "discretefairer.h"


namespace framework {


std::shared_ptr<common::MyMesh> ObjectLoader::loadFromWavefrontObj(const std::string& path) const
{
    common::MyMesh input_mesh;

    if (!OpenMesh::IO::read_mesh(input_mesh, "input1.obj"))
    {
        std::cout << "Error: Cannot read mesh from file." << std::endl;
        return nullptr;
    }
    else {
      core::DiscreteFairer df;
      df.execute(input_mesh);
      return std::make_shared<common::MyMesh>(input_mesh);
    }

    return nullptr;
}



}
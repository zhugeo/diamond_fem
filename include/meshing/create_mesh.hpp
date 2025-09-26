#ifndef _DIAMOND_FEM_MESHING_CREATE_MESH_HPPP
#define _DIAMOND_FEM_MESHING_CREATE_MESH_HPPP

#include <vector>

#include <meshing/mesh.hpp>

namespace diamond_fem::meshing {

Mesh CreateMesh(std::vector<internal::BorderRef> borders);

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_CREATE_MESH_HPPP

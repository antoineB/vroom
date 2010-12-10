#ifndef MY_TOOLS_HPP
#define MY_TOOLS_HPP

#include <ode/ode.h>
#include "OgreFramework.hpp"
#include "global.hpp"

namespace MyTools{
  using namespace Ogre;

  Vector3* getVerticesArray(const VertexData *vertex_data);
  unsigned int getVerticesNumber(const VertexData *vertex_data);
  unsigned int getIndicesNumber(IndexData *data);
  dTriIndex* getIndicesArray(IndexData *data, const unsigned int offset);

  void byOdeToOgre(dBodyID b, SceneNode* n);
  void byOdeToOgre(dGeomID g, SceneNode* n);
  dTriMeshDataID dTriMeshDataFromMesh(Ogre::Entity* e);
};

#endif

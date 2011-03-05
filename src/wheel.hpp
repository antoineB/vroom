#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <ode/ode.h>

#include "parsexml.hpp"
#include "type.hpp"
#include "conf.hpp"

class Wheel{
  
public:
  Wheel();  
  ~Wheel();

  void create(dSpaceID space, Ogre::SceneNode *node);
  void update();

  static DContactType type;

  struct Ph {
    dBodyID body;
    dGeomID geom;
    dMass mass;
  };

  struct Ph ph;

private:

  void fillContact();
  void createPhysics(dSpaceID s, unsigned int nbWheel, bool sphere = false);
  std::string generateName(unsigned int number);
  void createNodesAndMesh(Ogre::SceneNode *carNode);

  struct Cst {
    std::string nodeName;
    Ogre::SceneNode *wheelNode;
  };

  struct Cst cst;
};



#endif

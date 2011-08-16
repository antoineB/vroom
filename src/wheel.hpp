#ifndef WHEEL_HPP
#define WHEEL_HPP

#include "utils.hpp"
#include "type.hpp"
#include "conf.hpp"

#include <OGRE/OgreSceneNode.h>

#include <ode/ode.h>


class Wheel{
public:
  struct Cst {
    std::string xmlFile;
    std::string nodeName;
    Ogre::SceneNode *wheelNode;
  };

  struct Ph {
    dBodyID body;
    dGeomID geom;
    dMass mass;
  };

  Wheel();  
  ~Wheel();

  void initXml(const char* xmlFile, dSpaceID space);
  void update();

  void reset();

  DContactType type;

  struct Ph ph;

private:

  void createNodesAndMesh(Utils::Xml &x);
  void createPhysics(Utils::Xml &x, dSpaceID s);
  void disposePhysics(Utils::Xml &x);

  void setupContact(Utils::Xml &x);

  struct Cst cst;
};



#endif

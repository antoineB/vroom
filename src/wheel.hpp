#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <OGRE/OgreSceneNode.h>

#include <ode/ode.h>

#include "type.hpp"
#include "conf.hpp"

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

  void createXml(const char* xmlFile, dSpaceID space);
  void update();

  static void fillContact(dContact &mod);

  void reset();

  static DContactType type;

  struct Ph ph;

private:

  static void fillContact();

  void createNodesAndMeshXml();
  void createPhysicsXml(dSpaceID s);
  void disposePhysicsXml();


  struct Cst cst;
};



#endif

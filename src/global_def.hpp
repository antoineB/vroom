//this file is necessary to be included in main.cpp

#ifndef GLOBAL_DEF_HPP
#define GLOBAL_DEF_HPP

#include <OGRE/OgreSceneNode.h>
#include <vector>

struct Global{
  bool ogreUp;
  bool worldUp;
  unsigned int nbFrame;
  void *cross;

  std::vector<Ogre::SceneNode*> collidingPoints;
};

#endif

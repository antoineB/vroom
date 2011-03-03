#ifndef DEMO_HPP
#define DEMO_HPP

#include "global.hpp"

#include "OgreFramework.hpp"
#include <ode/ode.h>

class demo : public OIS::KeyListener {

public:
  demo();
  ~demo();
  
  void startDemo();
	
  bool keyPressed(const OIS::KeyEvent &keyEventRef);
  bool keyReleased(const OIS::KeyEvent &keyEventRef);

private:
  void setupDemoScene();
  void runDemo();
  void forFrameDo(unsigned int time);
  
  
  Ogre::SceneNode*			m_pCubeNode;
  Ogre::Entity*				m_pCubeEntity;

  bool					m_bShutdown;

  void initOde();
  void endOde();
};

void createBall(char* name, dReal x, dReal y, dReal z);


#endif

#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <ode/ode.h>

//#include "OgreFramework.hpp"
#include <vector>
#include "global_def.hpp"

//declaration
 extern  std::vector<dGeomID> geoms;
 extern  std::vector<std::string> names;
 extern Global _glb;

//macros
#define log_(X) std::clog<<X<<std::endl;
#define logC_(X) { std::clog<<X<<std::endl; exit(-1); }
#define sceneMgr_ OgreFramework::getSingletonPtr()->m_pSceneMgr
#define timer_ OgreFramework::getSingletonPtr()->m_pTimer
#define renderWnd_ OgreFramework::getSingletonPtr()->m_pRenderWnd
#define windowMgr_ CEGUI::WindowManager::getSingletonPtr()

#define envUp_() {						\
   if (sceneMgr_ == NULL) {					\
     log_("Ogre isn't lunched before the car is set up");	\
     exit(0);							\
   }								\
								\
   if (_glb.worldUp == false) {					\
     log_("Ode isn't lunched before the car is set up");	\
     exit(0);							\
   }								\
 }; 								


//convert a string into a dReal
#define conv_(STRING, DREAL) {			\
  std::stringstream ss;				\
  ss << STRING;					\
  ss >> DREAL;					\
  };

#define convDS_(DREAL, STRING) {		\
  std::stringstream ss;				\
  ss << DREAL;					\
  STRING = ss.str();				\
  };


#define HARD_DEBUG 0
#define dbg_(x) if(HARD_DEBUG) std::cout<<x<<std::endl;

namespace BitField {
  unsigned long getCollideStaticEnvironement();
  unsigned long getCategorieStaticEnvironement();

};

#endif

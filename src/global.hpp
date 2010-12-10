#ifndef GLOBAL_HPP
#define GLOBAL_HPP

#include <ode/ode.h>
#include "OgreFramework.hpp"
#include <vector>
#include "global_def.hpp"

#ifndef CAR_HPP
#ifndef SPACE_HPP
#include "car.hpp"
#endif
#endif

//declaration
 extern  std::vector<dGeomID> geoms;
 extern  std::vector<std::string> names;
 extern Global _glb;

#ifndef CAR_HPP
#ifndef SPACE_HPP
extern Car car;
#endif
#endif

//macros
#define log_(X) OgreFramework::getSingletonPtr()->m_pLog->logMessage(X);
#define sceneMgr_ OgreFramework::getSingletonPtr()->m_pSceneMgr
#define timer_ OgreFramework::getSingletonPtr()->m_pTimer
#define renderWnd_ OgreFramework::getSingletonPtr()->m_pRenderWnd

//dirty
#define HARD_DEBUG 0
#define _dbg(x) if(HARD_DEBUG) std::cout<<x<<std::endl;

#endif

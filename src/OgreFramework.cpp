#include "OgreFramework.hpp"

using namespace Ogre; 

template<> OgreFramework* Ogre::Singleton<OgreFramework>::ms_Singleton = 0;

OgreFramework::OgreFramework(){
  m_MoveSpeed			= 0.1f;
  m_RotateSpeed		        = 0.3f;

  m_bShutDownOgre		        = false;
  m_iNumScreenShots	        = 0;

  m_pRoot				= 0;
  m_pSceneMgr			= 0;
  m_pRenderWnd		        = 0;
  m_pCamera			= 0;
  m_pViewport			= 0;
  m_pLog				= 0;
  m_pTimer			= 0;

  m_pInputMgr			= 0;
  m_pKeyboard			= 0;
  m_pMouse			= 0;

  m_pTrayMgr                      = 0;
  m_FrameEvent                    = Ogre::FrameEvent();

  inCarView = 1;
}

bool OgreFramework::initOgre(Ogre::String wndTitle, OIS::KeyListener *pKeyListener,
			     OIS::MouseListener *pMouseListener){

  Ogre::LogManager* logMgr = new Ogre::LogManager();
	
  m_pLog = Ogre::LogManager::getSingleton()
    .createLog("OgreLogfile.log", true, true, false);

  m_pLog->setDebugOutputEnabled(true);
	
  m_pRoot = new Ogre::Root();

  if(!m_pRoot->showConfigDialog())
    return false;
  m_pRenderWnd = m_pRoot->initialise(true, wndTitle);

  m_pSceneMgr = m_pRoot->createSceneManager(ST_GENERIC, "SceneManager");
  m_pSceneMgr->setAmbientLight(Ogre::ColourValue(0.7f, 0.7f, 0.7f));
	
  m_pCamera = m_pSceneMgr->createCamera("Camera");
  m_pCamera->setPosition(Vector3(0, 5, 5));
  m_pCamera->lookAt(Vector3(0, 0, 0));
  m_pCamera->setNearClipDistance(1);

  m_pViewport = m_pRenderWnd->addViewport(m_pCamera);
  m_pViewport->setBackgroundColour(ColourValue(0.8f, 0.7f, 0.6f, 1.0f));

  m_pCamera->setAspectRatio(Real(m_pViewport->getActualWidth()) / Real(m_pViewport->getActualHeight()));
	
  m_pViewport->setCamera(m_pCamera);

  unsigned long hWnd = 0;
  OIS::ParamList paramList;
  m_pRenderWnd->getCustomAttribute("WINDOW", &hWnd);

  paramList.insert(OIS::ParamList::value_type("WINDOW", Ogre::StringConverter::toString(hWnd)));

  m_pInputMgr = OIS::InputManager::createInputSystem(paramList);

  m_pKeyboard = static_cast<OIS::Keyboard*>(m_pInputMgr->createInputObject(OIS::OISKeyboard, true));
  m_pMouse = static_cast<OIS::Mouse*>(m_pInputMgr->createInputObject(OIS::OISMouse, true));
    
  m_pMouse->getMouseState().height = m_pRenderWnd->getHeight();
  m_pMouse->getMouseState().width	 = m_pRenderWnd->getWidth();

  if(pKeyListener == 0)
    m_pKeyboard->setEventCallback(this);
  else
    m_pKeyboard->setEventCallback(pKeyListener);

  if(pMouseListener == 0)
    m_pMouse->setEventCallback(this);
  else
    m_pMouse->setEventCallback(pMouseListener);

  Ogre::String secName, typeName, archName;
  Ogre::ConfigFile cf;
  cf.load("resources.cfg");

  Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
  while (seci.hasMoreElements())
    {
      secName = seci.peekNextKey();
      Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
      Ogre::ConfigFile::SettingsMultiMap::iterator i;
      for (i = settings->begin(); i != settings->end(); ++i)
	{
	  typeName = i->first;
	  archName = i->second;
	  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
	}
    }
  Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  m_pTimer = new Ogre::Timer();
  m_pTimer->reset();
	
  m_pTrayMgr = new OgreBites::SdkTrayManager("TrayMgr", m_pRenderWnd, m_pMouse, this);
  m_pTrayMgr->showFrameStats(OgreBites::TL_BOTTOMLEFT);
  m_pTrayMgr->showLogo(OgreBites::TL_BOTTOMRIGHT);
  //m_pTrayMgr->hideCursor();

  m_pRenderWnd->setActive(true);

  return true;
}

OgreFramework::~OgreFramework(){
  if(m_pInputMgr)
    OIS::InputManager::destroyInputSystem(m_pInputMgr);

  delete m_pRoot;
}

bool OgreFramework::keyPressed(const OIS::KeyEvent &keyEventRef){

  switch(keyEventRef.key){

  case OIS::KC_ESCAPE :
    m_bShutDownOgre = true;
    return true;

  case OIS::KC_SYSRQ :
    m_pRenderWnd->writeContentsToTimestampedFile("BOF_Screenshot_", ".png");
    return true;

  case OIS::KC_M :
    static int mode = 0;
		
    if(mode == 2){
      m_pCamera->setPolygonMode(PM_SOLID);
      mode = 0;
    }
    else if(mode == 0){
      m_pCamera->setPolygonMode(PM_WIREFRAME);
      mode = 1;
    }
    else if(mode == 1){
      m_pCamera->setPolygonMode(PM_POINTS);
      mode = 2;
    }
    break;

  case OIS::KC_O :
    if(m_pTrayMgr->isLogoVisible()){
      m_pTrayMgr->hideLogo();
      m_pTrayMgr->hideFrameStats();
    }
    else{
      m_pTrayMgr->showLogo(OgreBites::TL_BOTTOMRIGHT);
      m_pTrayMgr->showFrameStats(OgreBites::TL_BOTTOMLEFT);
    }
    break;

  case OIS::KC_C :
    inCarView=(inCarView+1)%4;
    break;
  }

  return true;
}

bool OgreFramework::keyReleased(const OIS::KeyEvent &keyEventRef){
  return true;
}

bool OgreFramework::mouseMoved(const OIS::MouseEvent &evt){
  m_pCamera->yaw(Degree(evt.state.X.rel * -0.1f));
  m_pCamera->pitch(Degree(evt.state.Y.rel * -0.1f));
	
  return true;
}

bool OgreFramework::mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id){
  return true;
}

bool OgreFramework::mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id){
  return true;
}

void OgreFramework::updateOgre(double timeSinceLastFrame){
  m_MoveScale = m_MoveSpeed   * (float)timeSinceLastFrame;
  m_RotScale  = m_RotateSpeed * (float)timeSinceLastFrame;
		
  m_TranslateVector = Vector3::ZERO;

  getInput();
  if (inCarView == 0)
    myMoveCamera();   
  else if (inCarView == 1) 
    moveCamera();  
  else if (inCarView == 2) 
    myMoveCamera2();
  else
    myMoveCamera3();

  m_FrameEvent.timeSinceLastFrame = timeSinceLastFrame;
  m_pTrayMgr->frameRenderingQueued(m_FrameEvent);
}

#define QUICK 3
void OgreFramework::moveCamera(){
  if(m_pKeyboard->isKeyDown(OIS::KC_LSHIFT)) 
    m_pCamera->moveRelative(m_TranslateVector);
  else 
    if (m_pKeyboard->isKeyDown(OIS::KC_LCONTROL))
	m_pCamera->moveRelative(m_TranslateVector*QUICK);
	else
	  m_pCamera->moveRelative(m_TranslateVector / 10);
}

void OgreFramework::myMoveCamera3(){
  extern Car car;
    /*Ogre::Vector3 vec = car.getPosition();
  vec.y += 6.0;
  vec.z += 8.0;
  m_pCamera->setPosition(vec);
  //  m_pCamera->setOrientation(car.getOrientation());
  m_pCamera->setDirection(car.getPosition());*/
  m_pCamera->detachFromParent();
  sceneMgr_->getSceneNode("cam_pos")->attachObject(m_pCamera);
  //  m_pCamera->setOrientation(car.getOrientation());
}

void OgreFramework::myMoveCamera(){
  //  m_pCamera->detachFromParent();
  //sceneMgr_->getSceneNode("Camera")->attachObject(m_pCamera);


  extern Car car;
  Ogre::Vector3 vec = car.getPosition();
  vec.y += 3.2;
  vec.z += 1.0;
  m_pCamera->setPosition(vec);
  m_pCamera->setOrientation(car.getOrientation());
}


void OgreFramework::myMoveCamera2() {
  //m_pCamera->detachFromParent();
  //sceneMgr_->getSceneNode("Camera")->attachObject(m_pCamera);

  extern Car car;
  m_pCamera->setDirection(car.getPosition());
}

void OgreFramework::getInput(){
  if(m_pKeyboard->isKeyDown(OIS::KC_Q))
    m_TranslateVector.x = -m_MoveScale;
	
  if(m_pKeyboard->isKeyDown(OIS::KC_D))
    m_TranslateVector.x = m_MoveScale;
	
  if(m_pKeyboard->isKeyDown(OIS::KC_Z))
    m_TranslateVector.z = -m_MoveScale;
	
  if(m_pKeyboard->isKeyDown(OIS::KC_S))
    m_TranslateVector.z = m_MoveScale;
}

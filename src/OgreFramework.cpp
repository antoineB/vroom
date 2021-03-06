#include "OgreFramework.hpp"
#include <ostream>
#include <stdio.h>
#include <math.h>

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

  moveCursor = false;
  editing = false;

  inCarView = false;
  inCarView2 = true;
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
	
  m_pRenderWnd->setActive(true);

  mRenderer = &CEGUI::OgreRenderer::bootstrapSystem();
  CEGUI::Imageset::setDefaultResourceGroup("Imagesets");
  CEGUI::Font::setDefaultResourceGroup("Fonts");
  CEGUI::Scheme::setDefaultResourceGroup("Schemes");
  CEGUI::WidgetLookManager::setDefaultResourceGroup("LookNFeel");
  CEGUI::WindowManager::setDefaultResourceGroup("Layouts");
  CEGUI::SchemeManager::getSingleton().create("VanillaSkin.scheme");
  CEGUI::FontManager::getSingleton().create("DejaVuSans-10.font");

  CEGUI::System::getSingleton().setDefaultMouseCursor("Vanilla-Images", "MouseArrow");
  CEGUI::MouseCursor::getSingleton().setImage( CEGUI::System::getSingleton().getDefaultMouseCursor());

  CEGUI::Window *guiRoot = CEGUI::WindowManager::getSingleton().loadWindowLayout("car.layout"); 
  
  CEGUI::Window *guiHud = CEGUI::WindowManager::getSingleton().loadWindowLayout("hud.layout");
  CEGUI::System::getSingleton().setGUISheet(guiHud);

  return true;
}

bool OgreFramework::setBackWheelsErp(const CEGUI::EventArgs &evt) {
    extern Car car;
    CEGUI::Window *w = windowMgr_->getWindow("root/wheels/back/erp");
    car.setBackWheelsErp(((CEGUI::Scrollbar*)w)->getScrollPosition());
    return true;
}
   
bool OgreFramework::setBackWheelsCfm(const CEGUI::EventArgs &evt) {
  extern Car car;
  CEGUI::Window *w = windowMgr_->getWindow("root/wheels/back/cfm");
  car.setBackWheelsCfm(((CEGUI::Scrollbar*)w)->getScrollPosition());
  return true;
}

bool OgreFramework::setFrontWheelsErp(const CEGUI::EventArgs &evt) {
    extern Car car;
    CEGUI::Window *w = windowMgr_->getWindow("root/wheels/front/erp");
    car.setFrontWheelsErp(((CEGUI::Scrollbar*)w)->getScrollPosition());
    return true;
}

bool OgreFramework::setFrontWheelsCfm(const CEGUI::EventArgs &evt) {
  extern Car car;
  CEGUI::Window *w = windowMgr_->getWindow("root/wheels/front/cfm");
  car.setFrontWheelsCfm(((CEGUI::Scrollbar*)w)->getScrollPosition());
  return true;
}

static CEGUI::MouseButton convertButton(OIS::MouseButtonID buttonID) {
  switch (buttonID) {
  case OIS::MB_Left:
    return CEGUI::LeftButton;
    
  case OIS::MB_Right:
    return CEGUI::RightButton;
    
  case OIS::MB_Middle:
    return CEGUI::MiddleButton;
    
  default:
    return CEGUI::LeftButton;
  }
}

OgreFramework::~OgreFramework(){
  if(m_pInputMgr)
    OIS::InputManager::destroyInputSystem(m_pInputMgr);

  delete m_pRoot;
}

static dReal norm(const dReal *V) {
  return sqrt(pow(V[0], 2) + pow(V[1], 2) + pow(V[2], 2));
}

void OgreFramework::updateGui() {
  static double timeRefresh = 0;

  if (!editing) {
    if (timer_->getMillisecondsCPU() - timeRefresh > 250) {
      extern Car car;
      const dReal *V = car.getSpeed();
      char buf[30];
      sprintf(buf, "%.2f %.2f %.2f / %.2f", V[0], V[1], V[2], norm(V));
      windowMgr_->getWindow("hud/speed")->setText("speed " + std::string(buf));

      sprintf(buf, "FPS  %.2f   avg: %.2f", m_pRenderWnd->getLastFPS(), m_pRenderWnd->getAverageFPS());
      windowMgr_->getWindow("hud/fps")->setText(std::string(buf));
        
      timeRefresh = timer_->getMillisecondsCPU();
    }
  }
}

bool OgreFramework::keyPressed(const OIS::KeyEvent &keyEventRef){
  if (editing) {
    CEGUI::System &sys = CEGUI::System::getSingleton();
    sys.injectKeyDown(keyEventRef.key);
    sys.injectChar(keyEventRef.text);
    //return true; make the rest of the function unUsable
  }

  switch (keyEventRef.key) {

  case OIS::KC_E :
    editing = !editing;

    if (!editing) {
      moveCursor = false;
      windowMgr_->getWindow("root")->setVisible(false);
      CEGUI::MouseCursor::getSingletonPtr()->setVisible(false);    
      
      CEGUI::System::getSingleton().setGUISheet(windowMgr_->getWindow("hud"));
      windowMgr_->getWindow("hud")->setVisible(true);
    }
    else {
      CEGUI::System::getSingleton().setGUISheet(windowMgr_->getWindow("root"));
      moveCursor = true;
      windowMgr_->getWindow("root")->setVisible(true);
      CEGUI::MouseCursor::getSingletonPtr()->setVisible(true);
      CEGUI::Size s(
		    (float)(m_pViewport->getWidth() * m_pRenderWnd->getWidth()), 
		    (float)(m_pViewport->getHeight() * m_pRenderWnd->getHeight())
		    );
      CEGUI::System::getSingletonPtr()->notifyDisplaySizeChanged(s);
    }

    return true;

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

  case OIS::KC_C :
    inCarView = !inCarView;
    break;
  }

  return true;
}

bool OgreFramework::keyReleased(const OIS::KeyEvent &keyEventRef){
  if (editing) {
    CEGUI::System::getSingleton().injectKeyUp(keyEventRef.key);
  }
  return true;
}

bool OgreFramework::mouseMoved(const OIS::MouseEvent &evt){
  if (!moveCursor) {
    m_pCamera->yaw(Degree(evt.state.X.rel * -0.1f));
    m_pCamera->pitch(Degree(evt.state.Y.rel * -0.1f));
  }
  else {
    CEGUI::System &sys = CEGUI::System::getSingleton();
    sys.injectMouseMove(evt.state.X.rel, evt.state.Y.rel);
    // Scroll wheel.
    if (evt.state.Z.rel)
      sys.injectMouseWheelChange(evt.state.Z.rel / 120.0f);
  }
	
  return true;
}

bool OgreFramework::mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id){
  CEGUI::System::getSingleton().injectMouseButtonDown(convertButton(id));
  return true;
}

bool OgreFramework::mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id){
  CEGUI::System::getSingleton().injectMouseButtonUp(convertButton(id));
  return true;
}

void OgreFramework::updateOgre(double timeSinceLastFrame){
  m_MoveScale = m_MoveSpeed   * (float)timeSinceLastFrame;
  m_RotScale  = m_RotateSpeed * (float)timeSinceLastFrame;
		
  m_TranslateVector = Vector3::ZERO;

  getInput();

  if (inCarView)
    myMoveCamera();   
  else
    moveCamera();  
}

#define QUICK 3
void OgreFramework::moveCamera() {
  if (inCarView2) {
    m_pCamera->detachFromParent();

    m_pCamera->setPosition(Vector3(0, 5, 5));
    m_pCamera->lookAt(Vector3(0, 0, 0));

    inCarView2 = false;
  }
    
  if(m_pKeyboard->isKeyDown(OIS::KC_LSHIFT)) 
    m_pCamera->moveRelative(m_TranslateVector);
  else 
    if (m_pKeyboard->isKeyDown(OIS::KC_LCONTROL))
	m_pCamera->moveRelative(m_TranslateVector*QUICK);
	else
	  m_pCamera->moveRelative(m_TranslateVector / 10);
}

void OgreFramework::myMoveCamera() {
  if (!inCarView2) {
    m_pCamera->detachFromParent();
    sceneMgr_->getSceneNode("cam_pos")->attachObject(m_pCamera);
    m_pCamera->setPosition(0, 0, 0);
    m_pCamera->setOrientation(sceneMgr_->getSceneNode("cam_target")->getOrientation());
    
    inCarView2 = true;
  }
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

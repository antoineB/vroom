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

static void setCarMass(Conf::Car::Param &mod) {
  CEGUI::Window *mass = windowMgr_->getWindow("root/car_param/mass/total/value");
  CEGUI::Window *x = windowMgr_->getWindow("root/car_param/mass/pos/x");
  CEGUI::Window *y = windowMgr_->getWindow("root/car_param/mass/pos/y");
  CEGUI::Window *z = windowMgr_->getWindow("root/car_param/mass/pos/z");
  
  dReal v[4];
  conv_(mass->getText(), v[0]);
  conv_(x->getText(), v[1]);
  conv_(y->getText(), v[2]);
  conv_(z->getText(), v[3]);

  dMassSetBoxTotal(&mod.mass, v[0], v[1], v[2], v[3]);
}

static void setCarBox(Conf::Car::Param &mod) {
  CEGUI::Window *x = windowMgr_->getWindow("root/car_param/geom/box/x");
  CEGUI::Window *y = windowMgr_->getWindow("root/car_param/geom/box/y");
  CEGUI::Window *z = windowMgr_->getWindow("root/car_param/geom/box/z");
  
  conv_(x->getText(), mod.box[0]);
  conv_(y->getText(), mod.box[1]);
  conv_(z->getText(), mod.box[2]);
}

static void setCarOffset(Conf::Car::Param &mod) {
  CEGUI::Window *x = windowMgr_->getWindow("root/car_param/geom/offset/x");
  CEGUI::Window *y = windowMgr_->getWindow("root/car_param/geom/offset/y");
  CEGUI::Window *z = windowMgr_->getWindow("root/car_param/geom/offset/z");
  
  conv_(x->getText(), mod.offset[0]);
  conv_(y->getText(), mod.offset[1]);
  conv_(z->getText(), mod.offset[2]);
}

static void setCarJoints(Conf::Car::Param &mod) {
  CEGUI::Window *x = windowMgr_->getWindow("root/car_param/joint/front/left/axis1/x");
  CEGUI::Window *y = windowMgr_->getWindow("root/car_param/joint/front/left/axis1/y");
  CEGUI::Window *z = windowMgr_->getWindow("root/car_param/joint/front/left/axis1/z");
  conv_(x->getText(), mod.axis1[0][0]);
  conv_(y->getText(), mod.axis1[0][1]);
  conv_(z->getText(), mod.axis1[0][2]);

  x = windowMgr_->getWindow("root/car_param/joint/front/left/axis2/x");
  y = windowMgr_->getWindow("root/car_param/joint/front/left/axis2/y");
  z = windowMgr_->getWindow("root/car_param/joint/front/left/axis2/z");
  conv_(x->getText(), mod.axis2[0][0]);
  conv_(y->getText(), mod.axis2[0][1]);
  conv_(z->getText(), mod.axis2[0][2]);

  x = windowMgr_->getWindow("root/car_param/joint/front/right/axis1/x");
  y = windowMgr_->getWindow("root/car_param/joint/front/right/axis1/y");
  z = windowMgr_->getWindow("root/car_param/joint/front/right/axis1/z");
  conv_(x->getText(), mod.axis1[1][0]);
  conv_(y->getText(), mod.axis1[1][1]);
  conv_(z->getText(), mod.axis1[1][2]);

  x = windowMgr_->getWindow("root/car_param/joint/front/right/axis2/x");
  y = windowMgr_->getWindow("root/car_param/joint/front/right/axis2/y");
  z = windowMgr_->getWindow("root/car_param/joint/front/right/axis2/z");
  conv_(x->getText(), mod.axis2[1][0]);
  conv_(y->getText(), mod.axis2[1][1]);
  conv_(z->getText(), mod.axis2[1][2]);

  x = windowMgr_->getWindow("root/car_param/joint/back/left/axis1/x");
  y = windowMgr_->getWindow("root/car_param/joint/back/left/axis1/y");
  z = windowMgr_->getWindow("root/car_param/joint/back/left/axis1/z");
  conv_(x->getText(), mod.axis1[2][0]);
  conv_(y->getText(), mod.axis1[2][1]);
  conv_(z->getText(), mod.axis1[2][2]);

  x = windowMgr_->getWindow("root/car_param/joint/back/left/axis2/x");
  y = windowMgr_->getWindow("root/car_param/joint/back/left/axis2/y");
  z = windowMgr_->getWindow("root/car_param/joint/back/left/axis2/z");
  conv_(x->getText(), mod.axis2[2][0]);
  conv_(y->getText(), mod.axis2[2][1]);
  conv_(z->getText(), mod.axis2[2][2]);

  x = windowMgr_->getWindow("root/car_param/joint/back/right/axis1/x");
  y = windowMgr_->getWindow("root/car_param/joint/back/right/axis1/y");
  z = windowMgr_->getWindow("root/car_param/joint/back/right/axis1/z");
  conv_(x->getText(), mod.axis1[3][0]);
  conv_(y->getText(), mod.axis1[3][1]);
  conv_(z->getText(), mod.axis1[3][2]);

  x = windowMgr_->getWindow("root/car_param/joint/back/right/axis2/x");
  y = windowMgr_->getWindow("root/car_param/joint/back/right/axis2/y");
  z = windowMgr_->getWindow("root/car_param/joint/back/right/axis2/z");
  conv_(x->getText(), mod.axis2[3][0]);
  conv_(y->getText(), mod.axis2[3][1]);
  conv_(z->getText(), mod.axis2[3][2]);
}

static void setFlatGroundContact(Conf::FlatGround::Param &mod) {
    CEGUI::Window *mu = windowMgr_->getWindow("root/contact/ground/mu/value");
    CEGUI::Window *bounce = windowMgr_->getWindow("root/contact/ground/bounce/value");
    CEGUI::Window *bounceVel = windowMgr_->getWindow("root/contact/ground/bounce_vel/value");
    CEGUI::Window *erp = windowMgr_->getWindow("root/contact/ground/erp/value");
    CEGUI::Window *cfm = windowMgr_->getWindow("root/contact/ground/cfm/value");
    CEGUI::Window *slip1 = windowMgr_->getWindow("root/contact/ground/slip1/value");
    CEGUI::Window *slip2 = windowMgr_->getWindow("root/contact/ground/slip2/value");
    
    if (mu->getText() == "inf")
      mod.contact.surface.mu = dInfinity;
    else
      conv_(mu->getText(), mod.contact.surface.mu);
    conv_(bounce->getText(), mod.contact.surface.bounce);
    conv_(bounceVel->getText(), mod.contact.surface.bounce_vel);
    conv_(erp->getText(), mod.contact.surface.soft_erp);
    conv_(cfm->getText(), mod.contact.surface.soft_cfm);
    conv_(slip1->getText(), mod.contact.surface.slip1);
    conv_(slip2->getText(), mod.contact.surface.slip2);
}

static void setObstacleContact(Conf::Obstacle::Param &mod) {
  //the same as ground for now
    CEGUI::Window *mu = windowMgr_->getWindow("root/contact/ground/mu/value");
    CEGUI::Window *bounce = windowMgr_->getWindow("root/contact/ground/bounce/value");
    CEGUI::Window *bounceVel = windowMgr_->getWindow("root/contact/ground/bounce_vel/value");
    CEGUI::Window *erp = windowMgr_->getWindow("root/contact/ground/erp/value");
    CEGUI::Window *cfm = windowMgr_->getWindow("root/contact/ground/cfm/value");
    CEGUI::Window *slip1 = windowMgr_->getWindow("root/contact/ground/slip1/value");
    CEGUI::Window *slip2 = windowMgr_->getWindow("root/contact/ground/slip2/value");

    if (mu->getText() == "inf")
      mod.contact.surface.mu = dInfinity;
    else
      conv_(mu->getText(), mod.contact.surface.mu);
    conv_(bounce->getText(), mod.contact.surface.bounce);
    conv_(bounceVel->getText(), mod.contact.surface.bounce_vel);
    conv_(erp->getText(), mod.contact.surface.soft_erp);
    conv_(cfm->getText(), mod.contact.surface.soft_cfm);
    conv_(slip1->getText(), mod.contact.surface.slip1);
    conv_(slip2->getText(), mod.contact.surface.slip2);
}


static void setCarContact(Conf::Car::Param &mod) {
    CEGUI::Window *mu = windowMgr_->getWindow("root/contact/car/mu/value");
    CEGUI::Window *bounce = windowMgr_->getWindow("root/contact/car/bounce/value");
    CEGUI::Window *bounceVel = windowMgr_->getWindow("root/contact/car/bounce_vel/value");
    CEGUI::Window *erp = windowMgr_->getWindow("root/contact/car/erp/value");
    CEGUI::Window *cfm = windowMgr_->getWindow("root/contact/car/cfm/value");
    CEGUI::Window *slip1 = windowMgr_->getWindow("root/contact/car/slip1/value");
    CEGUI::Window *slip2 = windowMgr_->getWindow("root/contact/car/slip2/value");

    if (mu->getText() == "inf")
      mod.contact.surface.mu = dInfinity;
    else
      conv_(mu->getText(), mod.contact.surface.mu);
    conv_(bounce->getText(), mod.contact.surface.bounce);
    conv_(bounceVel->getText(), mod.contact.surface.bounce_vel);
    conv_(erp->getText(), mod.contact.surface.soft_erp);
    conv_(cfm->getText(), mod.contact.surface.soft_cfm);
    conv_(slip1->getText(), mod.contact.surface.slip1);
    conv_(slip2->getText(), mod.contact.surface.slip2);
}

static void setWheelContact(Conf::Car::Param &mod) {
    CEGUI::Window *mu = windowMgr_->getWindow("root/contact/wheel/mu/value");
    CEGUI::Window *bounce = windowMgr_->getWindow("root/contact/wheel/bounce/value");
    CEGUI::Window *bounceVel = windowMgr_->getWindow("root/contact/wheel/bounce_vel/value");
    CEGUI::Window *erp = windowMgr_->getWindow("root/contact/wheel/erp/value");
    CEGUI::Window *cfm = windowMgr_->getWindow("root/contact/wheel/cfm/value");
    CEGUI::Window *slip1 = windowMgr_->getWindow("root/contact/wheel/slip1/value");
    CEGUI::Window *slip2 = windowMgr_->getWindow("root/contact/wheel/slip2/value");

    if (mu->getText() == "inf")
      mod.contact.surface.mu = dInfinity;
    else
      conv_(mu->getText(), mod.wheel_contact.surface.mu);
    conv_(bounce->getText(), mod.wheel_contact.surface.bounce);
    conv_(bounceVel->getText(), mod.wheel_contact.surface.bounce_vel);
    conv_(erp->getText(), mod.wheel_contact.surface.soft_erp);
    conv_(cfm->getText(), mod.wheel_contact.surface.soft_cfm);
    conv_(slip1->getText(), mod.wheel_contact.surface.slip1);
    conv_(slip2->getText(), mod.wheel_contact.surface.slip2);
}

bool OgreFramework::changeCarValue(const CEGUI::EventArgs &evt) {
  extern Car car;
  Conf::Param mod;

  setCarMass(mod.car);
  setCarBox(mod.car);
  setCarOffset(mod.car);
  setCarJoints(mod.car);

  setFlatGroundContact(mod.ground);
  setObstacleContact(mod.obstacles);
  setCarContact(mod.car);
  setWheelContact(mod.car);

  car.reset(mod);
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
    inCarView=(inCarView+1)%4;
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
  if (inCarView == 0)
    myMoveCamera();   
  else if (inCarView == 1) 
    moveCamera();  
  else if (inCarView == 2) 
    myMoveCamera2();
  else
    myMoveCamera3();
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
  m_pCamera->detachFromParent();
  sceneMgr_->getSceneNode("cam_pos")->attachObject(m_pCamera);
}

void OgreFramework::myMoveCamera(){
  extern Car car;
  Ogre::Vector3 vec = car.getPosition();
  vec.y += 3.2;
  vec.z += 1.0;
  m_pCamera->setPosition(vec);
  m_pCamera->setOrientation(car.getOrientation());
}


void OgreFramework::myMoveCamera2() {
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

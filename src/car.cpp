#include "car.hpp"
#include "mytools.hpp"
#include "world.hpp"
#include <math.h>

using namespace Utils;

static void updateJointsGraph(Car *c) {

	for (int i = 0; i < 4; ++i) {

		dVector3 tmpAnchor2, tmpAnchor1, tmpAxis;
		dJointGetHinge2Anchor2(c->ph.joints[i], tmpAnchor2); //not sure that the axis are the good one
		dJointGetHinge2Anchor(c->ph.joints[i], tmpAnchor1);
		dJointGetHinge2Axis1(c->ph.joints[i], tmpAxis);

		Ogre::Vector3 anchor2((Ogre::Real*) tmpAnchor2);
		Ogre::Vector3 anchor1((Ogre::Real*) tmpAnchor1);
		Ogre::Vector3 axis((Ogre::Real*) tmpAxis);
		float displacement;

		displacement = (anchor1 - anchor2).dotProduct(axis);

		Ogre::SceneNode *nn = sceneMgr_->getSceneNode(
						c->cst.nodeName + "_anchor1_" + Ogre::StringConverter::toString(i));

		nn->setScale(1.5, 2.8 + 4*displacement, 1.5);
	}
}

static void createJointsGraph(Car *c) {
	for (int i = 0; i < 4; i++) {
		Ogre::SceneNode *nn = c->cst.carNode->createChildSceneNode(
				c->cst.nodeName + "_anchor1_" + Ogre::StringConverter::toString(i));



		dVector3 tmpAnchor1;
		dJointGetHinge2Anchor(c->ph.joints[i], tmpAnchor1);
		Ogre::Vector3 anchor1((Ogre::Real*) tmpAnchor1);
		nn->setPosition(anchor1);

		if (i % 2 == 0) {
			nn->translate(-0.25, -4.3, 0);
			nn->scale(1, 1.75, 1);
			Ogre::Degree a(30);
			nn->roll(Ogre::Radian(a.valueRadians()));
		}
		else {
			nn->translate(0.25, -4.3, 0);
			nn->scale(1, 1.75, 1);
			Ogre::Degree a(-30);
			nn->roll(Ogre::Radian(a.valueRadians()));
		}

		Ogre::Entity *ee = sceneMgr_->createEntity("spring.mesh");
		nn->attachObject(ee);
	}
}

void Car::update() {
	//graphical
	MyTools::byOdeToOgre(ph.geom, cst.carNode);

	for (int i = 0; i < 4; i++)
		wheels[i].update();

	updateJointsGraph(this);

	//physical
	updateMotor();
	updateSteering();
}

Car::~Car() {
}

Car::Car() :
  brake(false), speed(0.0), steer(0.0), 
  type(Type::UNDEFINED), spaceType(Type::UNDEFINED) {}

void Car::createSpace() {
	space = World::getSingletonPtr()->addSimpleSpace();
	//to avoid destroying geoms when the space is destroyed
	dSpaceSetCleanup(space, 0);
}

void Car::createPhysics(Utils::Xml &x) {
	Ogre::Entity *e = sceneMgr_->createEntity(x.mustString("body"));
	dTriMeshDataID data = MyTools::dTriMeshDataFromMesh(e);
	ph.geom = addTriMesh(data);

	dMassSetTrimeshTotal(&ph.mass, x.mustOReal("mass"), ph.geom);
	ph.mass.c[0] = 0;
	ph.mass.c[1] = 0;
	ph.mass.c[2] = 0;
	ph.body = World::getSingletonPtr()->add(ph.geom, &ph.mass);

	cst.brakeForce = x.mustOReal("engine.brake-force");
	cst.gasForce = x.mustOReal("engine.gas-force");
	cst.steeringForce = x.mustOReal("engine.steering-force");
	cst.lowRiderForce = x.mustOReal("lowrider-force");

	createJoints(x);
}

void Car::createJoints(Utils::Xml &x) {
	for (int i = 0; i < 4; i++) {
		ph.joints[i] = World::getSingletonPtr()->addHinge2(ph.body,
				wheels[i].ph.body, 0);
		dJointSetHinge2Param(ph.joints[i], dParamLoStop, 0);
		dJointSetHinge2Param(ph.joints[i], dParamHiStop, 0);

		if (i > 1) { //to get the the back wheel not rotate in y
			dJointSetHinge2Param(ph.joints[i], dParamStopERP, 1.0);
			dJointSetHinge2Param(ph.joints[i], dParamStopCFM, 0.0);
		}
	}
	dJointSetHinge2Param(ph.joints[0], dParamSuspensionERP,
			x.mustOReal("joints.front-right.erp"));
	dJointSetHinge2Param(ph.joints[0], dParamSuspensionCFM,
			x.mustOReal("joints.front-right.cfm"));

	dJointSetHinge2Param(ph.joints[1], dParamSuspensionERP,
			x.mustOReal("joints.front-left.erp"));
	dJointSetHinge2Param(ph.joints[1], dParamSuspensionCFM,
			x.mustOReal("joints.front-left.cfm"));

	dJointSetHinge2Param(ph.joints[2], dParamSuspensionERP,
			x.mustOReal("joints.back-right.erp"));
	dJointSetHinge2Param(ph.joints[2], dParamSuspensionCFM,
			x.mustOReal("joints.back-right.cfm"));

	dJointSetHinge2Param(ph.joints[3], dParamSuspensionERP,
			x.mustOReal("joints.back-left.erp"));
	dJointSetHinge2Param(ph.joints[3], dParamSuspensionCFM,
			x.mustOReal("joints.back-left.cfm"));

}

void Car::disposeGeoms(Utils::Xml &x) {
	Ogre::Real a = x.mustOReal("gravity-center.x");
	Ogre::Real b = x.mustOReal("gravity-center.y");
	Ogre::Real c = x.mustOReal("gravity-center.z");

	dBodySetPosition(ph.body, a, b, c);
	dGeomSetOffsetPosition(ph.geom, x.mustOReal("global-position.x") - a,
			x.mustOReal("global-position.y") - b,
			x.mustOReal("global-position.z") - c);
}

void Car::disposeJoints(Utils::Xml &x) {
	Ogre::Real a = x.mustOReal("global-position.x");
	Ogre::Real b = x.mustOReal("global-position.y");
	Ogre::Real c = x.mustOReal("global-position.z");

	std::string uris[] = { "../xml/" + x.mustString("wheels.uri", 0), "../xml/"
			+ x.mustString("wheels.uri", 1), "../xml/"
			+ x.mustString("wheels.uri", 2), "../xml/"
			+ x.mustString("wheels.uri", 3) };

	std::string names[] = { "joints.front-right.axis1.x",
			"joints.front-right.axis1.y", "joints.front-right.axis1.z",
			"joints.front-right.axis2.x", "joints.front-right.axis2.y",
			"joints.front-right.axis2.z",

			"joints.front-left.axis1.x", "joints.front-left.axis1.y",
			"joints.front-left.axis1.z", "joints.front-left.axis2.x",
			"joints.front-left.axis2.y", "joints.front-left.axis2.z",

			"joints.back-right.axis1.x", "joints.back-right.axis1.y",
			"joints.back-right.axis1.z", "joints.back-right.axis2.x",
			"joints.back-right.axis2.y", "joints.back-right.axis2.z",

			"joints.back-left.axis1.x", "joints.back-left.axis1.y",
			"joints.back-left.axis1.z", "joints.back-left.axis2.x",
			"joints.back-left.axis2.y", "joints.front-left.axis2.z" };

	for (int i = 0; i < 4; i++) {
		{
			Utils::Xml w(uris[i].c_str(), "wheel");
			dJointSetHinge2Anchor(ph.joints[i], a + w.mustOReal("position.x"),
					b + w.mustOReal("position.y"),
					c + w.mustOReal("position.z"));

			std::cout << a + w.mustOReal("position.x") << " - "
					<< b + w.mustOReal("position.y") << " - "
					<< c + w.mustOReal("position.z") << std::endl;

		}
		dJointSetHinge2Axis1(ph.joints[i], x.mustOReal(names[i * 6].c_str()),
				x.mustOReal(names[i * 6 + 1].c_str()),
				x.mustOReal(names[i * 6 + 2].c_str()));
		dJointSetHinge2Axis2(ph.joints[i],
				x.mustOReal(names[i * 6 + 3].c_str()),
				x.mustOReal(names[i * 6 + 4].c_str()),
				x.mustOReal(names[i * 6 + 5].c_str()));
	}
}

void Car::createNodesAndMeshes(Utils::Xml &x) {
	Ogre::SceneNode *node = sceneMgr_->getRootSceneNode()->createChildSceneNode(
			cst.nodeName);
	Ogre::SceneNode *fnode = node->createChildSceneNode("ford");

	cst.carNode = node;
	cst.subCarNode = fnode;

	fnode->scale(0.35, 0.35, 0.35);
	fnode->yaw(Ogre::Degree(x.mustOReal("rotation.y")));
	//fnode->translate(0.0, 1.9, 0.0);

	std::string names[] = { "bonet", "back", "front", "bottom", "top",
			"wind_window", "back_top", "back_window", "wind_window_frame",
			"left_back", "left_front", "right_back", "right_front", "left_door",
			"left_little_window", "left_window", "right_door",
			"right_little_window", "right_window" };

	for (int i = 0; i < 19; i++) {
		std::string mesh("meshes." + names[i]);
		std::string mat("materials." + names[i]);
		createAndAttachEntity(names[i], x.mustString(mesh.c_str()),
				x.mustString(mat.c_str()), fnode);
	}

	//  createLeftDoorGraphic();
	//  createRightDoorGraphic();
}

void Car::initXml(const char *xmlFile, Ogre::SceneNode *root) {
	envUp_();

	createSpace();

	Utils::Xml x(xmlFile, "car");

	cst.nodeName = x.mustString("name");

	createNodesAndMeshes(x);

	createCamNodes(x);

	std::string uris[] = { "../xml/" + x.mustString("wheels.uri", 0), "../xml/"
			+ x.mustString("wheels.uri", 1), "../xml/"
			+ x.mustString("wheels.uri", 2), "../xml/"
			+ x.mustString("wheels.uri", 3) };

	for (int i = 0; i < 4; i++)
		wheels[i].initXml(uris[i].c_str(), space);

	createPhysics(x);

	disposeGeoms(x);
	disposeJoints(x);

	setupContacs(x);

	MyTools::byOdeToOgre(ph.geom, cst.carNode);

	createJointsGraph(this);
}

void Car::setupContacs(Utils::Xml &x) {
  x.fillDContact("contact", this->type);
  x.fillDContact("space-contact", this->spaceType);
  
  dGeomSetData(ph.geom, &(this->type));
  dGeomSetData((dGeomID) space, &(this->spaceType));
}

void Car::printRotationMatrix() {
	const dReal *R = dGeomGetRotation(ph.geom);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << i << " " << j << " = " << R[i + j] << std::endl;
		}
	}
}

void Car::rotateWheels(dMatrix3 *R) {
	for (int i = 0; i < 3; i++)
		dGeomSetRotation(wheels[i].ph.geom, *R);
}

void Car::accelerate() {
	speed += 10.5;
}

void Car::slowDown() {
	speed -= 20.5;
}

void Car::setSpeed(float s) {
	speed = s;
}

void Car::turnRight() {
	steer += 0.08;
}

void Car::turnLeft() {
	steer -= 0.08;
}

void Car::setSteer(float s) {
	steer = s;
}

const dReal* Car::getSpeed() {
	return dBodyGetLinearVel(ph.body);
}

void Car::updateSteering() {
	static float st = 0.0;
	if (steer == 0) {
		st = 0;
		for (int i = 0; i < 2; i++) {
			dJointSetHinge2Param(ph.joints[i], dParamLoStop, 0);
			dJointSetHinge2Param(ph.joints[i], dParamHiStop, 0);
		}
	} else {
		st += steer;

		for (int i = 0; i < 2; i++) {
			dReal v = st - dJointGetHinge2Angle1(ph.joints[i]);
			if (v > 0.1)
				v = 1;
			if (v < -0.1)
				v = -1;

			dJointSetHinge2Param(ph.joints[i], dParamVel, v);
			dJointSetHinge2Param(ph.joints[i], dParamFMax, cst.steeringForce);
			dJointSetHinge2Param(ph.joints[i], dParamLoStop, -0.65);
			dJointSetHinge2Param(ph.joints[i], dParamHiStop, 0.65);
			dJointSetHinge2Param(ph.joints[i], dParamFudgeFactor, 1.0);
		}
	}
}

void Car::lowRideFront() {
	const dReal *p = dGeomGetPosition(wheels[1].ph.geom);
	dBodyAddForceAtPos(ph.body, 0.0, cst.lowRiderForce, 0.0, p[0], p[1], p[2]);
}

void Car::lowRideBack() {
	const dReal *p = dGeomGetPosition(wheels[2].ph.geom);
	dBodyAddForceAtPos(ph.body, 0.0, cst.lowRiderForce, 0.0, p[0], p[1], p[2]);
}

void Car::setBrake(bool b) {
	brake = b;
	if (brake == true) {
		type.contact.surface.slip1 = 1.0;
		type.contact.surface.slip2 = 1.0;
	} else {
		type.contact.surface.slip1 = 0.5;
		type.contact.surface.slip2 = 0.5;
	}
}

void Car::updateMotor() {
	if (brake) {
		for (int i = 2; i < 4; i++) {
			dJointSetHinge2Param(ph.joints[i], dParamVel2, 0);
			dJointSetHinge2Param(ph.joints[i], dParamFMax2, cst.brakeForce);
		}
		return;
	}

	static float sp = 0.0;
	if (speed == 0) {
		sp = 0;
		for (int i = 2; i < 4; i++)
			dJointSetHinge2Param(ph.joints[i], dParamFMax2, 0.01);
		return;
	}

	if (speed < -10)
		speed = -10;
	else if (speed > 10)
		speed = 10;

	sp += speed;
	if (sp < -100)
		sp = -100;
	else if (sp > 10)
		sp = 10;

	for (int i = 2; i < 4; i++) {
		dJointSetHinge2Param(ph.joints[i], dParamVel2, sp);
		dJointSetHinge2Param(ph.joints[i], dParamFMax2, cst.gasForce);
	}
}

Ogre::Vector3 Car::cam() {
	const dReal *pos = dBodyGetPosition(ph.body);
	return Ogre::Vector3((Ogre::Real) pos[0], (Ogre::Real) pos[1] + 3.2,
			(Ogre::Real) pos[2]);
}

Ogre::Vector3 Car::getPosition() {
	return cst.carNode->getPosition();
}

Ogre::Vector3 Car::getDirection() {
	const dReal *pos = dBodyGetPosition(ph.body);
	return Ogre::Vector3((Ogre::Real) pos[0], (Ogre::Real) pos[1],
			(Ogre::Real) pos[2]);
	//  return sceneMgr_->getSceneNode(nodeName.c_str())->getDirection();
}

Ogre::Quaternion Car::getOrientation() {
	return cst.carNode->getOrientation();
}

void Car::swayBars() {
	const float swayForce = 400.0;
	const float swayForceLimit = 40.0;

	for (int i = 0; i < 4; ++i) {

		dVector3 tmpAnchor2, tmpAnchor1, tmpAxis;
		dJointGetHinge2Anchor2(ph.joints[i], tmpAnchor2); //not sure that the axis are the good one
		dJointGetHinge2Anchor(ph.joints[i], tmpAnchor1);
		dJointGetHinge2Axis1(ph.joints[i], tmpAxis);

		Ogre::Vector3 anchor2((Ogre::Real*) tmpAnchor2);
		Ogre::Vector3 anchor1((Ogre::Real*) tmpAnchor1);
		Ogre::Vector3 axis((Ogre::Real*) tmpAxis);
		float displacement;

		displacement = (anchor1 - anchor2).dotProduct(axis);

		if (displacement > 0) {
			//      std::cout<<"displacement > 0"<<std::endl;
			float amt = displacement * swayForce;
			if (amt > swayForceLimit) {
				amt = swayForceLimit;
				std::cout << "limit reach" << std::endl;
			}
			//the axis are inversed
			//dBodyAddForce( w[i].getBody(), -axis.x * amt, -axis.y * amt, -axis.z * amt );
			dReal const * wp = dBodyGetPosition(wheels[i].ph.body);
			dBodyAddForceAtPos(ph.body, -axis.x * amt, -axis.y * amt,
					-axis.z * amt, wp[0], wp[1], wp[2]);
			//dBodyAddForce( w[i^1].getBody(), axis.x * amt, axis.y * amt, axis.z * amt );
			wp = dBodyGetPosition(wheels[i ^ 1].ph.body);
			dBodyAddForceAtPos(ph.body, axis.x * amt, axis.y * amt,
					axis.z * amt, wp[0], wp[1], wp[2]);
		}
	}
}

dReal Car::getPunch() {
	const dReal *V = dBodyGetLinearVel(ph.body);
	return sqrt(pow(V[0], 2) + pow(V[1], 2) + pow(V[2], 2)) * ph.mass.mass;
}

dReal Car::getFrontWheelsErp() {
	return dJointGetHinge2Param(ph.joints[0], dParamSuspensionERP);
}

dReal Car::getBackWheelsErp() {
	return dJointGetHinge2Param(ph.joints[2], dParamSuspensionERP);
}

void Car::setBackWheelsErp(dReal erp) {
	dJointSetHinge2Param(ph.joints[2], dParamSuspensionERP, erp);
	dJointSetHinge2Param(ph.joints[3], dParamSuspensionERP, erp);
}

void Car::setFrontWheelsErp(dReal erp) {
	dJointSetHinge2Param(ph.joints[0], dParamSuspensionERP, erp);
	dJointSetHinge2Param(ph.joints[1], dParamSuspensionERP, erp);
}

dReal Car::getFrontWheelsCfm() {
	return dJointGetHinge2Param(ph.joints[0], dParamSuspensionCFM);
}

dReal Car::getBackWheelsCfm() {
	return dJointGetHinge2Param(ph.joints[2], dParamSuspensionCFM);
}

void Car::setBackWheelsCfm(dReal cfm) {
	dJointSetHinge2Param(ph.joints[2], dParamSuspensionCFM, cfm);
	dJointSetHinge2Param(ph.joints[3], dParamSuspensionCFM, cfm);
}

void Car::setFrontWheelsCfm(dReal cfm) {
	dJointSetHinge2Param(ph.joints[0], dParamSuspensionCFM, cfm);
	dJointSetHinge2Param(ph.joints[1], dParamSuspensionCFM, cfm);
}

void Car::createAndAttachEntity(const std::string &name,
		const std::string &meshName, const std::string &materialName,
		Ogre::SceneNode *node) const {
	Ogre::Entity *e = sceneMgr_->createEntity(name, meshName);
	e->setMaterialName(materialName);
	node->attachObject(e);
}

void Car::setMass(dReal total, dReal x, dReal y, dReal z) {
	dMassSetBoxTotal(&ph.mass, total, 3.0, 3.0, 3.0);
	std::cout << total << " " << x << " " << y << " " << z << std::endl;
}

void Car::createCamNodes(Utils::Xml &x) {
	Ogre::SceneNode *cam = cst.carNode->createChildSceneNode("cam_pos");
	cam->setPosition(0.0, 8, 20.0);

	Ogre::SceneNode *camT = cst.carNode->createChildSceneNode("cam_target");
	camT->setPosition(0.0, 1.0, -7.0);

	cam->setAutoTracking(true, camT);
	cam->setFixedYawAxis(true);
}

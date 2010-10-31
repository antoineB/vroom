#include <iostream>
#include "parsexml.hpp"
#include "world.hpp"
#include "global.hpp"
#include "world.hpp"

std::filebuf ParseXml::fb=openOrDie();
std::ostream ParseXml::os(&fb);

void ParseXml::log(std::string s){
  os<<s<<std::endl;  
}

std::filebuf& ParseXml::openOrDie(){
  fb.open ("parsexml.log",std::ios::out);
  if(!fb.is_open()){
    std::cerr<<"impossible to open the parse.log file"<<std::endl;
    exit(1);
  }
}

void ParseXml::logAndDie(std::string s){
  os<<s<<std::endl;  
  fb.close();
  exit(1);
}


bool ParseXml::getVector3(TiXmlHandle handle, dReal* array,
			 std::string marker){
  memset(array, 0, 3*sizeof(dReal));

  handle = handle.FirstChild( marker.c_str() );

  TiXmlElement *elem;  
  elem=handle.FirstChild( "x" ).ToElement();
  if( elem )
    array[0] = World::atodr( elem->GetText() );
  else {
    log("x expected in "+marker);
    return false;
  }
  
  elem = handle.FirstChild( "y" ).ToElement();
  if(elem )
    array[1] = World::atodr( elem->GetText() );
  else{
    log("y expected in "+marker);
    return false;
  }
  
  elem=handle.FirstChild( "z" ).ToElement();
  if(elem )
    array[2] = World::atodr( elem->GetText() );
  else{
    log("z expected in "+marker);
    return false;
  }
  
  return true;
}

bool ParseXml::getValue(TiXmlHandle handle, dReal* value,
			 std::string marker){
  memset(value, 0, sizeof(dReal));

  TiXmlElement *elem;  
  elem=handle.FirstChild( marker.c_str() ).ToElement();
  if( elem )
    value[0] = World::atodr( elem->GetText() );
  else{
    log(marker+" expected");
    return false;
  }

  return true;
}


bool ParseXml::getValue(TiXmlHandle handle, std::string* value,
			 std::string marker){
  *value=="";

  TiXmlElement *elem;  
  elem=handle.FirstChild( marker.c_str() ).ToElement();
  if( elem )
    *value = elem->GetText();
  else{
    log(marker+" expected");
    return false;
  }

  return true;
}


bool ParseXml::getValue(TiXmlHandle handle, int* value,
			 std::string marker){
  memset(value,0,sizeof(int));

  TiXmlElement *elem;  
  elem=handle.FirstChild( marker.c_str() ).ToElement();
  if( elem )
    *value = atoi(elem->GetText());
  else{
    log(marker+" expected");
    return false;
  }

  return true;
}




bool ParseXml::getVector4(TiXmlHandle handle, dReal* array,
			  std::string marker){
  memset(array, 0, 4*sizeof(dReal));
  
  handle = handle.FirstChild( marker.c_str() );
  
  TiXmlElement *elem;  
  elem=handle.FirstChild( "w" ).ToElement();
  if( elem )
    array[0] = World::atodr( elem->GetText() );
  else{
    log("w expected in "+marker);    
    return false;
  }
  
  return getVector3(handle, array+1, marker);
}


/*bool ParseXml::getVector12(TiXmlHandle handle, dReal* array,
			 std::string marker){
  memset(array, 0, 3*sizeof(dReal));

  handle = handle.FirstChild( marker.c_str() );

  TiXmlElement *elem;  
  elem=handle.FirstChild( "xx" ).ToElement();
  if( elem )
    array[0] = World::atodr( elem->GetText() );
  else return false;
  
  elem = handle.FirstChild( "y" ).ToElement();
  if(elem )
    array[1] = World::atodr( elem->GetText() );
  else return false;
  
  elem=handle.FirstChild( "z" ).ToElement();
  if(elem )
    array[2] = World::atodr( elem->GetText() );
  else return false;
  
  return true;
  }*/

void ParseXml::parseXml(TiXmlHandle handle, dGeomID* g, Space space){

  TiXmlElement *elem= handle.ToElement();
  std::string type(elem->Attribute("type"));

  if(type=="")
    logAndDie("pas delement type specifier");
  
  if(type=="sphere"){
    dReal r[1];
    if(!getValue(handle,r,"radius"))
      logAndDie("pas d'element radius");
    *g=space.addSphere(r[0]);
  }

  else if(type=="box"){
    dReal r[3];
    if(!getVector3(handle,r,"box"))
      logAndDie("pas d'element box");
    *g=space.addBox(r[0], r[1], r[2]);
  }

  else if(type=="plane"){
    dReal r[4];
    if(!getVector3(handle,r,"plane"))
      logAndDie("pas d'element plane");
    *g=space.addPlane(r[0], r[1], r[2], r[3]);
  }

  else if(type=="cylinder"){
        dReal r[2];
    if(!getVector3(handle,r,"radius"))
      logAndDie("pas d'element radius");
    if(!getVector3(handle,r,"length"))
      logAndDie("pas d'element length");
    *g=space.addCylinder(r[0], r[1]);
  }

  else if(type=="trimesh"){
    std::string s;
    if(!getValue(handle,&s,"tri_mesh"))
      logAndDie("pas delemeent tri_mesh");
    Ogre::Entity* e=_sceneMgr->createEntity(s);
    dTriMeshDataID data = MyTools::dTriMeshDataFromMesh(e);
    _sceneMgr->destroyEntity(e);
    *g=space.addTriMesh(data);
  }

  /*else if(type=="geom_transform"){
       
  }

  else if(type=="capsule"){
    
  }

  else if(type=="convex"){

  }

  else if(type=="hightfield"){
	   
  }*/

  else{
    logAndDie("in geom for attribute type, geom class not supported");
  }
 
  {
    dReal array[3];
    if( getVector3( handle, array, "position") )
      dGeomSetPosition( *g, array[0], array[1], array[2] );
  }

  {
    dQuaternion array;
    if( getVector4( handle, array, "quaternion" ) )
      dGeomSetQuaternion ( *g, array);
  }

  /* {
    dMatrix3 array;
    if( getVector12( handle, array, "rotation" ) )
      dGeomSetRotation( g, array );  
      }*/

}

void ParseXml::parseXml(TiXmlHandle handle, dBodyID* b, dGeomID* g, dMass* m){

  if(g==NULL || m==NULL)
    logAndDie("pas de geom ou de mass");


  *b=World::getSingletonPtr()->add(*g,m);

  {
    dReal array[3];
    if( getVector3( handle, array, "position") )
      dBodySetPosition(*b, array[0], array[1], array[2] );
  }

  {
    dQuaternion array;
    if( getVector4( handle, array, "quaternion" ) )
      dBodySetQuaternion ( *b, array);
  }

  /*  {
    dMatrix3 array;
    if( getVector12( handle, array, "rotation" ) )
      dBodySetRotation( b, array );  
      }*/

  {
    dReal array[3];
    if( getVector3( handle, array, "linear_velocity") )
      dBodySetLinearVel( *b, array[0], array[1], array[2] );
  }
 
 {
    dReal array[3];
    if( getVector3( handle, array, "angular_velocity") )
      dBodySetAngularVel( *b, array[0], array[1], array[2] );
  }

}

void ParseXml::parseXml(TiXmlHandle handle, dMass* m){

  dReal* density;
  if(!getValue(handle,density,"density")){
    logAndDie("fuck u mother fucker");
  }

  TiXmlElement *elem= handle.ToElement();
  std::string type(elem->Attribute("type"));
  if(type==""){
    logAndDie("attribute was missing");
  }

  if(type=="sphere"){
    dReal* radius;
    if(!getValue(handle,radius,"radius")){
      logAndDie("fuck u mother fucker");
    }

    dMassSetSphere(m, density[0], radius[0]);
  }

  else if(type=="box"){
    dReal r[3];
    if(!getVector3(handle,r,"box"))
      logAndDie("son of bitch");
    dMassSetBox (m, *density, r[0], r[1], r[2]);
  }

  else if(type=="trimesh"){
    //void dMassSetTrimesh (dMass *, dReal density, dGeomID g)
    //void dMassTranslate (dMass *, dReal x, dReal y, dReal z);
    logAndDie("can support the trimesh mass rigth now");
  }

  else if(type=="plane"){
    logAndDie("wtf, a plane don't have any mass");
  }

  else if(type=="cylinder"){

    int *di;
    dReal r[2];
    if(!getValue(handle,di,"direction"))
      logAndDie("son of bitch");

    if(!getValue(handle,r,"radius"))
      logAndDie("son of bitch");

    if(!getValue(handle,r+1,"length"))
      logAndDie("son of bitch");

    dMassSetCylinder (m, *density, *di, r[0], r[1]);

  }

  /*else if(type=="capsule"){

  }

  else if(type=="convex"){

  }

  else if(type=="geom_transform"){
       
  }

  else if(type=="hightfield"){
	   
  

  }*/

  else
    logAndDie("in geom for attribute type, geom class not supported");
    
}

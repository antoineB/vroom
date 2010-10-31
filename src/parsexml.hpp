#ifndef PARSEXML_HPP
#define PARSEXML_HPP

#include <tinyxml.h>
#include <ode/ode.h>


#include <ostream>
#include <fstream>

#include "space.hpp"

class ParseXml{
  
  static std::filebuf fb;
  static std::ostream os;

  static std::filebuf& openOrDie();

public:
    
  //static void parseXml(std::string fileName, dJointID* j, unsigned int nb=0);
  static void parseXml(TiXmlHandle handle, dJointID* j, unsigned int nb=0);

  //static void parseXml(std::string fileName, dGeomID* g, unsigned int nb=0);
  static void parseXml(TiXmlHandle handle, dGeomID* g, Space space);
  
  //static void parseXml(std::string fileName, dBodyID* b, unsigned int nb=0);
  static void parseXml(TiXmlHandle handle, dBodyID* b, dGeomID*g, dMass* m);
  
  //static void parseXml(std::string fileName, dMass* m, unsigned int nb=0);
  static void parseXml(TiXmlHandle handle, dMass* m);

  static bool getVector3(TiXmlHandle handle, dReal* array,
				   std::string marker);
  static bool getVector4(TiXmlHandle handle, dReal* array,
				   std::string marker);
  //  static bool getVector12(TiXmlHandle handle, dReal* array,
  //				   string marker);

  static bool getValue(TiXmlHandle handle, dReal* value, std::string marker); 
  static bool getValue(TiXmlHandle handle, std::string* value, std::string marker);
  static bool getValue(TiXmlHandle handle, int* value, std::string marker);


  static void log(std::string s);
  static void logAndDie(std::string s);

};

#endif

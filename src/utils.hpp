#ifndef UTILS_HPP
#define UTILS_HPP

#include "global.hpp"
#include "type.hpp"

#include "tinyxml.h"

namespace Utils {

class Xml {
private:
  TiXmlDocument *documantHandle;
  TiXmlElement* docElem;
  bool mustQuit;
  
  Xml(const Xml&);

  void init(std::string &fileName, const char* root);
  void end();
  
public:

  Xml(const char* fileName, const char* root);
  Xml(std::string &fileName, const char* root);
  ~Xml();

  TiXmlElement* mustNode(std::string &name, int childPos = 0, TiXmlElement* xmlE = NULL);
  std::string mustString(std::string &name, int childPos, TiXmlElement* xmlE = NULL);
  float mustFloat(std::string &name, int childPos, TiXmlElement* xmlE = NULL);
  double mustDouble(std::string &name, int childPos, TiXmlElement* xmlE = NULL);
  int mustInt(std::string &name, int childPos, TiXmlElement* xmlE = NULL);
  long mustLong(std::string &name, int childPos, TiXmlElement* xmlE = NULL);
  Ogre::Real mustOReal(std::string &name, int childPos, TiXmlElement* xmlE);

  TiXmlElement* mustNode(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);
  std::string mustString(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);
  float mustFloat(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);
  double mustDouble(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);
  int mustInt(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);
  long mustLong(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);
  Ogre::Real mustOReal(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);



  //ATTRIBUTES
  std::string mustStringA(std::string &name, TiXmlElement* xmlE);
  float mustFloatA(std::string &name, TiXmlElement* xmlE);
  double mustDoubleA(std::string &name, TiXmlElement* xmlE);
  int mustIntA(std::string &name, TiXmlElement* xmlE);
  long mustLongA(std::string &name, TiXmlElement* xmlE);
  Ogre::Real mustORealA(std::string &name, TiXmlElement* xmlE);

  std::string mustStringA(const char *name, TiXmlElement* xmlE);
  float mustFloatA(const char *name, TiXmlElement* xmlE);
  double mustDoubleA(const char *name, TiXmlElement* xmlE);
  int mustIntA(const char *name, TiXmlElement* xmlE);
  long mustLongA(const char *name, TiXmlElement* xmlE);
  Ogre::Real mustORealA(const char *name, TiXmlElement* xmlE);


  void fillDContact(const char* contactNodeName, DContactType &ctc);

};

}


#endif

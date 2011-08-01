#include "utils.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <assert.h>

#include <OGRE/OgreStringConverter.h>

Utils::Xml::Xml(const char *fileName, const char *root) {
  std::string r(fileName);
  Utils::Xml::init(r, root);
}

Utils::Xml::Xml(std::string &fileName, const char *root) {
  Utils::Xml::init(fileName, root);
}

void Utils::Xml::init(std::string &fileName, const char *root) {
  documantHandle = NULL;
  docElem = NULL; 
  mustQuit = false;
  
  documantHandle = new TiXmlDocument(fileName);
  
  if(!documantHandle->LoadFile()) {
    log_("IO error in xml file: " + fileName);
    mustQuit = true;
    end();
  }
  
    docElem = documantHandle->FirstChildElement(root);

  if (!docElem) {
    log_("not found " + fileName + " with " + root + " as root node");
    mustQuit = true;
    end();
    }
}

void Utils::Xml::end() {
  assert(documantHandle != NULL);

  delete documantHandle;
  documantHandle = NULL;

  if (mustQuit) 
    exit(-1);
}

Utils::Xml::~Xml() {
  end();
}

TiXmlElement* Utils::Xml::mustNode(const char* name, int childPos, TiXmlElement* xmlE) {
  std::string s(name);
  return mustNode(s, childPos, xmlE);
}

std::string Utils::Xml::mustString(const char* name, int childPos, TiXmlElement* xmlE) {
  std::string s(name);
  return mustString(s, childPos, xmlE);
}

float Utils::Xml::mustFloat(const char* name, int childPos, TiXmlElement* xmlE) {
  std::string s(name);
  return mustFloat(s, childPos, xmlE);
}

float Utils::Xml::mustOReal(const char* name, int childPos, TiXmlElement* xmlE) {
  std::string s(name);
  return mustOReal(s, childPos, xmlE);
}

double Utils::Xml::mustDouble(const char* name, int childPos, TiXmlElement* xmlE) {
  std::string s(name);
  return mustDouble(s, childPos, xmlE);
}

int Utils::Xml::mustInt(const char* name, int childPos, TiXmlElement* xmlE) {
  std::string s(name);
  return mustInt(s, childPos, xmlE);
}

long Utils::Xml::mustLong(const char* name, int childPos, TiXmlElement* xmlE) {
  std::string s(name);
  return mustLong(s, childPos, xmlE);
}


TiXmlElement* Utils::Xml::mustNode(std::string &name, int childPos, TiXmlElement* xmlE) {
  assert(documantHandle != NULL);

  std::vector<std::string> vec;
  
  boost::split(vec, name, boost::is_any_of("."));
  
  TiXmlElement* elem = xmlE ? xmlE : docElem;
  
  for (std::vector<std::string>::iterator it = vec.begin(); it < vec.end(); ++it) {
     elem = elem->FirstChildElement(*it);
     if (!elem) {
       log_("impossible to find " + *it + " from " + name);
       mustQuit = true;
       end();
     }
   }

  for (int i = 0; i < childPos; ++i) {
    elem = elem->NextSiblingElement();
    if (!elem) {
      log_("impossible to find a " + name);
      mustQuit = true;
      end();
    }
  }

  return elem;
}

std::string Utils::Xml::mustString(std::string &name, int childPos, TiXmlElement* xmlE) {
  return mustNode(name, childPos, xmlE)->GetText();
}

float Utils::Xml::mustFloat(std::string &name, int childPos, TiXmlElement* xmlE) {
  float f;

  try {
    f = boost::lexical_cast<float>(mustNode(name, childPos, xmlE)->GetText());
  }
  catch (boost::bad_lexical_cast &) {
    mustQuit = true;
    log_("unable to convert to float " + name);
  }

  return f;
}

Ogre::Real Utils::Xml::mustOReal(std::string &name, int childPos, TiXmlElement* xmlE) {
  return Ogre::StringConverter::parseReal(mustNode(name, childPos, xmlE)->GetText());
}

double Utils::Xml::mustDouble(std::string &name, int childPos, TiXmlElement* xmlE) {
  double f;

  try {
    f = boost::lexical_cast<double>(mustNode(name, childPos, xmlE)->GetText());
  }
  catch (boost::bad_lexical_cast &) {
    mustQuit = true;
    log_("unable to convert to double " + name);
  }

  return f;
}
 
int Utils::Xml::mustInt(std::string &name, int childPos, TiXmlElement* xmlE) {
  int f;

  try {
    f = boost::lexical_cast<int>(mustNode(name, childPos, xmlE)->GetText());
  }
  catch (boost::bad_lexical_cast &) {
    mustQuit = true;
    log_("unable to convert to int " + name);
  }

  return f;
}

long Utils::Xml::mustLong(std::string &name, int childPos, TiXmlElement* xmlE) {
  long f;

  try {
    f = boost::lexical_cast<long>(mustNode(name, childPos, xmlE)->GetText());
  }
  catch (boost::bad_lexical_cast &) {
    mustQuit = true;
    log_("unable to convert to long " + name);
  }

  return f;
}


//ATRIBUTES

std::string Utils::Xml::mustStringA(std::string &name, TiXmlElement* xmlE) {
  return xmlE->Attribute(name.c_str());
}

float Utils::Xml::mustFloatA(std::string &name, TiXmlElement* xmlE) {
  float f;

  try {
    f = boost::lexical_cast<float>(xmlE->Attribute(name));
  }
  catch (boost::bad_lexical_cast &) {
    mustQuit = true;
    log_("unable to convert to float " + name);
  }

  return f;
}

double Utils::Xml::mustDoubleA(std::string &name, TiXmlElement* xmlE) {
  double f;

  try {
    f = boost::lexical_cast<double>(xmlE->Attribute(name));
  }
  catch (boost::bad_lexical_cast &) {
    mustQuit = true;
    log_("unable to convert to double " + name);
  }

  return f;
}
 
int Utils::Xml::mustIntA(std::string &name, TiXmlElement* xmlE) {
  int f;

  try {
    f = boost::lexical_cast<int>(xmlE->Attribute(name));
  }
  catch (boost::bad_lexical_cast &) {
    mustQuit = true;
    log_("unable to convert to int " + name);
  }

  return f;
}

long Utils::Xml::mustLongA(std::string &name, TiXmlElement* xmlE) {
  long f;

  try {
    f = boost::lexical_cast<long>(xmlE->Attribute(name));
  }
  catch (boost::bad_lexical_cast &) {
    mustQuit = true;
    log_("unable to convert to long " + name);
  }

  return f;
}

Ogre::Real Utils::Xml::mustORealA(std::string &name, TiXmlElement* xmlE) {
  if (xmlE->Attribute(name) == NULL)
    log_("unable to convert to long " + name);
  std::string s = (xmlE->Attribute(name))->c_str();
  return Ogre::StringConverter::parseReal(s);
}


std::string Utils::Xml::mustStringA(const char *name, TiXmlElement* xmlE) {
  return xmlE->Attribute(name);
}

float Utils::Xml::mustFloatA(const char *name, TiXmlElement* xmlE) {
  std::string s(name);
  return Utils::Xml::mustFloatA(s, xmlE);
}

double Utils::Xml::mustDoubleA(const char *name, TiXmlElement* xmlE) {
  std::string s(name);
  return Utils::Xml::mustDoubleA(s, xmlE);
}
 
int Utils::Xml::mustIntA(const char *name, TiXmlElement* xmlE) {  
  std::string s(name);
  return Utils::Xml::mustIntA(s, xmlE);
}

long Utils::Xml::mustLongA(const char *name, TiXmlElement* xmlE) {
  std::string s(name);
  return Utils::Xml::mustLongA(s, xmlE);
}

Ogre::Real Utils::Xml::mustORealA(const char *name, TiXmlElement* xmlE) {
  std::string s(name);
  return Utils::Xml::mustORealA(s, xmlE);
}


#include "utils.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <assert.h>

namespace Utils {
  namespace Xml {
    TiXmlDocument *documantHandle = NULL;
    TiXmlElement* docElem = NULL;
    bool mustQuit;
  };
};

void Utils::Xml::begin(std::string &fileName, const char *root) {
  assert(documantHandle == NULL);
 
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

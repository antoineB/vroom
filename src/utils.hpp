#ifndef UTILS_HPP
#define UTILS_HPP

#include "global.hpp"

#include "tinyxml.h"

namespace Utils {

namespace Xml {
  void begin(std::string &fileName, const char* root);
  void end();

  TiXmlElement* mustNode(std::string &name, int childPos = 0, TiXmlElement* xmlE = NULL);
  std::string mustString(std::string &name, int childPos, TiXmlElement* xmlE = NULL);
  float mustFloat(std::string &name, int childPos, TiXmlElement* xmlE = NULL);
  double mustDouble(std::string &name, int childPos, TiXmlElement* xmlE = NULL);
  int mustInt(std::string &name, int childPos, TiXmlElement* xmlE = NULL);
  long mustLong(std::string &name, int childPos, TiXmlElement* xmlE = NULL);

  TiXmlElement* mustNode(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);
  std::string mustString(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);
  float mustFloat(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);
  double mustDouble(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);
  int mustInt(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);
  long mustLong(const char* name, int childPos = 0, TiXmlElement* xmlE = NULL);

};

};


#endif

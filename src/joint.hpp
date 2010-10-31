#ifndef JOINT_HPP
#define JOINT_HPP

class Joint{
  //  dJointID j;

public:
  static enum Type{ //static necessaire?
    HINGE2=0, HINGE, SOCKET,
  };

  //inline dJointID getJoint();
  
  static void parseXml(dJointId j, std::string fileName);
  static void parseXml(dJointId j, TiXmlHandler handle);
  
};

#endif

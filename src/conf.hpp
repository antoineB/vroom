#ifndef CONF_HPP
#define CONF_HPP

#include <ode/ode.h>

namespace Conf {

  namespace Math {
    const dReal PI = 3.14159265;
  };




  namespace Car {
    const dReal POS[3] = { 0, 4., 0 };
    const dReal POSOFFSET[3] = { 0.0, 0.3, 0.0 };
    const dReal BOX[3] = { 7.48, 0.6, 17.56 };

    const dReal AXIS1[4][3] = {
      {0, 1, 0},
      {0, 1, 0},
      {0, 1, 0},
      {0, 1, 0}
    };
    
    const dReal AXIS2[4][3] = {
      {1, 0, 0},
      {1, 0, 0},
      {1, 0, 0},
      {1, 0, 0}
    };

    namespace Door {

      const dReal BOX_BOUND[3] = { 1.183 * 0.35 * 2.0, 
				   7.61 * 0.35 * 2.0, 
				   5.128 * 0.35 * 2.0 };

      const dReal DENSITY = 1; 
      const dReal BOX_MASS[3] = { 1.0, 1.5, 2.0 };

      namespace Left {
	const dReal POS[3] = { -9.513 * 0.35, 
			       (-7.612 * 0.35) +2.2, 
			       -0.404 * 0.35 };

	const dReal JOINT_POS[3] = { -9.513 * 0.35, 
				     -7.612 * 0.35, 
				     -0.404 * 0.35 };

	const dReal JOINT_AXIS[3] = { 0, 1, 0 };
	
      };
      
      namespace Right {
	const dReal POS[3] = { +9.513 * 0.35,
			       (-7.612 * 0.35) +2.2, 
			       -0.404 * 0.35 };

	const dReal JOINT_POS[3] = { 9.513 * 0.35, 
				     -7.612 * 0.35, 
				     -0.404 * 0.35 };

	const dReal JOINT_AXIS[3] = { 0, 1, 0 };

      };
    };
  };

  namespace Wheel {
    //the position of the wheel went car is in {O, O, O}
    const dReal POS[4][3] = {
      { 3.05, +0.0, -5.85 },
      { -3.05, +0.0, -5.85 },
      { 3.15, -0.2, 5.0 },
      { -3.15, -0.2, 5.0 }
    };
    
    const dReal DENSITY = 1;
    const dReal RADIUS = 1.20;
    const dReal WIDTH = 0.5;

    const dReal SCALE[3] = { 2.45, 2.45, 2.45 };

  };

};

#endif

#ifndef CONF_HPP
#define CONF_HPP

#include <ode/ode.h>

namespace Conf {

  namespace Car {


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

};

#endif

#ifndef SaiModel_RBDL_EXTENSIONS_H_
#define SaiModel_RBDL_EXTENSIONS_H_

#include <assert.h>

#include <iostream>

#include "rbdl/rbdl.h"

using namespace RigidBodyDynamics;

namespace SaiModel {

/**
 * @brief A function to extend RBDL inverse kinematic function to include joint
 * limits handling. The joint limits are handled by saturating the intermediate
 * solutions to the joint limits, and using the increment of the solution as an
 * additional convergence criterion.
 *
 * @param model The rbdl model of the robot
 * @param Qinit The initial guess for the joint positions
 * @param CS The inverse kinematics constraint set
 * @param QMin The lower joint limits
 * @param QMax The upper joint limits
 * @param Qres The resulting joint positions
 * @return RBDL_DLLAPI
 */
RBDL_DLLAPI bool InverseKinematicsWithJointLimits(
	Model &model, const Math::VectorNd &Qinit,
	InverseKinematicsConstraintSet &CS, Math::VectorNd &QMin,
	Math::VectorNd &QMax, Math::VectorNd &Qres);
} /* namespace SaiModel */

#endif /* SaiModel_RBDL_EXTENSIONS_H_ */
#ifndef SAI2MODEL_RBDL_EXTENSIONS_H_
#define SAI2MODEL_RBDL_EXTENSIONS_H_

#include <assert.h>

#include <iostream>

#include "rbdl/rbdl.h"

using namespace RigidBodyDynamics;

namespace Sai2Model {

RBDL_DLLAPI bool InverseKinematicsWithJointLimits(
	Model &model, const Math::VectorNd &Qinit,
	InverseKinematicsConstraintSet &CS, Math::VectorNd &QMin,
	Math::VectorNd &QMax, Math::VectorNd &Qres);
} /* namespace Sai2Model */

#endif /* SAI2MODEL_RBDL_EXTENSIONS_H_ */
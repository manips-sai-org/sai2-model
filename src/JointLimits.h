#ifndef SAI2MODEL_JOINT_LIMITS_H_
#define SAI2MODEL_JOINT_LIMITS_H_

#include <string>

namespace Sai2Model {

/**
 * @brief A structure to store joint limits for a robot
 * 
 */
struct JointLimit {
	/// @brief Name of the joint
	std::string joint_name = "";
	/// @brief Index of the joint in the kinematic chain
	int joint_index = -1;
	/// @brief Lower limit of the joint position
	double position_lower = 0.0;
	/// @brief Upper limit of the joint position
	double position_upper = 0.0;
	/// @brief Maximum velocity of the joint
	double velocity = 0.0;
	/// @brief Maximum effort of the joint
	double effort = 0.0;

	JointLimit(std::string joint_name, int index, double lower, double upper,
			   double vel, double f)
		: joint_name(joint_name),
		  joint_index(index),
		  position_lower(lower),
		  position_upper(upper),
		  velocity(vel),
		  effort(f) {}
};

}  // namespace Sai2Model

#endif	// SAI2MODEL_JOINT_LIMITS_H_
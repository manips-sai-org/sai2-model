#ifndef SAI2MODEL_JOINT_LIMITS_H_
#define SAI2MODEL_JOINT_LIMITS_H_

namespace Sai2Model {

struct JointLimit {
	std::string joint_name = "";
	int joint_index = -1;
	double position_lower = 0.0;
	double position_upper = 0.0;
	double velocity = 0.0;
	double effort = 0.0;

	JointLimit(std::string joint_name, int index, double lower, double upper, double vel, double f)
		: joint_index(index),
		  position_lower(lower),
		  position_upper(upper),
		  velocity(vel),
		  effort(f) {}
};

}  // namespace Sai2Model

#endif	// SAI2MODEL_JOINT_LIMITS_H_
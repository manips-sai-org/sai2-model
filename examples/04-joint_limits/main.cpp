// 04-joint linits
#include <Sai2Model.h>

#include <iostream>
#include <memory>

using namespace std;

const string robot_fname =
	string(EXAMPLES_FOLDER) + "/04-joint_limits/rpspbot.urdf";

int main(int argc, char** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_fname);

	cout << endl
		 << "Joint limits are parsed automatically in the urdf file."
		 << endl
		 << endl;

	for (const auto& limit : robot->jointLimits()) {
		cout << "Joint name: " << limit.joint_name
			 << " - joint index: " << limit.joint_index
			 << " - joint name: " << robot->jointName(limit.joint_index)
			 << " - lower limit: " << limit.position_lower
			 << " - upper limit: " << limit.position_upper
			 << " - velocity: " << limit.velocity << " effort: " << limit.effort
			 << endl;
	}
	cout << endl;

	return 0;
}
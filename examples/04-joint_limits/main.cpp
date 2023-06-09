#include <Sai2Model.h>

#include <iostream>
#include <memory>

using namespace std;

const string robot_fname = "resources/rpspbot.urdf";

int main(int argc, char** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_fname, false);

	cout << endl;
	for (const auto& limit : robot->joint_limits()) {
		cout << "Joint name: " << limit.joint_name
		     << " - joint index: " << limit.joint_index
			 << " - joint name: " << robot->joint_name(limit.joint_index)
			 << " - lower limit: " << limit.position_lower
			 << " - upper limit: " << limit.position_upper
			 << " - velocity: " << limit.velocity << " effort: " << limit.effort
			 << endl;
	}
	cout << endl;

	return 0;
}
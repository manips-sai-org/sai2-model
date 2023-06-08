#include <Sai2Model.h>

#include <iostream>
#include <memory>

using namespace std;

const string robot_fname = "resources/rpspbot.urdf";

int main(int argc, char** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_fname, false);
	int dof = robot->dof();

	robot->displayLinks();
	robot->displayJoints();

	cout << "q size: " << robot->q_size() << endl;
	cout << "dof size: " << robot->dof() << endl;
	cout << "q: " << robot->q().transpose() << endl;
	cout << "dq: " << robot->dq().transpose() << endl;

	for (const auto& limit : robot->joint_limits()) {
		cout << "Joint index: " << limit.joint_index
			 << " joint name: " << robot->joint_name(limit.joint_index)
			 << " lower limit: " << limit.position_lower
			 << " upper limit: " << limit.position_upper
			 << " velocity: " << limit.velocity << " effort: " << limit.effort
			 << endl;
	}
	cout << endl;

	cout << "joint names from Sai2Model call" << std::endl;
	for(int i=0 ; i<robot->q_size() ; ++i) {
		cout << robot->joint_name(i) << '\t';
	}
	cout << endl;

	vector<string> joint_names {"j0", "j1", "j2", "j3"};
	cout << "joint id per name:" << endl;
	for (const auto& joint_name : joint_names ) {
		cout << joint_name << " " << robot->joint_index(joint_name) << endl; 
	}
	cout << endl << endl;

	cout << "spherical joint w index: " << robot->spherical_joint_w_index("j2") << endl;

	return 0;
}
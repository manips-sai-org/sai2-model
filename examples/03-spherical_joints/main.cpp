#include <Sai2Model.h>

#include <iostream>
#include <memory>

using namespace std;

const string robot_fname = "resources/rpspsbot.urdf";

int main(int argc, char** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_fname, false);

	robot->displayLinks();
	robot->displayJoints();

	cout << "spherical joints in robot model:" << endl;
	for(const auto& joint_description : robot->spherical_joints()) {
		cout << "name: " << joint_description.name
			 << " - index: " << joint_description.index
			 << " - w_index: " << joint_description.w_index << endl;
	}
	cout << endl;

	cout << "q size: " << robot->q_size() << endl;
	cout << "dof size: " << robot->dof() << endl;
	cout << "q: " << robot->q().transpose() << endl;
	cout << "dq: " << robot->dq().transpose() << endl;
	cout << endl;

	cout << "joint names from Sai2Model call" << std::endl;
	for(const auto& name : robot->joint_names()) {
		std::cout << name << " , ";
	}
	std::cout << std::endl;

	cout << "joint names from Sai2Model individual call" << std::endl;
	for(int i=0 ; i<robot->q_size() ; ++i) {
		cout << "joint: " << i << " - name: " << robot->joint_name(i) << endl;
	}
	cout << endl;

	cout << "joint id per name:" << endl;
	for (const auto& joint_name : robot->joint_names() ) {
		cout << joint_name << ": " << robot->joint_index(joint_name) << endl; 
	}
	cout << endl << endl;

	cout << "spherical joint w indexes from individual call: " << endl;
	for(const auto& sph_joint : robot->spherical_joints()) {
		cout << "name: " << sph_joint.name
			 << " -  w_index: " << robot->spherical_joint_w_index(sph_joint.name)
			 << endl;
	}
	cout << endl;

	return 0;
}
// 01-create_model_from_file: example of how to create a robot model from a urdf
// file

#include <iostream>
#include <Sai2Model.h>

using namespace std;

const string robot_fname = "resources/pbot.urdf";

int main(int argc, char** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	Sai2Model::Sai2Model robot(robot_fname, true);

	cout << endl << endl;
	cout << "robot degrees of freedom : " << robot.dof() << endl;
	cout << "robot coordinates : " << robot.q().transpose() << endl;

	// get mass properties for link0
	double mass;
	Eigen::Vector3d com;
	Eigen::Matrix3d inertia;
	robot.getLinkMass(mass, com, inertia, "link0");
	cout << "link 0 mass properties: " << endl;
	cout << "mass: " << mass << endl;
	cout << "center of mass in link: " << com.transpose() << endl;
	cout << "inertia tensor:\n" << inertia << endl;

	return 0;
}
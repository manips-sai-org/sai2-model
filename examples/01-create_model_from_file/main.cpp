// 01-create_model_from_file: example of how to create a robot model from a urdf
// file

#include <SaiModel.h>

#include <iostream>

using namespace std;

const string robot_fname =
	string(EXAMPLES_FOLDER) + "/01-create_model_from_file/pbot.urdf";

int main(int argc, char** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	SaiModel::SaiModel robot(robot_fname, true);

	cout << endl << endl;
	cout << "robot degrees of freedom : " << robot.dof() << endl;
	cout << "robot coordinates : " << robot.q().transpose() << endl;

	// get mass properties for link0
	double mass;
	Eigen::Vector3d com;
	Eigen::Matrix3d inertia;
	SaiModel::LinkMassParams mass_params = robot.getLinkMassParams("link0");
	cout << "link 0 mass properties: " << endl;
	cout << "mass: " << mass_params.mass << endl;
	cout << "center of mass in link: " << mass_params.com_pos.transpose()
		 << endl;
	cout << "inertia tensor:\n" << mass_params.inertia << endl;

	return 0;
}
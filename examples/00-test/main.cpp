// 02-update_model: 
// example of how to use the different kinematic and dynamic functions

#include <iostream>
#include <Sai2Model.h>

using namespace std;

const string robot_fname = "resources/rrbot.urdf";

int main (int argc, char ** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	// Sai2Model::Sai2Model* robot = new Sai2Model::Sai2Model(robot_fname, false);
	Sai2Model::Sai2Model* robot = new Sai2Model::Sai2Model(robot_fname, false, Eigen::Vector3d(1.0,0.0,0.0));
	int dof = robot->dof();

	const string ee_link = "link1";
	const Eigen::Vector3d ee_pos_in_link = Eigen::Vector3d(0.0, 0.0, 1.0);

	// cout << robot->_rbdl_model->gravity.transpose() << endl;

	Eigen::VectorXd br = Eigen::VectorXd::Zero(2);
	Eigen::VectorXd bc = Eigen::VectorXd::Zero(2);

	robot->_q << 45.0/180.0*M_PI, 45.0/180.0*M_PI;
	// std::cout << "q : " << robot->_q.transpose() << "\n\n";
	robot->_dq << 1.7, 0.2;
	// robot->_ddq << 0.02, 0.55;
	robot->updateModel();

	robot->coriolisForce(br);
	robot->modifiedNewtonEuler(bc, robot->_q, robot->_dq, robot->_dq, Eigen::VectorXd::Zero(2));

	cout << "\ncoriolis by rbdl :\n" << br.transpose() << endl << endl;
	cout << "\ncoriolis custom :\n" << bc.transpose() << endl << endl;


	return 0;
}
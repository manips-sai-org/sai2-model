// 02-update_model: 
// example of how to use the different kinematic and dynamic functions

#include <iostream>
#include <Sai2Model.h>

using namespace std;

const string robot_fname = "resources/rrbot.urdf";

int main (int argc, char ** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	Eigen::Affine3d T_world_robot = Eigen::Affine3d::Identity();
	// Sai2Model::Sai2Model* robot = new Sai2Model::Sai2Model(robot_fname, false);
	Sai2Model::Sai2Model* robot = new Sai2Model::Sai2Model(robot_fname, false, T_world_robot , Eigen::Vector3d(0.0,3.0,0.0));
	int dof = robot->dof();

	const string ee_link = "link1";
	const Eigen::Vector3d ee_pos_in_link = Eigen::Vector3d(0.0, 0.0, 1.0);

	// cout << robot->_rbdl_model->gravity.transpose() << endl;

	Eigen::VectorXd br = Eigen::VectorXd::Zero(dof);
	Eigen::VectorXd bc = Eigen::VectorXd::Zero(dof);

	robot->_q << 0.0/180.0*M_PI, 45.0/180.0*M_PI, -25.0/180.0*M_PI;
	// std::cout << "q : " << robot->_q.transpose() << "\n\n";
	robot->_dq << 0.2, -1.4, 0.7;
	// robot->_ddq << 0.02, 0.55;
	robot->updateModel();

	Eigen::VectorXd gr = Eigen::VectorXd::Zero(dof);
	Eigen::VectorXd gc = Eigen::VectorXd::Zero(dof);
	robot->gravityVector(gr);
	robot->modifiedNewtonEuler(gc, true, robot->_q, Eigen::VectorXd::Zero(dof), Eigen::VectorXd::Zero(dof), Eigen::VectorXd::Zero(dof));


	robot->coriolisForce(br);
	robot->modifiedNewtonEuler(bc, false, robot->_q, robot->_dq, robot->_dq, Eigen::VectorXd::Zero(dof));

	Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dof,dof);
	robot->factorizedChristoffelMatrix(C);

	cout << "C matrix :\n" << C << endl;
	cout <<"\ncorresponding coriolis :\n" << (C*robot->_dq).transpose() << endl;
	cout << "\ncoriolis by rbdl :\n" << br.transpose() << endl;
	cout << "\ncoriolis custom :\n" << bc.transpose() << endl;

	cout << "\ngravity rbdl :\n" << gr.transpose() << endl;
	cout << "\ngravity custom :\n" << gc.transpose() << endl;


	// Eigen::MatrixXd T = Eigen::MatrixXd::Zero(4,4);
	// Eigen::VectorXd u = Eigen::VectorXd::Zero(4);

	// u << 1,2,3,4;
	// T.col(0) = u;
	// u.setZero();
	// u << 4,3,2,1;
	// cout << T << endl;


	return 0;
}
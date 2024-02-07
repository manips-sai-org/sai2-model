// 02-update_model:
// example of how to use the different kinematic and dynamic functions to get
// Jacobians, the mass matrix, task space inertia and dynamically consistent
// inverse of the jacobian

#include <Sai2Model.h>

#include <iostream>

using namespace std;

const string robot_fname = "resources/rprbot.urdf";
const string humanoid_fname = "resources/toro.urdf";

int main(int argc, char** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	Sai2Model::Sai2Model* robot = new Sai2Model::Sai2Model(robot_fname);
	int dof = robot->dof();

	const string ee_link = "link2";
	const Eigen::Vector3d ee_pos_in_link = Eigen::Vector3d(0.0, 0.0, 1.0);

	// Eigen::Vector3d position;
	// Eigen::Vector3d velocity;
	// Eigen::Matrix3d rotation;
	// Eigen::MatrixXd J(6, dof);
	// Eigen::VectorXd gravity(dof);

	// position and orientation of the end effector
	Eigen::Vector3d position = robot->position(ee_link, ee_pos_in_link);
	Eigen::Vector3d velocity = robot->linearVelocity(ee_link, ee_pos_in_link);
	Eigen::Matrix3d rotation = robot->rotation(ee_link);
	// jacobian at the end effector (1m from second joint)
	Eigen::MatrixXd J = robot->J(ee_link, ee_pos_in_link);
	// gravity and coriolis/centrifugal forces
	Eigen::VectorXd gravity = robot->jointGravityVector();

	cout << "------------------------------------------------------" << endl;
	cout << "               Initial configuration                  " << endl;
	cout << "------------------------------------------------------" << endl;
	cout << endl;
	cout << "robot coordinates : " << robot->q().transpose() << endl;
	cout << "position at end effector : " << position.transpose() << endl;
	cout << "velocity at end effector : " << velocity.transpose() << endl;
	cout << "orientation at end effector : \n" << rotation << endl;
	cout << "jacobian at the end effector \n" << J << endl;
	cout << "Mass matrix :\n" << robot->M() << endl;
	cout << "joint gravity : " << gravity.transpose() << endl;
	cout << endl;

	// modify joint positions and velocities
	Eigen::VectorXd new_q = Eigen::VectorXd::Zero(robot->qSize());
	Eigen::VectorXd new_dq = Eigen::VectorXd::Zero(robot->dof());
	new_q << M_PI / 2, 1, M_PI / 2;
	new_dq << 0, 1, M_PI / 12;
	robot->setQ(new_q);
	robot->setDq(new_dq);
	position = robot->position(ee_link, ee_pos_in_link);
	velocity = robot->linearVelocity(ee_link, ee_pos_in_link);
	rotation = robot->rotation(ee_link);
	J = robot->J(ee_link, ee_pos_in_link);
	gravity = robot->jointGravityVector();

	cout << endl;
	cout << "------------------------------------------------------" << endl;
	cout << "  we modify the joint positions to 90 degrees and 1m  " << endl;
	cout << " nothing will change before we call updateKinematics()" << endl;
	cout << "------------------------------------------------------" << endl;
	cout << endl;
	cout << "robot coordinates : " << robot->q().transpose() << endl;
	cout << "position at end effector : " << position.transpose() << endl;
	cout << "velocity at end effector : " << velocity.transpose() << endl;
	cout << "orientation at end effector : \n" << rotation << endl;
	cout << "Mass matrix :\n" << robot->M() << endl;
	cout << "jacobian at the end effector \n" << J << endl;
	cout << "joint gravity : " << gravity.transpose() << endl;
	cout << endl;

	// update kinematics
	robot->updateKinematics();
	position = robot->position(ee_link, ee_pos_in_link);
	velocity = robot->linearVelocity(ee_link, ee_pos_in_link);
	rotation = robot->rotation(ee_link);
	J = robot->J(ee_link, ee_pos_in_link);
	gravity = robot->jointGravityVector();

	cout << endl;
	cout << "------------------------------------------------------" << endl;
	cout << "               call to updateKinematics()             " << endl;
	cout << "      Everything updated except the mass matrix       " << endl;
	cout << "------------------------------------------------------" << endl;
	cout << endl;
	cout << "robot coordinates : " << robot->q().transpose() << endl;
	cout << "position at end effector : " << position.transpose() << endl;
	cout << "velocity at end effector : " << velocity.transpose() << endl;
	cout << "orientation at end effector : \n" << rotation << endl;
	cout << "Mass matrix :\n" << robot->M() << endl;
	cout << "jacobian at the end effector \n" << J << endl;
	cout << "joint gravity : " << gravity.transpose() << endl;
	cout << endl;

	// update model to update mass matrix as well
	robot->updateModel();

	cout << endl;
	cout << "------------------------------------------------------" << endl;
	cout << "               call to updateModel()                  " << endl;
	cout << "   The mass matrix (and its inverse) is recomputed    " << endl;
	cout << "------------------------------------------------------" << endl;
	cout << endl;
	cout << "robot coordinates : " << robot->q().transpose() << endl;
	cout << "Mass matrix :\n" << robot->M() << endl;
	cout << endl;

	// operational space matrices
	Eigen::MatrixXd J_task = J.row(2);
	Sai2Model::OpSpaceMatrices op_space_matrices =
		robot->operationalSpaceMatrices(J_task);
	cout << endl;
	cout << "------------------------------------------------------" << endl;
	cout << "   operational space matrices in this configuration   " << endl;
	cout << "------------------------------------------------------" << endl;
	cout << endl;
	cout << "robot coordinates : " << robot->q().transpose() << endl;
	cout << "position at end effector : " << position.transpose() << endl;
	cout << "operational space inertia :\n" << op_space_matrices.Lambda << endl;
	cout << "dynamically consistent inverse of the jacobian \n"
		 << op_space_matrices.Jbar << endl;
	cout << "nullspace matrix \n" << op_space_matrices.N << endl;
	cout << endl;

	// come back to initial position
	new_q.setZero();
	new_dq.setZero();
	robot->setQ(new_q);
	robot->setDq(new_dq);
	robot->updateModel();

	position = robot->position(ee_link, ee_pos_in_link);
	velocity = robot->linearVelocity(ee_link, ee_pos_in_link);
	rotation = robot->rotation(ee_link);
	J = robot->J(ee_link, ee_pos_in_link);
	gravity = robot->jointGravityVector();

	cout << endl;
	cout << "------------------------------------------------------" << endl;
	cout << "               back to initial position               " << endl;
	cout << " call updateModel() to update kinematics and dynamics " << endl;
	cout << "------------------------------------------------------" << endl;
	cout << endl;
	cout << "robot coordinates : " << robot->q().transpose() << endl;
	cout << "position at end effector : " << position.transpose() << endl;
	cout << "velocity at end effector : " << velocity.transpose() << endl;
	cout << "orientation at end effector : \n" << rotation << endl;
	cout << "Mass matrix :\n" << robot->M() << endl;
	cout << "jacobian at the end effector \n" << J << endl;
	cout << "joint gravity : " << gravity.transpose() << endl;
	cout << endl;

	// other tests
	robot->setQ(VectorXd::Random(robot->dof()));
	robot->setDq(VectorXd::Random(robot->dof()));
	robot->updateModel();
	MatrixXd J_joint = robot->linkDependency(ee_link);
	cout << "Robot partial task jacobian for joint dependency: \n" << J_joint << endl;
	auto JdotQdot = robot->jDotQDot(ee_link, ee_pos_in_link);
	cout << "Robot Jdotqdot: " << JdotQdot.transpose() << "\n";

	Sai2Model::Sai2Model* humanoid = new Sai2Model::Sai2Model(humanoid_fname);
	humanoid->setQ(VectorXd::Random(humanoid->dof()));
	humanoid->setDq(VectorXd::Random(humanoid->dof()));
	humanoid->updateModel();
	J_joint = humanoid->linkDependency("ra_link6");
	cout << "Humanoid partial task jacobian for joint dependency: \n" << J_joint << endl;
	JdotQdot = humanoid->jDotQDot("ra_link6");
	cout << "Humanoid Jdotqdot: " << JdotQdot.transpose() << "\n";	

	return 0;
}
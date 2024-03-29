// 03-spherical joints:
// example of a robot with spherical joints.
// For the spherical joints, q contains the quaternion x, y and z components in
// the middle of the q vector, and the w component of the quaternion is at the
// end of the vector
#include <Sai2Model.h>

#include <iostream>
#include <memory>

using namespace std;

int main(int argc, char** argv) {
	const string robot_fname =
		string(EXAMPLES_FOLDER) + "/03-spherical_joints/rpspsbot.urdf";
	cout << "Loading robot file: " << robot_fname << endl;

	auto robot = std::make_shared<Sai2Model::Sai2Model>(robot_fname);

	cout << endl << endl;
	cout << "Display links. The base link is 0, links attached to "
			"moving joints are numbered from 1 and links attached to fixed "
			"joints are numbered from MAX_INT/2"
		 << endl;
	robot->displayLinks();
	cout << endl << endl;
	cout << "Display joints. Only moving joints are numbered. RBDL numbering "
			"is showed so it starts from 1.\nWARNING !!! this numbering should "
			"not be used to interact with the q vector which uses a different "
			"numbering (starting from 0, and spherical joints have multiple "
			"indexes)"
		 << endl;
	robot->displayJoints();
	cout << endl << endl;

	cout << "Spherical joints in robot model. The index is the position of the "
			"x component of the quaternion (the y and z components are at "
			"index+1 and index+2), and the w_index is the index of the w "
			"component of the quaternion (placed at the end of the q vector "
			"after all the non spherical joints indexes): "
		 << endl;
	for (const auto& joint_description : robot->sphericalJoints()) {
		cout << "name: " << joint_description.name
			 << " - index: " << joint_description.index
			 << " - w_index: " << joint_description.w_index << endl;
	}
	cout << endl << endl;

	cout << "size and contents of q and dq vectors. Their size is different "
			"when there are spherical joints. If there are n spherical joints, "
			"the n last components of the q vector contain the w part of the "
			"quaternions representation of each spherical joint"
		 << endl;
	cout << "q size: " << robot->qSize() << endl;
	cout << "dof size: " << robot->dof() << endl;
	cout << "q: " << robot->q().transpose() << endl;
	cout << "dq: " << robot->dq().transpose() << endl;
	cout << endl << endl;

	cout << "joint names from Sai2Model call" << std::endl;
	for (const auto& name : robot->jointNames()) {
		std::cout << name << " , ";
	}
	cout << endl << endl;

	cout << "joint names from Sai2Model individual call, looping on q_size "
			"indexes"
		 << std::endl;
	for (int i = 0; i < robot->qSize(); ++i) {
		cout << "joint: " << i << " - name: " << robot->jointName(i) << endl;
	}
	cout << endl << endl;

	cout << "joint index per name. The spherical joints return the index of "
			"the x component"
		 << endl;
	for (const auto& joint_name : robot->jointNames()) {
		cout << joint_name << ": " << robot->jointIndex(joint_name) << endl;
	}
	cout << endl << endl;

	cout << "spherical joint w indexes from individual call: " << endl;
	for (const auto& sph_joint : robot->sphericalJoints()) {
		cout << "name: " << sph_joint.name
			 << " -  w_index: " << robot->sphericalJointIndexW(sph_joint.name)
			 << endl;
	}
	cout << endl << endl;

	// position of end effector
	string eef_link_name = "link5";
	Eigen::Vector3d pos = robot->position(eef_link_name);
	cout << "position of end effector: " << endl;
	cout << pos.transpose() << endl;
	cout << endl << endl;

	// change spherical joint position
	string sph_joint_name = robot->sphericalJoints()[0].name;
	Eigen::Quaterniond quat = Eigen::Quaterniond(
		Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d(1, 0, 1)));
	robot->setSphericalQuat(sph_joint_name, quat);
	cout << "setting first spherical joint to quaternion: "
		 << quat.coeffs().transpose() << endl;
	cout << "getting first spherical joint quaternion from model: "
		 << robot->sphericalQuat(sph_joint_name).coeffs().transpose() << endl;
	cout << "new model q vector: " << robot->q().transpose() << endl;

	robot->updateKinematics();
	pos = robot->position(eef_link_name);
	cout << "new position of end effector:\n" << pos.transpose() << endl;
	cout << endl << endl;

	return 0;
}
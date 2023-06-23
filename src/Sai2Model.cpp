/*
 * Sai2Model.cpp
 *
 *  Wrapper around RBDL plus functions to facilitate the whole body control
 * framework from Stanford robotics lab
 *
 *  Created on: Dec 14, 2016
 *      Author: Mikael Jorda
 */

#include "Sai2Model.h"

#include <UrdfToSai2Model.h>
#include <rbdl/rbdl.h>

// #include <stdexcept>

using namespace std;
using namespace Eigen;

namespace {

bool is_valid_quaternion(double x, double y, double z, double w) {
	Eigen::Quaterniond quaternion(w, x, y, z);

	if (std::abs(quaternion.squaredNorm() - 1.0) > 1e-3) {
		return false;
	}

	return true;
}

}  // namespace

namespace Sai2Model {

Sai2Model::Sai2Model(const string path_to_model_file, bool verbose,
					 const Affine3d T_world_robot,
					 const Vector3d world_gravity) {
	_rbdl_model = new RigidBodyDynamics::Model();

	// parse rbdl model from urdf
	bool success = RigidBodyDynamics::URDFReadFromFile(
		path_to_model_file.c_str(), _rbdl_model, _link_names_to_id_map,
		_joint_names_to_id_map, _joint_limits, false, verbose);
	if (!success) {
		throw std::runtime_error("Error loading model [" + path_to_model_file +
								 "]\n");
	}
	_rbdl_model->gravity = world_gravity;

	// create joint id to name map
	for (auto pair : _joint_names_to_id_map) {
		string joint_name = pair.first;
		int rbdl_index = pair.second;
		const auto& rbdl_joint = _rbdl_model->mJoints[rbdl_index];
		switch (rbdl_joint.mJointType) {
			case RigidBodyDynamics::JointTypeSpherical:
				for (int i = 0; i < 3; ++i) {
					_joint_id_to_names_map[rbdl_joint.q_index + i] = joint_name;
				}
				_joint_id_to_names_map[_rbdl_model
										   ->multdof3_w_index[rbdl_index]] =
					joint_name;
				break;

			case RigidBodyDynamics::JointTypeRevolute:
			case RigidBodyDynamics::JointTypeRevoluteX:
			case RigidBodyDynamics::JointTypeRevoluteY:
			case RigidBodyDynamics::JointTypeRevoluteZ:
			case RigidBodyDynamics::JointTypePrismatic:
				_joint_id_to_names_map[rbdl_joint.q_index] = joint_name;
				break;

			default:
				throw std::runtime_error(
					"error generating joint id to name map\n");
				break;
		}
	}

	// set the base position in the world
	_T_world_robot = T_world_robot;

	// set the number of degrees of freedom
	_dof = _rbdl_model->dof_count;

	// set the size of q vector
	_q_size = _rbdl_model->q_size;

	// Initialize state vectors
	_q.setZero(_q_size);
	// special case handle spherical joints. See rbdl/Joint class for details.
	for (uint i = 0; i < _rbdl_model->mJoints.size(); ++i) {
		if (_rbdl_model->mJoints[i].mJointType ==
			RigidBodyDynamics::JointTypeSpherical) {
			_rbdl_model->SetQuaternion(i, RigidBodyDynamics::Math::Quaternion(),
									   _q);
			int index = _rbdl_model->mJoints[i].q_index;
			int w_index = _rbdl_model->multdof3_w_index[i];
			_spherical_joints.push_back(
				SphericalJointDescription(joint_name(index), index, w_index));
		}
	}

	_dq.setZero(_dof);
	_ddq.setZero(_dof);
	_M.setIdentity(_dof, _dof);
	_M_inv.setIdentity(_dof, _dof);

	updateModel();
}

Sai2Model::~Sai2Model() {
	delete _rbdl_model;
	_rbdl_model = NULL;
}

void Sai2Model::set_q(const Eigen::VectorXd& q) {
	if (q.size() != _q_size) {
		throw invalid_argument("q size inconsistent in Sai2Model::set_q");
		return;
	}
	for (const auto& sph_joint : _spherical_joints) {
		if (!is_valid_quaternion(q(sph_joint.index), q(sph_joint.index + 1),
								 q(sph_joint.index + 2),
								 q(sph_joint.w_index))) {
			throw invalid_argument(
				"trying to set an invalid quaternion for joint " +
				sph_joint.name + "at index " + std::to_string(sph_joint.index) +
				", and w_index: " + std::to_string(sph_joint.w_index));
			return;
		}
	}
	_q = q;
}

void Sai2Model::set_dq(const Eigen::VectorXd& dq) {
	if (dq.size() != _dof) {
		throw invalid_argument("dq size inconsistent in Sai2Model::set_dq");
		return;
	}
	_dq = dq;
}

bool Sai2Model::isLinkInRobot(const std::string& link_name) const {
	if (_link_names_to_id_map.find(link_name) == _link_names_to_id_map.end()) {
		return false;
	}
	return true;
}

void Sai2Model::updateKinematics() {
	UpdateKinematicsCustom(*_rbdl_model, &_q, &_dq, &_ddq);
}

void Sai2Model::updateModel() {
	updateKinematics();
	updateDynamics();
}

void Sai2Model::updateModel(const Eigen::MatrixXd& M) {
	updateKinematics();

	if (M.rows() != M.cols() || M.rows() != _dof) {
		throw invalid_argument(
			"M matrix dimensions inconsistent in Sai2Model::updateModel");
		return;
	}
	updateInverseInertia();
}

void Sai2Model::updateKinematicsCustom(
	bool update_frame, bool update_link_velocities,
	bool update_link_acceleration,	// this does not apply gravity
	bool use_ddq) {
	VectorXd* Q_set = update_frame ? &_q : NULL;
	VectorXd* dQ_set = update_link_velocities ? &_dq : NULL;
	VectorXd* ddQ_set = NULL;
	VectorXd zero_ddq = VectorXd::Zero(_dof);
	if (update_link_acceleration) {
		ddQ_set = use_ddq ? &_ddq : &zero_ddq;
	}
	UpdateKinematicsCustom(*_rbdl_model, Q_set, dQ_set, ddQ_set);
}

int Sai2Model::dof() { return _dof; }

int Sai2Model::q_size() { return _q_size; }

void Sai2Model::jointGravityVector(VectorXd& g) {
	if (g.size() != _dof) {
		g.resize(_dof);
	}
	g.setZero();

	vector<RigidBodyDynamics::Body>::iterator it_body;
	int body_id;

	for (it_body = _rbdl_model->mBodies.begin(), body_id = 0;
		 it_body != _rbdl_model->mBodies.end(); ++it_body, ++body_id) {
		double mass = it_body->mMass;
		MatrixXd Jv = MatrixXd::Zero(3, _dof);
		CalcPointJacobian(*_rbdl_model, _q, body_id, it_body->mCenterOfMass, Jv,
						  false);

		g += Jv.transpose() * _T_world_robot.linear().transpose() *
			 (-mass * worldGravity());
	}
}

void Sai2Model::coriolisForce(VectorXd& b) {
	// returns v + g. Need to substract the gravity from it
	NonlinearEffects(*_rbdl_model, _q, _dq, b);

	VectorXd g = VectorXd::Zero(_dof);
	jointGravityVector(g);

	b -= g;
}

void Sai2Model::coriolisPlusGravity(VectorXd& h) {
	NonlinearEffects(*_rbdl_model, _q, _dq, h);
}

void Sai2Model::factorizedChristoffelMatrix(MatrixXd& C) {
	C.setZero(_dof, _dof);

	VectorXd vi = VectorXd::Zero(_dof);
	VectorXd u = VectorXd::Zero(_dof);

	for (int i = 0; i < _dof; i++) {
		vi.setZero();
		u.setZero();
		vi(i) = 1;
		modifiedNewtonEuler(u, false, _q, _dq, vi, VectorXd::Zero(_dof));
		C.col(i) = u;
	}
}

void Sai2Model::J_0(MatrixXd& J, const string& link_name,
					const Vector3d& pos_in_link) {
	J.setZero(6, _dof);
	MatrixXd J_temp = MatrixXd::Zero(6, _dof);
	CalcPointJacobian6D(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link,
						J_temp, false);

	// RBDL gives Jw as the top 3 rows and Jv as the bottom part. We need to
	// swap it here
	J << J_temp.block(3, 0, 3, _dof), J_temp.block(0, 0, 3, _dof);
}
void Sai2Model::J_0WorldFrame(MatrixXd& J, const string& link_name,
							  const Vector3d& pos_in_link) {
	J.setZero(6, _dof);
	MatrixXd J_temp = MatrixXd::Zero(6, _dof);
	CalcPointJacobian6D(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link,
						J_temp, false);

	// RBDL gives Jw as the top 3 rows and Jv as the bottom part. We need to
	// swap it here
	J << _T_world_robot.linear() * J_temp.block(3, 0, 3, _dof),
		_T_world_robot.linear() * J_temp.block(0, 0, 3, _dof);
}
void Sai2Model::J_0LocalFrame(MatrixXd& J, const string& link_name,
							  const Vector3d& pos_in_link,
							  const Matrix3d& rot_in_link) {
	J.setZero(6, _dof);
	MatrixXd J_temp = MatrixXd::Zero(6, _dof);
	CalcPointJacobian6D(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link,
						J_temp, false);

	// Matrix3d link_rot = CalcBodyWorldOrientation(*_rbdl_model, _q,
	// linkIdRbdl(link_name), false).transpose();

	Matrix3d R_0_link;
	rotation(R_0_link, link_name);
	MatrixXd R_link_ee = rot_in_link;

	// RBDL gives Jw as the top 3 rows and Jv as the bottom part. We need to
	// swap it here
	J << R_link_ee.transpose() * R_0_link.transpose() *
			 J_temp.block(3, 0, 3, _dof),
		R_link_ee.transpose() * R_0_link.transpose() *
			J_temp.block(0, 0, 3, _dof);
}

void Sai2Model::J(MatrixXd& J, const string& link_name,
				  const Vector3d& pos_in_link) {
	J.setZero(6, _dof);
	CalcPointJacobian6D(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link, J,
						false);
}
void Sai2Model::JWorldFrame(MatrixXd& J, const string& link_name,
							const Vector3d& pos_in_link) {
	J.setZero(6, _dof);
	CalcPointJacobian6D(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link, J,
						false);

	J.block(0, 0, 3, _dof) = _T_world_robot.linear() * J.block(0, 0, 3, _dof);
	J.block(3, 0, 3, _dof) = _T_world_robot.linear() * J.block(3, 0, 3, _dof);
}
void Sai2Model::JLocalFrame(MatrixXd& J, const string& link_name,
							const Vector3d& pos_in_link,
							const Matrix3d& rot_in_link) {
	J.setZero(6, _dof);
	CalcPointJacobian6D(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link, J,
						false);

	Matrix3d link_rot =
		CalcBodyWorldOrientation(*_rbdl_model, _q, linkIdRbdl(link_name), false)
			.transpose();

	J.block(0, 0, 3, _dof) =
		rot_in_link.transpose() * link_rot.transpose() * J.block(0, 0, 3, _dof);
	J.block(3, 0, 3, _dof) =
		rot_in_link.transpose() * link_rot.transpose() * J.block(3, 0, 3, _dof);
}

void Sai2Model::Jv(MatrixXd& J, const string& link_name,
				   const Vector3d& pos_in_link) {
	J.setZero(3, _dof);
	CalcPointJacobian(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link, J,
					  false);
}
void Sai2Model::JvWorldFrame(MatrixXd& J, const string& link_name,
							 const Vector3d& pos_in_link) {
	J.setZero(3, _dof);
	CalcPointJacobian(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link, J,
					  false);

	J = _T_world_robot.linear() * J;
}
void Sai2Model::JvLocalFrame(MatrixXd& J, const string& link_name,
							 const Vector3d& pos_in_link,
							 const Matrix3d& rot_in_link) {
	J.setZero(3, _dof);
	CalcPointJacobian(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link, J,
					  false);

	Matrix3d link_rot =
		CalcBodyWorldOrientation(*_rbdl_model, _q, linkIdRbdl(link_name), false)
			.transpose();

	J = rot_in_link.transpose() * link_rot.transpose() * J;
}

void Sai2Model::Jw(MatrixXd& J, const string& link_name) {
	// compute the full jacobian at the center of the link and take rotational
	// part
	MatrixXd J_temp = MatrixXd::Zero(6, _dof);
	CalcPointJacobian6D(*_rbdl_model, _q, linkIdRbdl(link_name),
						Vector3d::Zero(), J_temp, false);
	J = J_temp.topRows<3>();
}
void Sai2Model::JwWorldFrame(MatrixXd& J, const string& link_name) {
	// compute the full jacobian at the center of the link and take rotational
	// part
	MatrixXd J_temp = MatrixXd::Zero(6, _dof);
	CalcPointJacobian6D(*_rbdl_model, _q, linkIdRbdl(link_name),
						Vector3d::Zero(), J_temp, false);
	J = _T_world_robot.linear() * J_temp.topRows<3>();
}
void Sai2Model::JwLocalFrame(MatrixXd& J, const string& link_name,
							 const Matrix3d& rot_in_link) {
	// compute the full jacobian at the center of the link and take rotational
	// part
	MatrixXd J_temp = MatrixXd::Zero(6, _dof);
	CalcPointJacobian6D(*_rbdl_model, _q, linkIdRbdl(link_name),
						Vector3d::Zero(), J_temp, false);

	Matrix3d link_rot =
		CalcBodyWorldOrientation(*_rbdl_model, _q, linkIdRbdl(link_name), false)
			.transpose();

	J = rot_in_link.transpose() * link_rot.transpose() * J_temp.topRows<3>();
}

void Sai2Model::computeIK3d(
	VectorXd& q_result, const vector<string>& link_names,
	const vector<Vector3d>& point_positions_in_links,
	const vector<Vector3d>& desired_point_positions_in_robot_frame) {
	if (link_names.size() != point_positions_in_links.size() ||
		link_names.size() != desired_point_positions_in_robot_frame.size()) {
		throw runtime_error(
			"size of the 3 input vectors do not match in "
			"Sai2Model::computeIK3d\n");
	}

	vector<unsigned int> link_ids;
	vector<RigidBodyDynamics::Math::Vector3d> body_point;
	vector<RigidBodyDynamics::Math::Vector3d> target_pos;
	for (int i = 0; i < link_names.size(); i++) {
		link_ids.push_back(linkIdRbdl(link_names[i]));
		body_point.push_back(point_positions_in_links[i]);
		target_pos.push_back(desired_point_positions_in_robot_frame[i]);
	}

	InverseKinematics(
		*_rbdl_model, _q, link_ids, body_point, target_pos,
		q_result);	// modifies the internal model so we need to re update the
					// kinematics with the previous q value to keep it unchanged
	updateKinematics();
}

void Sai2Model::computeIK3d_JL(
	VectorXd& q_result, const vector<string>& link_names,
	const vector<Vector3d>& point_positions_in_links,
	const vector<Vector3d>& desired_point_positions_in_robot_frame,
	const VectorXd q_min, const VectorXd q_max, const VectorXd weights) {
	if (link_names.size() != point_positions_in_links.size() ||
		link_names.size() != desired_point_positions_in_robot_frame.size()) {
		throw runtime_error(
			"size of the 3 input vectors do not match in "
			"Sai2Model::computeIK3d\n");
	}

	vector<unsigned int> link_ids;
	vector<RigidBodyDynamics::Math::Vector3d> body_point;
	vector<RigidBodyDynamics::Math::Vector3d> target_pos;
	for (int i = 0; i < link_names.size(); i++) {
		link_ids.push_back(linkIdRbdl(link_names[i]));
		body_point.push_back(point_positions_in_links[i]);
		target_pos.push_back(desired_point_positions_in_robot_frame[i]);
	}

	InverseKinematics_JL(*_rbdl_model, _q, q_min, q_max, weights, link_ids,
						 body_point, target_pos, q_result, 1.0e-12, 0.0001,
						 500);
	// InverseKinematics_JL modifies the internal model so we need to re update
	// the kinematics with the previous q value to keep it unchanged
	updateKinematics();
}

void Sai2Model::computeIK6d(
	VectorXd& q_result, const vector<string>& link_names,
	const vector<Affine3d>& frame_in_links,
	const vector<Affine3d>& desired_frame_locations_in_robot_frame) {
	if (link_names.size() != frame_in_links.size() ||
		link_names.size() != desired_frame_locations_in_robot_frame.size()) {
		throw runtime_error(
			"size of the 3 input vectors do not match in "
			"Sai2Model::computeIK6d\n");
	}

	vector<unsigned int> link_ids;
	vector<RigidBodyDynamics::Math::Vector3d> point_pos;
	vector<RigidBodyDynamics::Math::Vector3d> desired_point_pos;
	for (int i = 0; i < link_names.size(); i++) {
		link_ids.push_back(linkIdRbdl(link_names[i]));
		link_ids.push_back(linkIdRbdl(link_names[i]));
		link_ids.push_back(linkIdRbdl(link_names[i]));

		point_pos.push_back(frame_in_links[i] * Vector3d::Zero());
		point_pos.push_back(frame_in_links[i] * Vector3d::UnitX());
		point_pos.push_back(frame_in_links[i] * Vector3d::UnitY());

		desired_point_pos.push_back(desired_frame_locations_in_robot_frame[i] *
									Vector3d::Zero());
		desired_point_pos.push_back(desired_frame_locations_in_robot_frame[i] *
									Vector3d::UnitX());
		desired_point_pos.push_back(desired_frame_locations_in_robot_frame[i] *
									Vector3d::UnitY());
	}

	InverseKinematics(
		*_rbdl_model, _q, link_ids, point_pos, desired_point_pos,
		q_result);	// modifies the internal model so we need to re update the
					// kinematics with the previous q value to keep it unchanged
	updateKinematics();
}

void Sai2Model::transform(Affine3d& T, const string& link_name,
						  const Vector3d& pos_in_link,
						  const Matrix3d& rot_in_link) {
	unsigned int link_id = linkIdRbdl(link_name);
	T.linear() =
		CalcBodyWorldOrientation(*_rbdl_model, _q, link_id, false).transpose() *
		rot_in_link;
	T.translation() = CalcBodyToBaseCoordinates(*_rbdl_model, _q, link_id,
												pos_in_link, false);
}
void Sai2Model::transformInWorld(Affine3d& T, const string& link_name,
								 const Vector3d& pos_in_link,
								 const Matrix3d& rot_in_link) {
	transform(T, link_name, pos_in_link, rot_in_link);
	T = _T_world_robot * T;
}

void Sai2Model::velocity6d(VectorXd& vel6d, const string link_name,
						   const Vector3d& pos_in_link) {
	vel6d.setZero(6);
	VectorXd v_tmp = VectorXd::Zero(6);
	v_tmp = CalcPointVelocity6D(*_rbdl_model, _q, _dq, linkIdRbdl(link_name),
								pos_in_link, false);
	vel6d << v_tmp.tail(3), v_tmp.head(3);
}

void Sai2Model::velocity6dInWorld(VectorXd& vel6d, const string link_name,
								  const Vector3d& pos_in_link) {
	vel6d.setZero(6);
	VectorXd v_tmp = VectorXd::Zero(6);
	v_tmp = CalcPointVelocity6D(*_rbdl_model, _q, _dq, linkIdRbdl(link_name),
								pos_in_link, false);
	vel6d << _T_world_robot.linear() * v_tmp.tail(3),
		_T_world_robot.linear() * v_tmp.head(3);
}

void Sai2Model::acceleration6d(VectorXd& accel6d, const string link_name,
							   const Vector3d& pos_in_link) {
	accel6d.setZero(6);
	VectorXd a_tmp = VectorXd::Zero(6);
	a_tmp = CalcPointAcceleration6D(*_rbdl_model, _q, _dq, _ddq,
									linkIdRbdl(link_name), pos_in_link, false);
	accel6d << a_tmp.tail(3), a_tmp.head(3);
}
void Sai2Model::acceleration6dInWorld(VectorXd& accel6d, const string link_name,
									  const Vector3d& pos_in_link) {
	accel6d.setZero(6);
	VectorXd a_tmp = VectorXd::Zero(6);
	a_tmp = CalcPointAcceleration6D(*_rbdl_model, _q, _dq, _ddq,
									linkIdRbdl(link_name), pos_in_link, false);
	accel6d << _T_world_robot.linear() * a_tmp.tail(3),
		_T_world_robot.linear() * a_tmp.head(3);
}

void Sai2Model::position(Vector3d& pos, const string& link_name,
						 const Vector3d& pos_in_link) {
	pos = CalcBodyToBaseCoordinates(*_rbdl_model, _q, linkIdRbdl(link_name),
									pos_in_link, false);
}

void Sai2Model::positionInWorld(Vector3d& pos, const string& link_name,
								const Vector3d& pos_in_link) {
	pos = CalcBodyToBaseCoordinates(*_rbdl_model, _q, linkIdRbdl(link_name),
									pos_in_link, false);
	pos = _T_world_robot * pos;
}

void Sai2Model::linearVelocity(Vector3d& vel, const string& link_name,
							   const Vector3d& pos_in_link) {
	vel = CalcPointVelocity(*_rbdl_model, _q, _dq, linkIdRbdl(link_name),
							pos_in_link, false);
}
void Sai2Model::linearVelocityInWorld(Vector3d& vel, const string& link_name,
									  const Vector3d& pos_in_link) {
	vel = CalcPointVelocity(*_rbdl_model, _q, _dq, linkIdRbdl(link_name),
							pos_in_link, false);
	vel = _T_world_robot.linear() * vel;
}

void Sai2Model::linearAcceleration(Vector3d& accel, const string& link_name,
								   const Vector3d& pos_in_link) {
	accel = CalcPointAcceleration(*_rbdl_model, _q, _dq, _ddq,
								  linkIdRbdl(link_name), pos_in_link, false);
}
void Sai2Model::linearAccelerationInWorld(Vector3d& accel,
										  const string& link_name,
										  const Vector3d& pos_in_link) {
	accel = CalcPointAcceleration(*_rbdl_model, _q, _dq, _ddq,
								  linkIdRbdl(link_name), pos_in_link, false);
	accel = _T_world_robot.linear() * accel;
}

void Sai2Model::rotation(Matrix3d& rot, const string& link_name,
						 const Matrix3d& rot_in_link) {
	rot =
		CalcBodyWorldOrientation(*_rbdl_model, _q, linkIdRbdl(link_name), false)
			.transpose();
	rot = rot * rot_in_link;
}
void Sai2Model::rotationInWorld(Matrix3d& rot, const string& link_name,
								const Matrix3d& rot_in_link) {
	rot =
		CalcBodyWorldOrientation(*_rbdl_model, _q, linkIdRbdl(link_name), false)
			.transpose();
	rot = _T_world_robot.linear() * rot * rot_in_link;
}

void Sai2Model::angularVelocity(Vector3d& avel, const string& link_name,
								const Vector3d& pos_in_link) {
	VectorXd v_tmp = VectorXd::Zero(6);
	v_tmp = CalcPointVelocity6D(*_rbdl_model, _q, _dq, linkIdRbdl(link_name),
								pos_in_link, false);
	avel = v_tmp.head(3);
}
void Sai2Model::angularVelocityInWorld(Vector3d& avel, const string& link_name,
									   const Vector3d& pos_in_link) {
	VectorXd v_tmp = VectorXd::Zero(6);
	v_tmp = CalcPointVelocity6D(*_rbdl_model, _q, _dq, linkIdRbdl(link_name),
								pos_in_link, false);
	avel = v_tmp.head(3);
	avel = _T_world_robot.linear() * avel;
}

void Sai2Model::angularAcceleration(Vector3d& aaccel, const string& link_name,
									const Vector3d& pos_in_link) {
	VectorXd a_tmp = VectorXd::Zero(6);
	a_tmp = CalcPointAcceleration6D(*_rbdl_model, _q, _dq, _ddq,
									linkIdRbdl(link_name), pos_in_link, false);
	aaccel = a_tmp.head(3);
}
void Sai2Model::angularAccelerationInWorld(Vector3d& aaccel,
										   const string& link_name,
										   const Vector3d& pos_in_link) {
	VectorXd a_tmp = VectorXd::Zero(6);
	a_tmp = CalcPointAcceleration6D(*_rbdl_model, _q, _dq, _ddq,
									linkIdRbdl(link_name), pos_in_link, false);
	aaccel = a_tmp.head(3);
	aaccel = _T_world_robot.linear() * aaccel;
}

unsigned int Sai2Model::linkIdRbdl(const string& link_name) {
	if (_link_names_to_id_map.find(link_name) == _link_names_to_id_map.end()) {
		throw runtime_error("link [" + link_name + "] does not exist");
	}
	return _link_names_to_id_map[link_name];
}

int Sai2Model::joint_index(const string& joint_name) {
	if (_joint_names_to_id_map.find(joint_name) ==
		_joint_names_to_id_map.end()) {
		throw runtime_error("joint [" + joint_name + "] does not exist");
	}
	return _rbdl_model->mJoints[_joint_names_to_id_map[joint_name]].q_index;
}

int Sai2Model::spherical_joint_w_index(const string& joint_name) {
	if (_joint_names_to_id_map.find(joint_name) ==
		_joint_names_to_id_map.end()) {
		throw runtime_error("joint [" + joint_name + "] does not exist");
	}
	for (auto it = _spherical_joints.cbegin(); it != _spherical_joints.cend();
		 ++it) {
		if (it->name == joint_name) {
			return it->w_index;
		}
	}
	throw runtime_error("joint [" + joint_name + "] is not spherical");
}

std::string Sai2Model::joint_name(const int joint_id) {
	if (joint_id < 0 || joint_id >= _q_size) {
		throw std::invalid_argument(
			"cannot get joint name for id out of bounds");
	}
	return _joint_id_to_names_map[joint_id];
}

std::vector<std::string> Sai2Model::joint_names() const {
	std::vector<std::string> names;
	names.reserve(_joint_names_to_id_map.size());

	for (const auto& pair : _joint_names_to_id_map) {
		names.push_back(pair.first);
	}

	return names;
}

void Sai2Model::getLinkMass(double& mass, Vector3d& center_of_mass,
							Matrix3d& inertia, const string& link_name) {
	RigidBodyDynamics::Body b = _rbdl_model->mBodies[linkIdRbdl(link_name)];

	mass = b.mMass;
	center_of_mass = b.mCenterOfMass;
	inertia = b.mInertia;
}

void Sai2Model::getLinkMass(double& mass, Vector3d& center_of_mass,
							const string& link_name) {
	RigidBodyDynamics::Body b = _rbdl_model->mBodies[linkIdRbdl(link_name)];

	mass = b.mMass;
	center_of_mass = b.mCenterOfMass;
}

void Sai2Model::comPosition(Vector3d& robot_com) {
	robot_com.setZero();
	double mass;
	double robot_mass = 0.0;
	Vector3d center_of_mass_local;
	Vector3d center_of_mass_global_frame;
	Matrix3d inertia;
	int n_bodies = _rbdl_model->mBodies.size();
	for (int i = 0; i < n_bodies; i++) {
		RigidBodyDynamics::Body b = _rbdl_model->mBodies[i];

		mass = b.mMass;
		center_of_mass_local = b.mCenterOfMass;

		center_of_mass_global_frame = CalcBodyToBaseCoordinates(
			*_rbdl_model, _q, i, center_of_mass_local, false);

		robot_com += center_of_mass_global_frame * mass;
		robot_mass += mass;
	}
	robot_com = robot_com / robot_mass;
}

void Sai2Model::comJacobian(MatrixXd& Jv_com) {
	Jv_com.setZero(3, _dof);
	MatrixXd link_Jv;
	double mass;
	double robot_mass = 0.0;
	Vector3d center_of_mass_local;
	Matrix3d inertia;
	int n_bodies = _rbdl_model->mBodies.size();
	for (int i = 0; i < n_bodies; i++) {
		RigidBodyDynamics::Body b = _rbdl_model->mBodies[i];

		mass = b.mMass;
		center_of_mass_local = b.mCenterOfMass;
		inertia = b.mInertia;

		link_Jv.setZero(3, _dof);
		CalcPointJacobian(*_rbdl_model, _q, i, center_of_mass_local, link_Jv,
						  false);

		Jv_com += link_Jv * mass;
		robot_mass += mass;
	}
	Jv_com = Jv_com / robot_mass;
}

void Sai2Model::URangeJacobian(MatrixXd& URange, const MatrixXd& task_jacobian,
							   const MatrixXd& N_prec, const double tolerance) {
	// check matrices dimmnsions
	if (N_prec.rows() != N_prec.cols() || N_prec.rows() != _dof) {
		throw invalid_argument(
			"N_prec matrix dimensions inconsistent in "
			"Sai2Model::nullspaceMatrix");
		return;
	} else if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"jacobian matrix dimensions inconsistent with model dof in "
			"Sai2Model::nullspaceMatrix");
		return;
	}

	const int task_size = task_jacobian.rows();

	JacobiSVD<MatrixXd> svd(task_jacobian * N_prec,
							ComputeThinU | ComputeThinV);

	double sigma_0 = svd.singularValues()(0);
	if (sigma_0 < tolerance) {
		URange = MatrixXd::Zero(task_size, 1);
		return;
	}

	int task_dof = task_size;
	for (int i = svd.singularValues().size() - 1; i > 0; i--) {
		if (svd.singularValues()(i) / sigma_0 < tolerance) {
			task_dof -= 1;
		} else {
			break;
		}
	}

	if (task_dof == task_size) {
		URange = MatrixXd::Identity(task_size, task_size);
	} else {
		URange = svd.matrixU().leftCols(task_dof);
	}
}

void Sai2Model::URangeJacobian(MatrixXd& URange, const MatrixXd& task_jacobian,
							   const double tolerance) {
	MatrixXd N_prec = MatrixXd::Identity(_dof, _dof);
	URangeJacobian(URange, task_jacobian, N_prec, tolerance);
}

void Sai2Model::taskInertiaMatrix(MatrixXd& Lambda,
								  const MatrixXd& task_jacobian) {
	// check the task jacobian is compatible with the robot model
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"Sai2Model::taksInertiaMatrix");
		return;
	}

	// resize Matrices
	int k = task_jacobian.rows();
	Lambda.setZero(k, k);

	// compute task inertia
	MatrixXd inv_inertia = task_jacobian * _M_inv * task_jacobian.transpose();
	Lambda = inv_inertia.llt().solve(MatrixXd::Identity(k, k));
}

void Sai2Model::taskInertiaMatrixWithPseudoInv(MatrixXd& Lambda,
											   const MatrixXd& task_jacobian) {
	// check the task jacobian is compatible with the robot model
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"Sai2Model::taksInertiaMatrixWithPseudoInv");
		return;
	}

	// resize Matrices
	int k = task_jacobian.rows();
	Lambda.setZero(k, k);

	// compute task inertia
	MatrixXd inv_inertia = task_jacobian * _M_inv * task_jacobian.transpose();

	// compute SVD pseudoinverse
	// TODO: make class function?
	JacobiSVD<MatrixXd> svd(inv_inertia, ComputeThinU | ComputeThinV);
	const double epsilon = numeric_limits<double>::epsilon();
	double tolerance = epsilon * max(inv_inertia.cols(), inv_inertia.rows()) *
					   svd.singularValues().array().abs()(0);
	Lambda = svd.matrixV() *
			 (svd.singularValues().array().abs() > tolerance)
				 .select(svd.singularValues().array().inverse(), 0)
				 .matrix()
				 .asDiagonal() *
			 svd.matrixU().adjoint();
}

void Sai2Model::dynConsistentInverseJacobian(MatrixXd& Jbar,
											 const MatrixXd& task_jacobian) {
	// check the task jacobian is compatible with the robot model
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"Sai2Model::dynConsistentInverseJacobian");
		return;
	}

	// resize Matrices
	int k = task_jacobian.rows();
	Jbar.setZero(_dof, k);

	MatrixXd task_inertia(task_jacobian.rows(), task_jacobian.rows());
	taskInertiaMatrix(task_inertia, task_jacobian);
	Jbar = _M_inv * task_jacobian.transpose() * task_inertia;
}

void Sai2Model::nullspaceMatrix(MatrixXd& N, const MatrixXd& task_jacobian) {
	MatrixXd N_prec = MatrixXd::Identity(dof(), dof());
	nullspaceMatrix(N, task_jacobian, N_prec);
}

void Sai2Model::nullspaceMatrix(MatrixXd& N, const MatrixXd& task_jacobian,
								const MatrixXd& N_prec) {
	// check matrices dimmnsions
	if (N_prec.rows() != N_prec.cols() || N_prec.rows() != _dof) {
		throw invalid_argument(
			"N_prec matrix dimensions inconsistent in "
			"Sai2Model::nullspaceMatrix");
		return;
	} else if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"jacobian matrix dimensions inconsistent with model dof in "
			"Sai2Model::nullspaceMatrix");
		return;
	}

	// resize Matrices
	N.setIdentity(_dof, _dof);

	MatrixXd Jbar = MatrixXd::Zero(task_jacobian.cols(), task_jacobian.rows());
	dynConsistentInverseJacobian(Jbar, task_jacobian);
	N = MatrixXd::Identity(_dof, _dof) - Jbar * task_jacobian;
	N = N * N_prec;
}

void Sai2Model::operationalSpaceMatrices(MatrixXd& Lambda, MatrixXd& Jbar,
										 MatrixXd& N,
										 const MatrixXd& task_jacobian) {
	MatrixXd N_prec = MatrixXd::Identity(dof(), dof());
	operationalSpaceMatrices(Lambda, Jbar, N, task_jacobian, N_prec);
}

void Sai2Model::operationalSpaceMatrices(MatrixXd& Lambda, MatrixXd& Jbar,
										 MatrixXd& N,
										 const MatrixXd& task_jacobian,
										 const MatrixXd& N_prec) {
	// check matrices have the right size
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"Sai2Model::operationalSpaceMatrices");
		return;
	} else if (N_prec.rows() != N_prec.cols() || N_prec.rows() != _dof) {
		throw invalid_argument(
			"N_prec matrix dimensions inconsistent in "
			"Sai2Model::operationalSpaceMatrices");
		return;
	}

	// resize matrices
	int k = task_jacobian.rows();
	Lambda.setZero(k, k);
	Jbar.setZero(_dof, k);
	N.setIdentity(_dof, _dof);

	// Compute the matrices
	MatrixXd inv_inertia = task_jacobian * _M_inv * task_jacobian.transpose();
	Lambda = inv_inertia.llt().solve(MatrixXd::Identity(k, k));
	Jbar = _M_inv * task_jacobian.transpose() * Lambda;
	MatrixXd Ni = MatrixXd::Identity(_dof, _dof);
	N = MatrixXd::Identity(_dof, _dof) - Jbar * task_jacobian;
	N = N * N_prec;
}

void Sai2Model::addEnvironmentalContact(const string link,
										const Vector3d pos_in_link,
										const Matrix3d orientation,
										const ContactType contact_type) {
	for (vector<ContactModel>::iterator it = _environmental_contacts.begin();
		 it != _environmental_contacts.end(); ++it) {
		if (it->_link_name == link) {
			throw invalid_argument(
				"Environmental contact on link " + link +
				" already exists in Sai2Model::addEnvironmentalContact()");
		}
	}
	_environmental_contacts.push_back(
		ContactModel(link, pos_in_link, orientation, contact_type));
}
void Sai2Model::deleteEnvironmentalContact(const string link_name) {
	vector<ContactModel> new_contacts;
	for (vector<ContactModel>::iterator it = _environmental_contacts.begin();
		 it != _environmental_contacts.end(); ++it) {
		if (it->_link_name != link_name) {
			new_contacts.push_back(*it);
		}
	}
	_environmental_contacts = new_contacts;
}

void Sai2Model::updateEnvironmentalContact(const string link,
										   const Vector3d pos_in_link,
										   const Matrix3d orientation,
										   const ContactType contact_type) {
	for (vector<ContactModel>::iterator it = _environmental_contacts.begin();
		 it != _environmental_contacts.end(); ++it) {
		if (it->_link_name == link) {
			it->_contact_position = pos_in_link;
			it->_contact_orientation = orientation;
			it->_contact_type = contact_type;
			return;
		}
	}
	throw invalid_argument(
		"Environmental contact on link " + link +
		" does not exist in Sai2Model::updateManipulationContact()");
}

void Sai2Model::addManipulationContact(const string link,
									   const Vector3d pos_in_link,
									   const Matrix3d orientation,
									   const ContactType contact_type) {
	for (vector<ContactModel>::iterator it = _manipulation_contacts.begin();
		 it != _manipulation_contacts.end(); ++it) {
		if (it->_link_name == link) {
			throw invalid_argument(
				"Environmental contact on link " + link +
				" already exists in Sai2Model::addManipulationContact()");
		}
	}
	_manipulation_contacts.push_back(
		ContactModel(link, pos_in_link, orientation, contact_type));
}

void Sai2Model::deleteManipulationContact(const string link_name) {
	vector<ContactModel> new_contacts;
	for (vector<ContactModel>::iterator it = _manipulation_contacts.begin();
		 it != _manipulation_contacts.end(); ++it) {
		if (it->_link_name != link_name) {
			new_contacts.push_back(*it);
		}
	}
	_manipulation_contacts = new_contacts;
}

void Sai2Model::updateManipulationContact(const string link,
										  const Vector3d pos_in_link,
										  const Matrix3d orientation,
										  const ContactType contact_type) {
	for (vector<ContactModel>::iterator it = _manipulation_contacts.begin();
		 it != _manipulation_contacts.end(); ++it) {
		if (it->_link_name == link) {
			it->_contact_position = pos_in_link;
			it->_contact_orientation = orientation;
			it->_contact_type = contact_type;
			return;
		}
	}
	throw invalid_argument(
		"Environmental contact on link " + link +
		" does not exist in Sai2Model::updateManipulationContact()");
}

void Sai2Model::manipulationGraspMatrix(MatrixXd& G, MatrixXd& G_inv,
										Matrix3d& R,
										const Vector3d center_point) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _manipulation_contacts.size(); i++) {
		Vector3d pi;
		position(pi, _manipulation_contacts[i]._link_name,
				 _manipulation_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_manipulation_contacts[i]._contact_type);
	}
	graspMatrix(G, G_inv, R, center_point, contact_locations, contact_types);
}
void Sai2Model::manipulationGraspMatrixInWorld(MatrixXd& G, MatrixXd& G_inv,
											   Matrix3d& R,
											   const Vector3d center_point) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _manipulation_contacts.size(); i++) {
		Vector3d pi;
		positionInWorld(pi, _manipulation_contacts[i]._link_name,
						_manipulation_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_manipulation_contacts[i]._contact_type);
	}
	graspMatrix(G, G_inv, R, center_point, contact_locations, contact_types);
}
void Sai2Model::manipulationGraspMatrixLocalContactForces(
	MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, const Vector3d center_point) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _manipulation_contacts.size(); i++) {
		Vector3d pi;
		position(pi, _manipulation_contacts[i]._link_name,
				 _manipulation_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_manipulation_contacts[i]._contact_type);
	}
	graspMatrix(G, G_inv, R, center_point, contact_locations, contact_types);

	int n = _manipulation_contacts.size();
	int k = 0;
	for (int i = 0; i < n; i++) {
		if (_manipulation_contacts[i]._contact_type == SurfaceContact) {
			k++;
		}
	}

	int k_tmp = 0;
	for (int i = 0; i < n; i++) {
		Matrix3d R_tmp = Matrix3d::Identity();
		rotation(R_tmp, _manipulation_contacts[i]._link_name,
				 _manipulation_contacts[i]._contact_orientation);

		G.block(0, 3 * i, 3 * (n + k), 3) =
			G.block(0, 3 * i, 3 * (n + k), 3) * R_tmp;
		G_inv.block(3 * i, 0, 3, 3 * (n + k)) =
			R_tmp.transpose() * G_inv.block(3 * i, 0, 3, 3 * (n + k));
		if (_manipulation_contacts[i]._contact_type == SurfaceContact) {
			G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
				G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
				R_tmp.transpose() *
				G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));

			if (n > 2) {
				G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
					R_tmp.transpose() *
					G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));
				G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
					G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			}

			k_tmp++;
		}
	}
}
void Sai2Model::manipulationGraspMatrixLocalContactForcesToWorld(
	MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, const Vector3d center_point) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _manipulation_contacts.size(); i++) {
		Vector3d pi;
		positionInWorld(pi, _manipulation_contacts[i]._link_name,
						_manipulation_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_manipulation_contacts[i]._contact_type);
	}
	graspMatrix(G, G_inv, R, center_point, contact_locations, contact_types);

	int n = _manipulation_contacts.size();
	int k = 0;
	for (int i = 0; i < n; i++) {
		if (_manipulation_contacts[i]._contact_type == SurfaceContact) {
			k++;
		}
	}

	int k_tmp = 0;
	for (int i = 0; i < n; i++) {
		Matrix3d R_tmp = Matrix3d::Identity();
		rotationInWorld(R_tmp, _manipulation_contacts[i]._link_name,
						_manipulation_contacts[i]._contact_orientation);

		G.block(0, 3 * i, 3 * (n + k), 3) =
			G.block(0, 3 * i, 3 * (n + k), 3) * R_tmp;
		G_inv.block(3 * i, 0, 3, 3 * (n + k)) =
			R_tmp.transpose() * G_inv.block(3 * i, 0, 3, 3 * (n + k));
		if (_manipulation_contacts[i]._contact_type == SurfaceContact) {
			G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
				G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
				R_tmp.transpose() *
				G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));

			if (n > 2) {
				G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
					R_tmp.transpose() *
					G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));
				G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
					G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			}

			k_tmp++;
		}
	}
}

void Sai2Model::manipulationGraspMatrixAtGeometricCenter(
	MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, Vector3d& geometric_center) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _manipulation_contacts.size(); i++) {
		Vector3d pi;
		position(pi, _manipulation_contacts[i]._link_name,
				 _manipulation_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_manipulation_contacts[i]._contact_type);
	}
	graspMatrixAtGeometricCenter(G, G_inv, R, geometric_center,
								 contact_locations, contact_types);
}

void Sai2Model::manipulationGraspMatrixAtGeometricCenterInWorld(
	MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, Vector3d& geometric_center) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _manipulation_contacts.size(); i++) {
		Vector3d pi;
		positionInWorld(pi, _manipulation_contacts[i]._link_name,
						_manipulation_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_manipulation_contacts[i]._contact_type);
	}
	graspMatrixAtGeometricCenter(G, G_inv, R, geometric_center,
								 contact_locations, contact_types);
}
void Sai2Model::manipulationGraspMatrixAtGeometricCenterLocalContactForces(
	MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, Vector3d& geometric_center) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _manipulation_contacts.size(); i++) {
		Vector3d pi;
		position(pi, _manipulation_contacts[i]._link_name,
				 _manipulation_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_manipulation_contacts[i]._contact_type);
	}
	graspMatrixAtGeometricCenter(G, G_inv, R, geometric_center,
								 contact_locations, contact_types);

	int n = _manipulation_contacts.size();
	int k = 0;
	for (int i = 0; i < n; i++) {
		if (_manipulation_contacts[i]._contact_type == SurfaceContact) {
			k++;
		}
	}

	int k_tmp = 0;
	for (int i = 0; i < n; i++) {
		Matrix3d R_tmp = Matrix3d::Identity();
		rotation(R_tmp, _manipulation_contacts[i]._link_name,
				 _manipulation_contacts[i]._contact_orientation);

		G.block(0, 3 * i, 3 * (n + k), 3) =
			G.block(0, 3 * i, 3 * (n + k), 3) * R_tmp;
		G_inv.block(3 * i, 0, 3, 3 * (n + k)) =
			R_tmp.transpose() * G_inv.block(3 * i, 0, 3, 3 * (n + k));
		if (_manipulation_contacts[i]._contact_type == SurfaceContact) {
			G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
				G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
				R_tmp.transpose() *
				G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));

			if (n > 2) {
				G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
					R_tmp.transpose() *
					G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));
				G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
					G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			}

			k_tmp++;
		}
	}
}
void Sai2Model::
	manipulationGraspMatrixAtGeometricCenterLocalContactForcesToWorld(
		MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, Vector3d& geometric_center) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _manipulation_contacts.size(); i++) {
		Vector3d pi;
		positionInWorld(pi, _manipulation_contacts[i]._link_name,
						_manipulation_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_manipulation_contacts[i]._contact_type);
	}
	graspMatrixAtGeometricCenter(G, G_inv, R, geometric_center,
								 contact_locations, contact_types);

	int n = _manipulation_contacts.size();
	int k = 0;
	for (int i = 0; i < n; i++) {
		if (_manipulation_contacts[i]._contact_type == SurfaceContact) {
			k++;
		}
	}

	int k_tmp = 0;
	for (int i = 0; i < n; i++) {
		Matrix3d R_tmp = Matrix3d::Identity();
		rotationInWorld(R_tmp, _manipulation_contacts[i]._link_name,
						_manipulation_contacts[i]._contact_orientation);

		G.block(0, 3 * i, 3 * (n + k), 3) =
			G.block(0, 3 * i, 3 * (n + k), 3) * R_tmp;
		G_inv.block(3 * i, 0, 3, 3 * (n + k)) =
			R_tmp.transpose() * G_inv.block(3 * i, 0, 3, 3 * (n + k));
		if (_manipulation_contacts[i]._contact_type == SurfaceContact) {
			G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
				G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
				R_tmp.transpose() *
				G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));

			if (n > 2) {
				G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
					R_tmp.transpose() *
					G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));
				G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
					G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			}

			k_tmp++;
		}
	}
}

void Sai2Model::environmentalGraspMatrix(MatrixXd& G, MatrixXd& G_inv,
										 Matrix3d& R,
										 const Vector3d center_point) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _environmental_contacts.size(); i++) {
		Vector3d pi;
		position(pi, _environmental_contacts[i]._link_name,
				 _environmental_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_environmental_contacts[i]._contact_type);
	}
	graspMatrix(G, G_inv, R, center_point, contact_locations, contact_types);
}
void Sai2Model::environmentalGraspMatrixInWorld(MatrixXd& G, MatrixXd& G_inv,
												Matrix3d& R,
												const Vector3d center_point) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _environmental_contacts.size(); i++) {
		Vector3d pi;
		positionInWorld(pi, _environmental_contacts[i]._link_name,
						_environmental_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_environmental_contacts[i]._contact_type);
	}
	graspMatrix(G, G_inv, R, center_point, contact_locations, contact_types);
}
void Sai2Model::environmentalGraspMatrixLocalContactForces(
	MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, const Vector3d center_point) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _environmental_contacts.size(); i++) {
		Vector3d pi;
		position(pi, _environmental_contacts[i]._link_name,
				 _environmental_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_environmental_contacts[i]._contact_type);
	}
	graspMatrix(G, G_inv, R, center_point, contact_locations, contact_types);

	int n = _environmental_contacts.size();
	int k = 0;
	for (int i = 0; i < n; i++) {
		if (_environmental_contacts[i]._contact_type == SurfaceContact) {
			k++;
		}
	}

	int k_tmp = 0;
	for (int i = 0; i < n; i++) {
		Matrix3d R_tmp = Matrix3d::Identity();
		rotation(R_tmp, _environmental_contacts[i]._link_name,
				 _environmental_contacts[i]._contact_orientation);

		G.block(0, 3 * i, 3 * (n + k), 3) =
			G.block(0, 3 * i, 3 * (n + k), 3) * R_tmp;
		G_inv.block(3 * i, 0, 3, 3 * (n + k)) =
			R_tmp.transpose() * G_inv.block(3 * i, 0, 3, 3 * (n + k));
		if (_environmental_contacts[i]._contact_type == SurfaceContact) {
			G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
				G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
				R_tmp.transpose() *
				G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));

			if (n > 2) {
				G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
					R_tmp.transpose() *
					G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));
				G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
					G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			}

			k_tmp++;
		}
	}
}
void Sai2Model::environmentalGraspMatrixLocalContactForcesToWorld(
	MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, const Vector3d center_point) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _environmental_contacts.size(); i++) {
		Vector3d pi;
		positionInWorld(pi, _environmental_contacts[i]._link_name,
						_environmental_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_environmental_contacts[i]._contact_type);
	}
	graspMatrix(G, G_inv, R, center_point, contact_locations, contact_types);

	int n = _environmental_contacts.size();
	int k = 0;
	for (int i = 0; i < n; i++) {
		if (_environmental_contacts[i]._contact_type == SurfaceContact) {
			k++;
		}
	}

	int k_tmp = 0;
	for (int i = 0; i < n; i++) {
		Matrix3d R_tmp = Matrix3d::Identity();
		rotationInWorld(R_tmp, _environmental_contacts[i]._link_name,
						_environmental_contacts[i]._contact_orientation);

		G.block(0, 3 * i, 3 * (n + k), 3) =
			G.block(0, 3 * i, 3 * (n + k), 3) * R_tmp;
		G_inv.block(3 * i, 0, 3, 3 * (n + k)) =
			R_tmp.transpose() * G_inv.block(3 * i, 0, 3, 3 * (n + k));
		if (_environmental_contacts[i]._contact_type == SurfaceContact) {
			G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
				G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
				R_tmp.transpose() *
				G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));

			if (n > 2) {
				G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
					R_tmp.transpose() *
					G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));
				G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
					G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			}

			k_tmp++;
		}
	}
}

void Sai2Model::environmentalGraspMatrixAtGeometricCenter(
	MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, Vector3d& geometric_center) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _environmental_contacts.size(); i++) {
		Vector3d pi;
		position(pi, _environmental_contacts[i]._link_name,
				 _environmental_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_environmental_contacts[i]._contact_type);
	}
	graspMatrixAtGeometricCenter(G, G_inv, R, geometric_center,
								 contact_locations, contact_types);
}
void Sai2Model::environmentalGraspMatrixAtGeometricCenterInWorld(
	MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, Vector3d& geometric_center) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _environmental_contacts.size(); i++) {
		Vector3d pi;
		positionInWorld(pi, _environmental_contacts[i]._link_name,
						_environmental_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_environmental_contacts[i]._contact_type);
	}
	graspMatrixAtGeometricCenter(G, G_inv, R, geometric_center,
								 contact_locations, contact_types);
}

void Sai2Model::environmentalGraspMatrixAtGeometricCenterLocalContactForces(
	MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, Vector3d& geometric_center) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _environmental_contacts.size(); i++) {
		Vector3d pi;
		position(pi, _environmental_contacts[i]._link_name,
				 _environmental_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_environmental_contacts[i]._contact_type);
	}

	graspMatrixAtGeometricCenter(G, G_inv, R, geometric_center,
								 contact_locations, contact_types);

	int n = _environmental_contacts.size();
	int k = 0;
	for (int i = 0; i < n; i++) {
		if (_environmental_contacts[i]._contact_type == SurfaceContact) {
			k++;
		}
	}

	int k_tmp = 0;
	for (int i = 0; i < n; i++) {
		Matrix3d R_tmp = Matrix3d::Identity();
		rotation(R_tmp, _environmental_contacts[i]._link_name,
				 _environmental_contacts[i]._contact_orientation);

		G.block(0, 3 * i, 3 * (n + k), 3) =
			G.block(0, 3 * i, 3 * (n + k), 3) * R_tmp;
		G_inv.block(3 * i, 0, 3, 3 * (n + k)) =
			R_tmp.transpose() * G_inv.block(3 * i, 0, 3, 3 * (n + k));
		if (_environmental_contacts[i]._contact_type == SurfaceContact) {
			G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
				G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
				R_tmp.transpose() *
				G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));

			if (n > 2) {
				G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
					R_tmp.transpose() *
					G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));
				G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
					G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			}

			k_tmp++;
		}
	}
}
void Sai2Model::
	environmentalGraspMatrixAtGeometricCenterLocalContactForcesToWorld(
		MatrixXd& G, MatrixXd& G_inv, Matrix3d& R, Vector3d& geometric_center) {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	for (int i = 0; i < _environmental_contacts.size(); i++) {
		Vector3d pi;
		positionInWorld(pi, _environmental_contacts[i]._link_name,
						_environmental_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		contact_types.push_back(_environmental_contacts[i]._contact_type);
	}
	graspMatrixAtGeometricCenter(G, G_inv, R, geometric_center,
								 contact_locations, contact_types);

	int n = _environmental_contacts.size();
	int k = 0;
	for (int i = 0; i < n; i++) {
		if (_environmental_contacts[i]._contact_type == SurfaceContact) {
			k++;
		}
	}

	int k_tmp = 0;
	for (int i = 0; i < n; i++) {
		Matrix3d R_tmp = Matrix3d::Identity();
		rotationInWorld(R_tmp, _environmental_contacts[i]._link_name,
						_environmental_contacts[i]._contact_orientation);

		G.block(0, 3 * i, 3 * (n + k), 3) =
			G.block(0, 3 * i, 3 * (n + k), 3) * R_tmp;
		G_inv.block(3 * i, 0, 3, 3 * (n + k)) =
			R_tmp.transpose() * G_inv.block(3 * i, 0, 3, 3 * (n + k));
		if (_environmental_contacts[i]._contact_type == SurfaceContact) {
			G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
				G.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
				R_tmp.transpose() *
				G_inv.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));

			if (n > 2) {
				G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k)) =
					R_tmp.transpose() *
					G.block(3 * (n + k_tmp), 0, 3, 3 * (n + k));
				G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) =
					G_inv.block(0, 3 * (n + k_tmp), 3 * (n + k), 3) * R_tmp;
			}

			k_tmp++;
		}
	}
}

void Sai2Model::Sai2Model::displayJoints() {
	cout << "\nRobot Joints :" << endl;
	for (map<string, int>::iterator it = _joint_names_to_id_map.begin();
		 it != _joint_names_to_id_map.end(); ++it) {
		cout << "joint : " << it->first << "\t id : " << it->second << endl;
	}
	cout << endl;
}

void Sai2Model::displayLinks() {
	cout << "\nRobot Links :" << endl;
	for (map<string, int>::iterator it = _link_names_to_id_map.begin();
		 it != _link_names_to_id_map.end(); ++it) {
		cout << "link : " << it->first << "\t id : " << it->second << endl;
	}
	cout << endl;
}

void Sai2Model::updateDynamics() {
	if (_M.rows() != _dof || _M.cols() != _dof) {
		_M.setZero(_dof, _dof);
	}

	CompositeRigidBodyAlgorithm(*_rbdl_model, _q, _M, false);
	updateInverseInertia();
}

void Sai2Model::updateInverseInertia() { _M_inv = _M.inverse(); }

void Sai2Model::modifiedNewtonEuler(VectorXd& u, const bool consider_gravity,
									const VectorXd& q, const VectorXd& dq,
									const VectorXd& dqa, const VectorXd& ddq) {
	u = VectorXd::Zero(_dof);

	vector<Vector3d> w, wa, dw, ddO, ddB, f, tau;
	vector<Vector3d> ripi_list, rib_list, z_list;
	Vector3d w_i, wa_i, dw_i, ddO_i, ddB_i, f_i, tau_i;
	Vector3d wp, wap, dwp, ddOp, ddBp, fc, tauc;
	Vector3d z, r_ipi, r_ipb, r_ib;

	// initial conditions forward recursion
	w_i.setZero();
	wa_i.setZero();
	dw_i.setZero();
	if (consider_gravity) {
		ddO_i = -_T_world_robot.linear().transpose() * _rbdl_model->gravity;
	} else {
		ddO_i.setZero();
	}
	ddB_i = ddO_i;

	f_i.setZero();
	tau_i.setZero();

	z.setZero();
	r_ipi.setZero();
	r_ib.setZero();

	w.push_back(w_i);
	wa.push_back(wa_i);
	dw.push_back(dw_i);
	ddO.push_back(ddO_i);
	ddB.push_back(ddB_i);

	z_list.push_back(z);
	rib_list.push_back(r_ib);
	ripi_list.push_back(r_ipi);

	f.push_back(f_i);
	tau.push_back(tau_i);

	for (int i = 1; i < _dof + 1; i++) {
		int parent = _rbdl_model->lambda_q[i];
		vector<unsigned int> children = _rbdl_model->mu[i];
		int child;
		if (children.empty()) {
			child = i;
			r_ipi.setZero();
		} else if (children.size() == 1) {
			child = _rbdl_model->mu[i][0];
			r_ipi = _rbdl_model->X_lambda[child].r;
		} else {
			throw("tree structures not implemented yet");
		}
		z = _rbdl_model->mJoints[i].mJointAxes->head(3);
		r_ipb = _rbdl_model->mBodies[i].mCenterOfMass;
		r_ib = -r_ipi + r_ipb;

		// transform parent quantities in local frame
		wp = _rbdl_model->X_lambda[i].E * w[parent];
		wap = _rbdl_model->X_lambda[i].E * wa[parent];
		dwp = _rbdl_model->X_lambda[i].E * dw[parent];
		ddOp = _rbdl_model->X_lambda[i].E * ddO[parent];

		w_i = wp + dq(parent) * z;
		wa_i = wap + dqa(parent) * z;
		dw_i = dwp + ddq(parent) * z + dq(parent) * wap.cross(z);
		ddO_i = ddOp + dw_i.cross(r_ipi) + w_i.cross(wa_i.cross(r_ipi));
		ddB_i = ddO_i + dw_i.cross(r_ib) + w_i.cross(wa_i.cross(r_ib));

		w.push_back(w_i);
		wa.push_back(wa_i);
		dw.push_back(dw_i);
		ddO.push_back(ddO_i);
		ddB.push_back(ddB_i);

		z_list.push_back(z);
		rib_list.push_back(r_ib);
		ripi_list.push_back(r_ipi);

		f.push_back(f_i);
		tau.push_back(tau_i);
	}

	// backward recursion
	for (int i = _dof; i > 0; i--) {
		Vector3d fip, tauip;
		vector<unsigned int> children = _rbdl_model->mu[i];
		if (children.size() == 0) {
			fip.setZero();
			tauip.setZero();
		} else if (children.size() == 1) {
			int child = children[0];
			fip = _rbdl_model->X_lambda[child].E.transpose() * f[child];
			tauip = _rbdl_model->X_lambda[child].E.transpose() * tau[child];
		} else {
			throw("tree structures not implemented yet");
		}

		double m = _rbdl_model->mBodies[i].mMass;
		Matrix3d I = _rbdl_model->mBodies[i].mInertia;

		f_i = fip + m * ddB[i];
		tau_i = tauip - f_i.cross(ripi_list[i] + rib_list[i]) +
				fip.cross(rib_list[i]) + I * dw[i] + wa[i].cross(I * w[i]);

		Vector3d zp = z_list[i];
		u(i - 1) = tau_i.dot(zp);

		f[i] = f_i;
		tau[i] = tau_i;
	}
}

void orientationError(Vector3d& delta_phi, const Matrix3d& desired_orientation,
					  const Matrix3d& current_orientation) {
	// check that the matrices are valid rotations
	Matrix3d Q1 = desired_orientation * desired_orientation.transpose() -
				  Matrix3d::Identity();
	Matrix3d Q2 = current_orientation * current_orientation.transpose() -
				  Matrix3d::Identity();
	if (Q1.norm() > 0.0001 || Q2.norm() > 0.0001) {
		cout << "des orientation:\n" << desired_orientation << endl;
		cout << "cur orientation:\n" << current_orientation << endl;
		cout << "Q1: " << Q1.norm() << endl;
		cout << "Q2: " << Q2.norm() << endl;
		throw invalid_argument(
			"Invalid rotation matrices in Sai2Model::orientationError");
		return;
	} else {
		Vector3d rc1 = current_orientation.block<3, 1>(0, 0);
		Vector3d rc2 = current_orientation.block<3, 1>(0, 1);
		Vector3d rc3 = current_orientation.block<3, 1>(0, 2);
		Vector3d rd1 = desired_orientation.block<3, 1>(0, 0);
		Vector3d rd2 = desired_orientation.block<3, 1>(0, 1);
		Vector3d rd3 = desired_orientation.block<3, 1>(0, 2);
		delta_phi =
			-1.0 / 2.0 * (rc1.cross(rd1) + rc2.cross(rd2) + rc3.cross(rd3));
	}
}

void orientationError(Vector3d& delta_phi,
					  const Quaterniond& desired_orientation,
					  const Quaterniond& current_orientation) {
	Quaterniond inv_dlambda =
		desired_orientation * current_orientation.conjugate();
	delta_phi = -2.0 * inv_dlambda.vec();
}

Matrix3d CrossProductOperator(const Vector3d& v) {
	Matrix3d v_hat;
	v_hat << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
	return v_hat;
}

void graspMatrix(MatrixXd& G, MatrixXd& G_inv, Matrix3d& R,
				 const Vector3d center_point,
				 const vector<Vector3d>& contact_locations,
				 const vector<ContactType> contact_types) {
	// compute grasp matrix at geometric center
	Vector3d geom_center;
	graspMatrixAtGeometricCenter(G, G_inv, R, geom_center, contact_locations,
								 contact_types);

	// transform the resultant forces and moments to the desired resolving point
	Vector3d r_cg = geom_center - center_point;
	Matrix3d Rcross_cg = CrossProductOperator(r_cg);

	int n = G.rows();

	G.block(3, 0, 3, n) += Rcross_cg * G.block(0, 0, 3, n);
	G_inv.block(0, 0, n, 3) -= G_inv.block(0, 3, n, 3) * Rcross_cg;
}

void graspMatrixAtGeometricCenter(MatrixXd& G, MatrixXd& G_inv, Matrix3d& R,
								  Vector3d& geometric_center,
								  const vector<Vector3d>& contact_locations,
								  const vector<ContactType> contact_types) {
	G = MatrixXd::Zero(1, 1);
	G_inv = MatrixXd::Zero(1, 1);
	R = Matrix3d::Identity();
	geometric_center.setZero();

	// number of contact points
	int n = contact_locations.size();
	if (n < 2) {
		throw invalid_argument(
			"invalid number of contact points (2 points min) in "
			"Sai2Model::graspMatrixAtGeometricCenter\n");
	}
	if (n > 4) {
		throw invalid_argument(
			"invalid number of contact points (4 points max) in "
			"Sai2Model::graspMatrixAtGeometricCenter\n");
	}
	if (contact_types.size() != n) {
		throw invalid_argument(
			"argument contact_locations and contact_types need to be of the "
			"same length in Sai2Model::graspMatrixAtGeometricCenter\n");
	}

	// TODO : support line contact
	// number of surface contacts (that can apply a moment)
	int k = 0;
	for (int i = 0; i < n; i++) {
		if (contact_types[i] == SurfaceContact) {
			k++;
		}
	}

	// find geometric center
	for (int i = 0; i < n; i++) {
		geometric_center += contact_locations[i] / n;
	}

	// prepare Wf and Wm matrices
	MatrixXd Wf = MatrixXd::Zero(6, 3 * n);
	MatrixXd Wm = MatrixXd::Zero(6, 3 * k);

	for (int i = 0; i < n; i++) {
		Vector3d ri = contact_locations[i] - geometric_center;
		Wf.block<3, 3>(0, 3 * i) = Matrix3d::Identity();
		Wf.block<3, 3>(3, 3 * i) = CrossProductOperator(ri);
	}
	for (int i = 0; i < k; i++) {
		Wm.block<3, 3>(3, 3 * i) = Matrix3d::Identity();
	}

	// prepare E and I
	MatrixXd E, I;

	Vector3d x, y, z;

	switch (n) {
		case 2: {
			// resize E
			E = MatrixXd::Zero(6, 1);

			// compute the point to point vectors
			Vector3d e12 = contact_locations[1] - contact_locations[0];
			double l = e12.norm();
			e12.normalize();

			// fill in E matrix
			E.block<3, 1>(0, 0) = -e12;
			E.block<3, 1>(3, 0) = e12;

			// create Ebar
			MatrixXd Ebar = 0.5 * E.transpose();

			// find R
			x = e12;
			if (abs((x.cross(Vector3d(1, 0, 0))).norm()) <
				1e-3)  // local x is aligned with world x
			{
				if (x.dot(Vector3d(1, 0, 0)) > 0)  // same direction
				{
					R = Matrix3d::Identity();
					// cout << "R is identity" << endl;
				} else	// rotation around Z axis by 180 degrees
				{
					R << -1, 0, 0, 0, -1, 0, 0, 0, 1;
				}
				x = R.col(0);
				y = R.col(1);
				z = R.col(2);
			} else {
				y = x.cross(Vector3d(1, 0, 0));
				y.normalize();
				z = x.cross(y);
				z.normalize();
				R.block<3, 1>(0, 0) = x;
				R.block<3, 1>(0, 1) = y;
				R.block<3, 1>(0, 2) = z;
			}

			switch (k) {
				case 0: {
					// throw runtime_error("Case 2-0 not implemented yet\n");
					G.setZero(6, 6);
					G.block<3, 3>(0, 0) = Matrix3d::Identity();
					G.block<3, 3>(0, 3) = Matrix3d::Identity();
					G.block<1, 3>(3, 0) = l / 2.0 * z.transpose();
					G.block<1, 3>(3, 3) = -l / 2.0 * z.transpose();
					G.block<1, 3>(4, 0) = -l / 2.0 * y.transpose();
					G.block<1, 3>(4, 3) = l / 2.0 * y.transpose();
					G.block<1, 3>(5, 0) = -1.0 / 2.0 * x.transpose();
					G.block<1, 3>(5, 3) = 1.0 / 2.0 * x.transpose();

					G_inv.setZero(6, 6);
					G_inv.block<3, 3>(0, 0) = 0.5 * Matrix3d::Identity();
					G_inv.block<3, 3>(3, 0) = 0.5 * Matrix3d::Identity();
					G_inv.block<3, 1>(0, 3) = l * z;
					G_inv.block<3, 1>(3, 3) = -l * z;
					G_inv.block<3, 1>(0, 4) = -l * y;
					G_inv.block<3, 1>(3, 4) = l * y;
					G_inv.block<3, 1>(0, 5) = x;
					G_inv.block<3, 1>(3, 5) = x;

					break;
				}
				case 1: {
					// only 2 internal moments
					I = MatrixXd::Zero(2, 3);

					I << 0, 1, 0, 0, 0, 1;
					I = I * R.transpose();

					// populate G
					G = MatrixXd::Zero(9, 9);
					G.block<6, 6>(0, 0) = Wf;
					G.block<6, 3>(0, 6) = Wm;
					G.block<1, 6>(6, 0) = Ebar;
					G.block<2, 3>(7, 6) = I;

					// TODO : find explicit form for this case
					G_inv = MatrixXd::Zero(9, 9);
					G_inv = G.inverse();

					break;
				}
				case 2: {
					I = MatrixXd::Zero(5, 6);

					// find I
					I << -0.5, 0, 0, 0.5, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
						0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
					// I = I*Rr.transpose();
					I.block<5, 3>(0, 0) = I.block<5, 3>(0, 0) * R.transpose();
					I.block<5, 3>(0, 3) = I.block<5, 3>(0, 3) * R.transpose();

					// populate G
					G = MatrixXd::Zero(12, 12);
					G.block<6, 6>(0, 0) = Wf;
					G.block<6, 6>(0, 6) = Wm;
					G.block<1, 6>(6, 0) = Ebar;
					G.block<5, 6>(7, 6) = I;

					G_inv = MatrixXd::Zero(12, 12);
					Matrix3d rx_cross = CrossProductOperator(x);
					Matrix3d rx_cross_square = rx_cross * rx_cross;
					G_inv.block<3, 3>(0, 0) = 0.5 * Matrix3d::Identity();
					G_inv.block<3, 3>(3, 0) = 0.5 * Matrix3d::Identity();
					G_inv.block<3, 3>(0, 3) = rx_cross / l;
					G_inv.block<3, 3>(3, 3) = -rx_cross / l;
					G_inv.block<3, 3>(6, 3) =
						(Matrix3d::Identity() + rx_cross_square) / 2.0;
					G_inv.block<3, 3>(9, 3) =
						(Matrix3d::Identity() + rx_cross_square) / 2.0;

					G_inv.block<3, 1>(0, 6) = -x;
					G_inv.block<3, 1>(3, 6) = x;

					G_inv.block<3, 1>(6, 7) = -x;
					G_inv.block<3, 1>(9, 7) = x;

					G_inv.block<3, 1>(0, 8) = -z / l;
					G_inv.block<3, 1>(3, 8) = z / l;
					G_inv.block<3, 1>(6, 8) = y;

					G_inv.block<3, 1>(0, 9) = y / l;
					G_inv.block<3, 1>(3, 9) = -y / l;
					G_inv.block<3, 1>(6, 9) = z;

					G_inv.block<3, 1>(0, 10) = -z / l;
					G_inv.block<3, 1>(3, 10) = z / l;
					G_inv.block<3, 1>(9, 10) = y;

					G_inv.block<3, 1>(0, 11) = y / l;
					G_inv.block<3, 1>(3, 11) = -y / l;
					G_inv.block<3, 1>(9, 11) = z;

					break;
				}
				default:
					throw runtime_error(
						"Should not arrive here (number of contact points is "
						"2, number of surface contacts incoherent) in "
						"Sai2Model::graspMatrixAtGeometricCenter\n");
			}
			break;
		}

		case 3: {
			// compute Wf_bar
			Matrix3d bot_right_Wf_WfT =
				Wf.block<3, 9>(3, 0) * Wf.block<3, 9>(3, 0).transpose();
			MatrixXd WfWfT_inv = MatrixXd::Identity(6, 6) / 3.0;
			WfWfT_inv.block<3, 3>(3, 3) = bot_right_Wf_WfT.inverse();

			MatrixXd Wf_bar = Wf.transpose() * WfWfT_inv;

			// resize E
			E = MatrixXd::Zero(9, 3);

			// compute the point to point vectors
			Vector3d e12 = contact_locations[1] - contact_locations[0];
			Vector3d e13 = contact_locations[2] - contact_locations[0];
			Vector3d e23 = contact_locations[2] - contact_locations[1];

			e12.normalize();
			e13.normalize();
			e23.normalize();

			// fill in E matrix
			E.block<3, 1>(0, 0) = -e12;
			E.block<3, 1>(3, 0) = e12;
			E.block<3, 1>(0, 1) = -e13;
			E.block<3, 1>(6, 1) = e13;
			E.block<3, 1>(3, 2) = -e23;
			E.block<3, 1>(6, 2) = e23;

			// create Ebar
			MatrixXd Ebar = (E.transpose() * E).inverse() * E.transpose();

			if (k < 0 || k > 3) {
				throw runtime_error(
					"Should not happen (number of contact points is 3, number "
					"of surface contacts incoherent) in "
					"Sai2Model::graspMatrixAtGeometricCenter\n");
			} else if (k == 0) {
				// populate G
				G = MatrixXd::Zero(9, 9);
				G.block<6, 9>(0, 0) = Wf;
				G.block<3, 9>(6, 0) = Ebar;

				G_inv.setZero(9, 9);
				G_inv.block<9, 6>(0, 0) = Wf_bar;
				G_inv.block<9, 3>(0, 6) = E;
			} else {
				int tk = 3 * k;
				// compute I
				MatrixXd I = MatrixXd::Identity(tk, tk);

				// populate G
				G = MatrixXd::Zero(9 + tk, 9 + tk);
				G.block(0, 0, 6, 9) = Wf;
				G.block(0, 9, 6, tk) = Wm;
				G.block(6, 0, 3, 9) = Ebar;
				G.block(9, 9, tk, tk) = I;

				G_inv.setZero(9 + tk, 9 + tk);
				G_inv.block(0, 0, 9, 6) = Wf_bar;
				G_inv.block(0, 6, 9, 3) = E;
				G_inv.block(0, 9, 9, tk) = -Wf_bar * Wm;
				G_inv.block(9, 9, tk, tk) = I;
			}
			break;
		}

		case 4: {
			// compute Wf_bar
			Matrix3d bot_right_Wf_WfT =
				Wf.block<3, 12>(3, 0) * Wf.block<3, 12>(3, 0).transpose();
			MatrixXd WfWfT_inv = MatrixXd::Identity(6, 6) / 4.0;
			WfWfT_inv.block<3, 3>(3, 3) = bot_right_Wf_WfT.inverse();

			MatrixXd Wf_bar = Wf.transpose() * WfWfT_inv;

			// resize E
			E = MatrixXd::Zero(12, 6);

			// compute the point to point vectors
			Vector3d e12 = contact_locations[1] - contact_locations[0];
			Vector3d e13 = contact_locations[2] - contact_locations[0];
			Vector3d e14 = contact_locations[3] - contact_locations[0];
			Vector3d e23 = contact_locations[2] - contact_locations[1];
			Vector3d e24 = contact_locations[3] - contact_locations[1];
			Vector3d e34 = contact_locations[3] - contact_locations[2];

			e12.normalize();
			e13.normalize();
			e14.normalize();
			e23.normalize();
			e24.normalize();
			e34.normalize();

			// fill in E matrix
			E.block<3, 1>(0, 0) = -e12;
			E.block<3, 1>(3, 0) = e12;
			E.block<3, 1>(0, 1) = -e13;
			E.block<3, 1>(6, 1) = e13;
			E.block<3, 1>(0, 2) = -e14;
			E.block<3, 1>(9, 2) = e14;
			E.block<3, 1>(3, 3) = -e23;
			E.block<3, 1>(6, 3) = e23;
			E.block<3, 1>(3, 4) = -e24;
			E.block<3, 1>(9, 4) = e24;
			E.block<3, 1>(6, 5) = -e34;
			E.block<3, 1>(9, 5) = e34;

			// create Ebar
			MatrixXd Ebar = (E.transpose() * E).inverse() * E.transpose();

			if (k < 0 || k > 4) {
				throw runtime_error(
					"Should not arrive here (number of contact points is 4, "
					"number of surface contacts incoherent) in "
					"Sai2Model::graspMatrixAtGeometricCenter\n");
			} else if (k == 0) {
				// populate G
				G = MatrixXd::Zero(12, 12);
				G.block<6, 12>(0, 0) = Wf;
				G.block<6, 12>(6, 0) = Ebar;

				G_inv.setZero(12, 12);
				G_inv.block<12, 6>(0, 0) = Wf_bar;
				G_inv.block<12, 6>(0, 6) = E;
			} else {
				int tk = 3 * k;

				// compute I
				MatrixXd I = MatrixXd::Identity(tk, tk);

				// populate G
				G = MatrixXd::Zero(12 + tk, 12 + tk);
				G.block(0, 0, 6, 12) = Wf;
				G.block(0, 12, 6, tk) = Wm;
				G.block(6, 0, 6, 12) = Ebar;
				G.block(12, 12, tk, tk) = I;

				G_inv.setZero(12 + tk, 12 + tk);
				G_inv.block(0, 0, 12, 6) = Wf_bar;
				G_inv.block(0, 6, 12, 6) = E;
				G_inv.block(0, 12, 12, tk) = -Wf_bar * Wm;
				G_inv.block(12, 12, tk, tk) = I;
			}
			break;
		}

		default:
			throw runtime_error(
				"Should not arrive here (number of contact points is not 2, 3 "
				"or 4) in Sai2Model::graspMatrixAtGeometricCenter \n");
	}
}

}  // namespace Sai2Model

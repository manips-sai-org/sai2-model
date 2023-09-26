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

bool isValidQuaternion(double x, double y, double z, double w) {
	Eigen::Quaterniond quaternion(w, x, y, z);

	if (std::abs(quaternion.squaredNorm() - 1.0) > 1e-3) {
		return false;
	}

	return true;
}

bool isPositiveDefinite(const MatrixXd& matrix) {
	// square
	if (matrix.rows() != matrix.cols()) return false;

	// symmetric
	if (!matrix.transpose().isApprox(matrix)) return false;

	// positive eigenvalues
	SelfAdjointEigenSolver<MatrixXd> eigensolver(matrix);
	const auto& eigenvalues = eigensolver.eigenvalues();

	for (int i = 0; i < eigenvalues.size(); ++i) {
		if (eigenvalues(i) <= 0) return false;
	}

	return true;
}

}  // namespace

namespace Sai2Model {

Sai2Model::Sai2Model(const string path_to_model_file, bool verbose) {
	_rbdl_model = new RigidBodyDynamics::Model();

	// parse rbdl model from urdf
	bool success = RigidBodyDynamics::URDFReadFromFile(
		path_to_model_file.c_str(), _rbdl_model, _link_names_to_id_map,
		_joint_names_to_id_map, _joint_limits, false, verbose);
	if (!success) {
		throw std::runtime_error("Error loading model [" + path_to_model_file +
								 "]\n");
	}

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
	_T_world_robot = Affine3d::Identity();

	// set the gravity in rbdl model
	// it is the world gravity expressed in robot base frame
	_rbdl_model->gravity = Vector3d(0, 0, -9.81);

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
				SphericalJointDescription(jointName(index), index, w_index));
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

void Sai2Model::setQ(const Eigen::VectorXd& q) {
	if (q.size() != _q_size) {
		throw invalid_argument("q size inconsistent in Sai2Model::setQ");
		return;
	}
	for (const auto& sph_joint : _spherical_joints) {
		if (!isValidQuaternion(q(sph_joint.index), q(sph_joint.index + 1),
							   q(sph_joint.index + 2), q(sph_joint.w_index))) {
			throw invalid_argument(
				"trying to set an invalid quaternion for joint " +
				sph_joint.name + " at index " +
				std::to_string(sph_joint.index) +
				", and w_index: " + std::to_string(sph_joint.w_index));
			return;
		}
	}
	_q = q;
}

void Sai2Model::setDq(const Eigen::VectorXd& dq) {
	if (dq.size() != _dof) {
		throw invalid_argument("dq size inconsistent in Sai2Model::setDq");
		return;
	}
	_dq = dq;
}

void Sai2Model::setDdq(const Eigen::VectorXd& ddq) {
	if (ddq.size() != _dof) {
		throw invalid_argument("ddq size inconsistent in Sai2Model::setDdq");
		return;
	}
	_ddq = ddq;
}

const Eigen::Quaterniond Sai2Model::sphericalQuat(
	const std::string& joint_name) const {
	for (auto joint : _spherical_joints) {
		if (joint.name == joint_name) {
			int i = joint.index;
			int iw = joint.w_index;
			return Eigen::Quaterniond(_q(iw), _q(i), _q(i + 1), _q(i + 2));
		}
	}
	throw invalid_argument(
		"cannot get the quaternion for non existing spherical joint " +
		joint_name);
}

void Sai2Model::setSphericalQuat(const std::string& joint_name,
								 const Eigen::Quaterniond quat) {
	for (auto joint : _spherical_joints) {
		if (joint.name == joint_name) {
			int i = joint.index;
			int iw = joint.w_index;
			_q(i) = quat.x();
			_q(i + 1) = quat.y();
			_q(i + 2) = quat.z();
			_q(iw) = quat.w();
			return;
		}
	}
	throw invalid_argument(
		"cannot set the quaternion for non existing spherical joint " +
		joint_name);
}

void Sai2Model::setTRobotBase(const Affine3d& T) {
	Vector3d world_gravity = worldGravity();
	_T_world_robot = T;
	// world gravity in robot base frame changes
	_rbdl_model->gravity = _T_world_robot.linear().transpose() * world_gravity;
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

	if (!isPositiveDefinite(M)) {
		throw invalid_argument(
			"M is not symmetric positive definite Sai2Model::updateModel");
	}
	if (M.rows() != _dof) {
		throw invalid_argument(
			"M matrix dimensions inconsistent in Sai2Model::updateModel");
	}
	_M = M;
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

VectorXd Sai2Model::jointGravityVector() {
	VectorXd g = VectorXd::Zero(_dof);

	vector<RigidBodyDynamics::Body>::iterator it_body;
	int body_id;

	for (it_body = _rbdl_model->mBodies.begin(), body_id = 0;
		 it_body != _rbdl_model->mBodies.end(); ++it_body, ++body_id) {
		double mass = it_body->mMass;
		MatrixXd Jv = MatrixXd::Zero(3, _dof);
		CalcPointJacobian(*_rbdl_model, _q, body_id, it_body->mCenterOfMass, Jv,
						  false);

		g += Jv.transpose() * (-mass * _rbdl_model->gravity);
	}
	return g;
}

VectorXd Sai2Model::coriolisForce() {
	// returns v + g. Need to substract the gravity from it
	VectorXd b = VectorXd::Zero(_dof);
	NonlinearEffects(*_rbdl_model, _q, _dq, b);

	return b - jointGravityVector();
}

VectorXd Sai2Model::coriolisPlusGravity() {
	VectorXd h = VectorXd::Zero(_dof);
	NonlinearEffects(*_rbdl_model, _q, _dq, h);
	return h;
}

MatrixXd Sai2Model::factorizedChristoffelMatrix() {
	MatrixXd C = MatrixXd::Zero(_dof, _dof);

	VectorXd vi = VectorXd::Zero(_dof);

	for (int i = 0; i < _dof; i++) {
		vi.setZero();
		vi(i) = 1;
		C.col(i) =
			modifiedNewtonEuler(false, _q, _dq, vi, VectorXd::Zero(_dof));
	}
	return C;
}

MatrixXd Sai2Model::J(const string& link_name,
					  const Vector3d& pos_in_link) const {
	MatrixXd J = MatrixXd::Zero(6, _dof);
	CalcPointJacobian6D(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link, J,
						false);
	// RBDL gives Jw as the top 3 rows and Jv as the bottom part. We need to
	// swap it here
	J.block(0, 0, 3, _dof).swap(J.block(3, 0, 3, _dof));
	return J;
}

MatrixXd Sai2Model::JWorldFrame(const string& link_name,
								const Vector3d& pos_in_link) const {
	MatrixXd J = this->J(link_name, pos_in_link);
	J.topRows<3>() = _T_world_robot.linear() * J.topRows<3>();
	J.bottomRows<3>() = _T_world_robot.linear() * J.bottomRows<3>();
	return J;
}

MatrixXd Sai2Model::JLocalFrame(const string& link_name,
								const Vector3d& pos_in_link,
								const Matrix3d& rot_in_link) const {
	MatrixXd J = this->J(link_name, pos_in_link);
	Matrix3d R_base_ee = rotation(link_name) * rot_in_link;
	J.topRows<3>() = R_base_ee.transpose() * J.topRows<3>();
	J.bottomRows<3>() = R_base_ee.transpose() * J.bottomRows<3>();
	return J;
}

MatrixXd Sai2Model::Jv(const string& link_name,
					   const Vector3d& pos_in_link) const {
	MatrixXd J = MatrixXd::Zero(3, _dof);
	CalcPointJacobian(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link, J,
					  false);
	return J;
}

MatrixXd Sai2Model::JvWorldFrame(const string& link_name,
								 const Vector3d& pos_in_link) const {
	return _T_world_robot.linear() * Jv(link_name, pos_in_link);
}

MatrixXd Sai2Model::JvLocalFrame(const string& link_name,
								 const Vector3d& pos_in_link,
								 const Matrix3d& rot_in_link) const {
	return rotation(link_name, rot_in_link).transpose() *
		   Jv(link_name, pos_in_link);
}

MatrixXd Sai2Model::Jw(const string& link_name) const {
	// compute the full jacobian at the center of the link and take rotational
	// part
	return J(link_name).bottomRows<3>();
}

MatrixXd Sai2Model::JwWorldFrame(const string& link_name) const {
	return _T_world_robot.linear() * Jw(link_name);
}

MatrixXd Sai2Model::JwLocalFrame(const string& link_name,
								 const Matrix3d& rot_in_link) const {
	return rotation(link_name, rot_in_link).transpose() * Jw(link_name);
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

void Sai2Model::computeIK3dJL(
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

	InverseKinematicsJL(*_rbdl_model, _q, q_min, q_max, weights, link_ids,
						body_point, target_pos, q_result, 1.0e-12, 0.0001, 500);
	// InverseKinematicsJL modifies the internal model so we need to re update
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

Affine3d Sai2Model::transform(const string& link_name,
							  const Vector3d& pos_in_link,
							  const Matrix3d& rot_in_link) const {
	unsigned int link_id = linkIdRbdl(link_name);
	Eigen::Affine3d T(
		CalcBodyWorldOrientation(*_rbdl_model, _q, link_id, false).transpose() *
		rot_in_link);
	T.translation() = CalcBodyToBaseCoordinates(*_rbdl_model, _q, link_id,
												pos_in_link, false);
	return T;
}

Affine3d Sai2Model::transformInWorld(const string& link_name,
									 const Vector3d& pos_in_link,
									 const Matrix3d& rot_in_link) const {
	return _T_world_robot * transform(link_name, pos_in_link, rot_in_link);
}

Vector6d Sai2Model::velocity6d(const string link_name,
							   const Vector3d& pos_in_link) const {
	Vector6d vel6d = CalcPointVelocity6D(
		*_rbdl_model, _q, _dq, linkIdRbdl(link_name), pos_in_link, false);
	vel6d.head(3).swap(vel6d.tail(3));
	return vel6d;
}

Vector6d Sai2Model::velocity6dInWorld(const string link_name,
									  const Vector3d& pos_in_link) const {
	Vector6d vel6d = velocity6d(link_name, pos_in_link);
	vel6d.head(3) = _T_world_robot.linear() * vel6d.head(3);
	vel6d.tail(3) = _T_world_robot.linear() * vel6d.tail(3);
	return vel6d;
}

Vector6d Sai2Model::acceleration6d(const string link_name,
								   const Vector3d& pos_in_link) const {
	Vector6d acc6d = CalcPointAcceleration6D(
		*_rbdl_model, _q, _dq, _ddq, linkIdRbdl(link_name), pos_in_link, false);
	acc6d.head(3).swap(acc6d.tail(3));
	return acc6d;
}
Vector6d Sai2Model::acceleration6dInWorld(const string link_name,
										  const Vector3d& pos_in_link) const {
	Vector6d accel6d = acceleration6d(link_name, pos_in_link);
	accel6d.head(3) = _T_world_robot.linear() * accel6d.head(3);
	accel6d.tail(3) = _T_world_robot.linear() * accel6d.tail(3);
	return accel6d;
}

Vector3d Sai2Model::position(const string& link_name,
							 const Vector3d& pos_in_link) const {
	return CalcBodyToBaseCoordinates(*_rbdl_model, _q, linkIdRbdl(link_name),
									 pos_in_link, false);
}

Vector3d Sai2Model::positionInWorld(const string& link_name,
									const Vector3d& pos_in_link) const {
	return _T_world_robot * position(link_name, pos_in_link);
}

Vector3d Sai2Model::linearVelocity(const string& link_name,
								   const Vector3d& pos_in_link) const {
	return CalcPointVelocity(*_rbdl_model, _q, _dq, linkIdRbdl(link_name),
							 pos_in_link, false);
}
Vector3d Sai2Model::linearVelocityInWorld(const string& link_name,
										  const Vector3d& pos_in_link) const {
	return _T_world_robot.linear() * linearVelocity(link_name, pos_in_link);
}

Vector3d Sai2Model::linearAcceleration(const string& link_name,
									   const Vector3d& pos_in_link) const {
	return CalcPointAcceleration(*_rbdl_model, _q, _dq, _ddq,
								 linkIdRbdl(link_name), pos_in_link, false);
}
Vector3d Sai2Model::linearAccelerationInWorld(
	const string& link_name, const Vector3d& pos_in_link) const {
	return _T_world_robot.linear() * linearAcceleration(link_name, pos_in_link);
}

Matrix3d Sai2Model::rotation(const string& link_name,
							 const Matrix3d& rot_in_link) const {
	return CalcBodyWorldOrientation(*_rbdl_model, _q, linkIdRbdl(link_name),
									false)
			   .transpose() *
		   rot_in_link;
}

Matrix3d Sai2Model::rotationInWorld(const string& link_name,
									const Matrix3d& rot_in_link) const {
	return _T_world_robot.linear() * rotation(link_name, rot_in_link);
}

Vector3d Sai2Model::angularVelocity(const string& link_name) const {
	return velocity6d(link_name, Vector3d::Zero()).tail(3);
}

Vector3d Sai2Model::angularVelocityInWorld(const string& link_name) const {
	return _T_world_robot.linear() * angularVelocity(link_name);
}

Vector3d Sai2Model::angularAcceleration(const string& link_name) const {
	return acceleration6d(link_name, Vector3d::Zero()).tail(3);
}
Vector3d Sai2Model::angularAccelerationInWorld(const string& link_name) const {
	return _T_world_robot.linear() * angularAcceleration(link_name);
}

unsigned int Sai2Model::linkIdRbdl(const string& link_name) const {
	if (_link_names_to_id_map.find(link_name) == _link_names_to_id_map.end()) {
		throw invalid_argument("link [" + link_name + "] does not exist");
	}
	return _link_names_to_id_map.at(link_name);
}

int Sai2Model::jointIndex(const string& joint_name) const {
	if (_joint_names_to_id_map.find(joint_name) ==
		_joint_names_to_id_map.end()) {
		throw invalid_argument("joint [" + joint_name + "] does not exist");
	}
	return _rbdl_model->mJoints.at(_joint_names_to_id_map.at(joint_name))
		.q_index;
}

int Sai2Model::sphericalJointIndexW(const string& joint_name) const {
	if (_joint_names_to_id_map.find(joint_name) ==
		_joint_names_to_id_map.end()) {
		throw invalid_argument("joint [" + joint_name + "] does not exist");
	}
	for (auto it = _spherical_joints.cbegin(); it != _spherical_joints.cend();
		 ++it) {
		if (it->name == joint_name) {
			return it->w_index;
		}
	}
	throw invalid_argument("joint [" + joint_name + "] is not spherical");
}

std::string Sai2Model::jointName(const int joint_id) const {
	if (joint_id < 0 || joint_id >= _q_size) {
		throw std::invalid_argument(
			"cannot get joint name for id out of bounds");
	}
	return _joint_id_to_names_map.at(joint_id);
}

std::vector<std::string> Sai2Model::jointNames() const {
	std::vector<std::string> names;
	names.reserve(_joint_names_to_id_map.size());

	for (const auto& pair : _joint_names_to_id_map) {
		names.push_back(pair.first);
	}

	return names;
}

LinkMassParams Sai2Model::getLinkMassParams(const string& link_name) const {
	RigidBodyDynamics::Body b = _rbdl_model->mBodies.at(linkIdRbdl(link_name));
	return LinkMassParams(b.mMass, b.mCenterOfMass, b.mInertia);
}

Vector3d Sai2Model::comPosition() const {
	Vector3d robot_com = Vector3d::Zero();
	double robot_mass = 0.0;
	Vector3d center_of_mass_global_frame;
	int n_bodies = _rbdl_model->mBodies.size();
	for (int i = 0; i < n_bodies; i++) {
		RigidBodyDynamics::Body b = _rbdl_model->mBodies.at(i);

		center_of_mass_global_frame = CalcBodyToBaseCoordinates(
			*_rbdl_model, _q, i, b.mCenterOfMass, false);

		robot_com += center_of_mass_global_frame * b.mMass;
		robot_mass += b.mMass;
	}
	return robot_com / robot_mass;
}

MatrixXd Sai2Model::comJacobian() const {
	MatrixXd Jv_com = MatrixXd::Zero(3, _dof);
	MatrixXd link_Jv;
	double robot_mass = 0.0;
	int n_bodies = _rbdl_model->mBodies.size();
	for (int i = 0; i < n_bodies; i++) {
		RigidBodyDynamics::Body b = _rbdl_model->mBodies[i];

		link_Jv.setZero(3, _dof);
		CalcPointJacobian(*_rbdl_model, _q, i, b.mCenterOfMass, link_Jv, false);

		Jv_com += link_Jv * b.mMass;
		robot_mass += b.mMass;
	}
	return Jv_com / robot_mass;
}

Eigen::MatrixXd Sai2Model::taskInertiaMatrix(
	const MatrixXd& task_jacobian) const {
	// check the task jacobian is compatible with the robot model
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"Sai2Model::taksInertiaMatrix");
	}

	// resize Matrices
	int k = task_jacobian.rows();

	// compute task inertia
	MatrixXd inv_inertia = task_jacobian * _M_inv * task_jacobian.transpose();
	return inv_inertia.llt().solve(MatrixXd::Identity(k, k));
}

MatrixXd Sai2Model::taskInertiaMatrixWithPseudoInv(
	const MatrixXd& task_jacobian) const {
	// check the task jacobian is compatible with the robot model
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"Sai2Model::taksInertiaMatrixWithPseudoInv");
	}

	// resize Matrices
	int k = task_jacobian.rows();

	// compute task inertia
	MatrixXd inv_inertia = task_jacobian * _M_inv * task_jacobian.transpose();

	// compute SVD pseudoinverse
	// TODO: make class function?
	JacobiSVD<MatrixXd> svd(inv_inertia, ComputeThinU | ComputeThinV);
	const double epsilon = numeric_limits<double>::epsilon();
	double tolerance = epsilon * max(inv_inertia.cols(), inv_inertia.rows()) *
					   svd.singularValues().array().abs()(0);
	return svd.matrixV() *
		   (svd.singularValues().array().abs() > tolerance)
			   .select(svd.singularValues().array().inverse(), 0)
			   .matrix()
			   .asDiagonal() *
		   svd.matrixU().adjoint();
}

MatrixXd Sai2Model::dynConsistentInverseJacobian(
	const MatrixXd& task_jacobian) const {
	// check the task jacobian is compatible with the robot model
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"Sai2Model::dynConsistentInverseJacobian");
	}
	return _M_inv * task_jacobian.transpose() *
		   taskInertiaMatrix(task_jacobian);
}

MatrixXd Sai2Model::nullspaceMatrix(const MatrixXd& task_jacobian) const {
	// check matrices dimmnsions
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"jacobian matrix dimensions inconsistent with model dof in "
			"Sai2Model::nullspaceMatrix");
	}

	MatrixXd Jbar = dynConsistentInverseJacobian(task_jacobian);
	return (MatrixXd::Identity(_dof, _dof) - Jbar * task_jacobian);
}

OpSpaceMatrices Sai2Model::operationalSpaceMatrices(
	const MatrixXd& task_jacobian) const {
	// check matrices have the right size
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"Sai2Model::operationalSpaceMatrices");
	}

	// Compute the matrices
	MatrixXd Lambda = taskInertiaMatrix(task_jacobian);
	MatrixXd Jbar = _M_inv * task_jacobian.transpose() * Lambda;
	MatrixXd N =
		(MatrixXd::Identity(_dof, _dof) - Jbar * task_jacobian);
	return OpSpaceMatrices(task_jacobian, Lambda, Jbar, N);
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

GraspMatrixData Sai2Model::manipulationGraspMatrix(
	const Vector3d& desired_resultant_point,
	const bool resultant_in_world_frame,
	const bool contact_forces_in_local_frames) const {
	GraspMatrixData G_data = manipulationGraspMatrixAtGeometricCenter(
		resultant_in_world_frame, contact_forces_in_local_frames);

	Vector3d r_cg = G_data.resultant_point - desired_resultant_point;
	Matrix3d Rcross_cg = crossProductOperator(r_cg);

	int n = G_data.G.cols();

	G_data.G.block(3, 0, 3, n) += Rcross_cg * G_data.G.block(0, 0, 3, n);
	G_data.G_inv.block(0, 0, n, 3) -=
		G_data.G_inv.block(0, 3, n, 3) * Rcross_cg;

	G_data.resultant_point = desired_resultant_point;

	return G_data;
}

GraspMatrixData Sai2Model::manipulationGraspMatrixAtGeometricCenter(
	const bool resultant_in_world_frame,
	const bool contact_forces_in_local_frames) const {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	int num_surface_contacts = 0;
	for (const auto& contact : _manipulation_contacts) {
		if (resultant_in_world_frame) {
			contact_locations.push_back(
				positionInWorld(contact._link_name, contact._contact_position));

		} else {
			contact_locations.push_back(
				position(contact._link_name, contact._contact_position));
		}
		contact_types.push_back(contact._contact_type);
		if (contact._contact_type == SurfaceContact) {
			num_surface_contacts++;
		}
	}
	GraspMatrixData G_data =
		graspMatrixAtGeometricCenter(contact_locations, contact_types);

	if (contact_forces_in_local_frames) {
		int n_contact_points = _manipulation_contacts.size();
		int current_surface_contact = 0;
		for (int i = 0; i < n_contact_points; i++) {
			Matrix3d R_local;
			if (resultant_in_world_frame) {
				R_local = rotationInWorld(
					_manipulation_contacts[i]._link_name,
					_manipulation_contacts[i]._contact_orientation);
			} else {
				R_local =
					rotation(_manipulation_contacts[i]._link_name,
							 _manipulation_contacts[i]._contact_orientation);
			}

			G_data.G
				.block(0, 3 * i, 3 * (n_contact_points + num_surface_contacts),
					   3)
				.applyOnTheRight(R_local);
			G_data.G_inv
				.block(3 * i, 0, 3,
					   3 * (n_contact_points + num_surface_contacts))
				.applyOnTheLeft(R_local.transpose());

			if (_manipulation_contacts[i]._contact_type == SurfaceContact) {
				G_data.G
					.block(0, 3 * (n_contact_points + current_surface_contact),
						   3 * (n_contact_points + num_surface_contacts), 3)
					.applyOnTheRight(R_local);
				G_data.G_inv
					.block(3 * (n_contact_points + current_surface_contact), 0,
						   3, 3 * (n_contact_points + num_surface_contacts))
					.applyOnTheLeft(R_local.transpose());

				if (n_contact_points > 2) {
					G_data.G
						.block(3 * (n_contact_points + current_surface_contact),
							   0, 3,
							   3 * (n_contact_points + num_surface_contacts))
						.applyOnTheLeft(R_local.transpose());
					G_data.G_inv
						.block(0,
							   3 * (n_contact_points + current_surface_contact),
							   3 * (n_contact_points + num_surface_contacts), 3)
						.applyOnTheRight(R_local);
				}

				current_surface_contact++;
			}
		}
	}
	return G_data;
}

GraspMatrixData Sai2Model::environmentalGraspMatrix(
	const Vector3d& desired_resultant_point,
	const bool resultant_in_world_frame,
	const bool contact_forces_in_local_frames) const {
	GraspMatrixData G_data = environmentalGraspMatrixAtGeometricCenter(
		resultant_in_world_frame, contact_forces_in_local_frames);

	Vector3d r_cg = G_data.resultant_point - desired_resultant_point;
	Matrix3d Rcross_cg = crossProductOperator(r_cg);

	int n = G_data.G.cols();

	G_data.G.block(3, 0, 3, n) += Rcross_cg * G_data.G.block(0, 0, 3, n);
	G_data.G_inv.block(0, 0, n, 3) -=
		G_data.G_inv.block(0, 3, n, 3) * Rcross_cg;

	G_data.resultant_point = desired_resultant_point;

	return G_data;
}

GraspMatrixData Sai2Model::environmentalGraspMatrixAtGeometricCenter(
	const bool resultant_in_world_frame,
	const bool contact_forces_in_local_frames) const {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	int num_surface_contacts = 0;
	for (const auto& contact : _environmental_contacts) {
		if (resultant_in_world_frame) {
			contact_locations.push_back(
				positionInWorld(contact._link_name, contact._contact_position));

		} else {
			contact_locations.push_back(
				position(contact._link_name, contact._contact_position));
		}
		contact_types.push_back(contact._contact_type);
		if (contact._contact_type == SurfaceContact) {
			num_surface_contacts++;
		}
	}
	GraspMatrixData G_data =
		graspMatrixAtGeometricCenter(contact_locations, contact_types);

	if (contact_forces_in_local_frames) {
		int n_contact_points = _environmental_contacts.size();
		int current_surface_contact = 0;
		for (int i = 0; i < n_contact_points; i++) {
			Matrix3d R_local;
			if (resultant_in_world_frame) {
				R_local = rotationInWorld(
					_environmental_contacts[i]._link_name,
					_environmental_contacts[i]._contact_orientation);
			} else {
				R_local =
					rotation(_environmental_contacts[i]._link_name,
							 _environmental_contacts[i]._contact_orientation);
			}

			G_data.G
				.block(0, 3 * i, 3 * (n_contact_points + num_surface_contacts),
					   3)
				.applyOnTheRight(R_local);
			G_data.G_inv
				.block(3 * i, 0, 3,
					   3 * (n_contact_points + num_surface_contacts))
				.applyOnTheLeft(R_local.transpose());

			if (_environmental_contacts[i]._contact_type == SurfaceContact) {
				G_data.G
					.block(0, 3 * (n_contact_points + current_surface_contact),
						   3 * (n_contact_points + num_surface_contacts), 3)
					.applyOnTheRight(R_local);
				G_data.G_inv
					.block(3 * (n_contact_points + current_surface_contact), 0,
						   3, 3 * (n_contact_points + num_surface_contacts))
					.applyOnTheLeft(R_local.transpose());

				if (n_contact_points > 2) {
					G_data.G
						.block(3 * (n_contact_points + current_surface_contact),
							   0, 3,
							   3 * (n_contact_points + num_surface_contacts))
						.applyOnTheLeft(R_local.transpose());
					G_data.G_inv
						.block(0,
							   3 * (n_contact_points + current_surface_contact),
							   3 * (n_contact_points + num_surface_contacts), 3)
						.applyOnTheRight(R_local);
				}

				current_surface_contact++;
			}
		}
	}
	return G_data;
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

VectorXd Sai2Model::modifiedNewtonEuler(const bool consider_gravity,
										const VectorXd& q, const VectorXd& dq,
										const VectorXd& dqa,
										const VectorXd& ddq) {
	VectorXd tau = VectorXd::Zero(_dof);

	vector<Vector3d> w, wa, dw, ddO, ddB, f, mom;
	vector<Vector3d> ripi_list, rib_list, z_list;
	Vector3d w_i, wa_i, dw_i, ddO_i, ddB_i, f_i, mom_i;
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
	mom_i.setZero();

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
	mom.push_back(mom_i);

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
		mom.push_back(mom_i);
	}

	// backward recursion
	for (int i = _dof; i > 0; i--) {
		Vector3d fip, mom_ip;
		vector<unsigned int> children = _rbdl_model->mu[i];
		if (children.size() == 0) {
			fip.setZero();
			mom_ip.setZero();
		} else if (children.size() == 1) {
			int child = children[0];
			fip = _rbdl_model->X_lambda[child].E.transpose() * f[child];
			mom_ip = _rbdl_model->X_lambda[child].E.transpose() * mom[child];
		} else {
			throw("tree structures not implemented yet");
		}

		double m = _rbdl_model->mBodies[i].mMass;
		Matrix3d I = _rbdl_model->mBodies[i].mInertia;

		f_i = fip + m * ddB[i];
		mom_i = mom_ip - f_i.cross(ripi_list[i] + rib_list[i]) +
				fip.cross(rib_list[i]) + I * dw[i] + wa[i].cross(I * w[i]);

		Vector3d zp = z_list[i];
		tau(i - 1) = mom_i.dot(zp);

		f[i] = f_i;
		mom[i] = mom_i;
	}
	return tau;
}

MatrixXd matrixRangeBasis(const MatrixXd& matrix, const double& tolerance) {
	const int range_size = matrix.rows();
	if(matrix.norm() < tolerance) {
		return MatrixXd::Zero(range_size, 1);
	}

	JacobiSVD<MatrixXd> svd(matrix, ComputeThinU | ComputeThinV);

	double sigma_0 = svd.singularValues()(0);
	if (sigma_0 < tolerance) {
		return MatrixXd::Zero(range_size, 1);
	}

	const int max_range = min(matrix.rows(), matrix.cols());
	int task_dof = max_range;
	for (int i = svd.singularValues().size() - 1; i > 0; i--) {
		if (svd.singularValues()(i) / sigma_0 < tolerance) {
			task_dof -= 1;
		} else {
			break;
		}
	}

	if (task_dof == matrix.rows()) {
		return MatrixXd::Identity(max_range, max_range);
	} else {
		return svd.matrixU().leftCols(task_dof);
	}
}

Vector3d orientationError(const Matrix3d& desired_orientation,
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
	} else {
		Vector3d rc1 = current_orientation.block<3, 1>(0, 0);
		Vector3d rc2 = current_orientation.block<3, 1>(0, 1);
		Vector3d rc3 = current_orientation.block<3, 1>(0, 2);
		Vector3d rd1 = desired_orientation.block<3, 1>(0, 0);
		Vector3d rd2 = desired_orientation.block<3, 1>(0, 1);
		Vector3d rd3 = desired_orientation.block<3, 1>(0, 2);
		return -1.0 / 2.0 * (rc1.cross(rd1) + rc2.cross(rd2) + rc3.cross(rd3));
	}
}

Vector3d orientationError(const Quaterniond& desired_orientation,
						  const Quaterniond& current_orientation) {
	Quaterniond inv_dlambda =
		desired_orientation * current_orientation.conjugate();
	return -2.0 * inv_dlambda.vec();
}

Matrix3d crossProductOperator(const Vector3d& v) {
	Matrix3d v_hat;
	v_hat << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
	return v_hat;
}

GraspMatrixData graspMatrixAtGeometricCenter(
	const vector<Vector3d>& contact_locations,
	const vector<ContactType>& contact_types) {
	MatrixXd G = MatrixXd::Zero(1, 1);
	MatrixXd G_inv = MatrixXd::Zero(1, 1);
	Matrix3d R = Matrix3d::Identity();
	Vector3d geometric_center = Vector3d::Zero();

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
		Wf.block<3, 3>(3, 3 * i) = crossProductOperator(ri);
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
					// throw runtime_error("Case 2-0 not implemented
					// yet\n");
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
					Matrix3d rx_cross = crossProductOperator(x);
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
						"Should not arrive here (number of contact points "
						"is "
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
					"Should not happen (number of contact points is 3, "
					"number "
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
					"Should not arrive here (number of contact points is "
					"4, "
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
				"Should not arrive here (number of contact points is not "
				"2, 3 "
				"or 4) in Sai2Model::graspMatrixAtGeometricCenter \n");
	}

	return GraspMatrixData(G, G_inv, R, geometric_center);
}

}  // namespace Sai2Model

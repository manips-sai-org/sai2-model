/*
 * SaiModel.h
 *
 *  Created on: Dec 14, 2016
 *      Author: Mikael Jorda
 */

#ifndef SaiModel_H_
#define SaiModel_H_

#include <rbdl/rbdl.h>

#include "JointLimits.h"
#include "parser/SaiModelParserUtils.h"

using namespace std;
using namespace Eigen;

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
}  // namespace Eigen

namespace SaiModel {

/**
 * @brief A structure to store the mass parameters of a link
 *
 */
struct LinkMassParams {
	/// @brief Mass of the link
	double mass;
	/// @brief Position of the center of mass in the link frame
	Vector3d com_pos;
	/// @brief Inertia of the link in the center of mass frame
	Matrix3d inertia;

	LinkMassParams(const double& mass, const Vector3d& com_pos,
				   const Matrix3d& inertia)
		: mass(mass), com_pos(com_pos), inertia(inertia) {}
};

/**
 * @brief A structure to store the operational space matrices
 *
 */
struct OpSpaceMatrices {
	/// @brief The task jacobian
	MatrixXd J;
	/// @brief The task operational space inertia matrix
	MatrixXd Lambda;
	/// @brief The dynamically consistent inverse of the task jacobian
	MatrixXd Jbar;
	/// @brief The dynamically consistent nullspace matrix of the task jacobian
	MatrixXd N;

	OpSpaceMatrices(const MatrixXd& J, const MatrixXd& Lambda,
					const MatrixXd& Jbar, const MatrixXd& N)
		: J(J), Lambda(Lambda), Jbar(Jbar), N(N) {}
};

/**
 * @brief A structure to store the grasp matrix data
 *
 */
struct GraspMatrixData {
	/// @brief The grasp matrix
	MatrixXd G;
	/// @brief The inverse of the grasp matrix
	MatrixXd G_inv;
	/// @brief The Rotation matrix that expresses the direction of tension in a
	/// 2 contacts grasp
	MatrixXd R;
	/// @brief The position at which the resultant forces are computed
	Vector3d resultant_point;

	GraspMatrixData(const MatrixXd& G, const MatrixXd& G_inv, const MatrixXd& R,
					const VectorXd& resultant_point)
		: G(G), G_inv(G_inv), R(R), resultant_point(resultant_point) {}
};

/**
 * @brief A structure to store the force sensor data
 *
 */
struct ForceSensorData {
	/// @brief Name of the robot or object to which the sensor is attached
	std::string robot_or_object_name;
	/// @brief Name of the link to which the sensor is attached
	std::string link_name;
	/// @brief transform from link to sensor frame. Measured moments are with
	/// respect to the sensor frame origin
	Eigen::Affine3d transform_in_link;
	/// @brief force applied to the environment in sensor frame
	Eigen::Vector3d force_local_frame;
	/// @brief moment applied to the environment in sensor frame
	Eigen::Vector3d moment_local_frame;
	/// @brief force applied to the environment in world frame
	Eigen::Vector3d force_world_frame;
	/// @brief moment applied to the environment in world frame
	Eigen::Vector3d moment_world_frame;

	ForceSensorData()
		: robot_or_object_name(""),
		  link_name(""),
		  transform_in_link(Eigen::Affine3d::Identity()),
		  force_local_frame(Eigen::Vector3d::Zero()),
		  moment_local_frame(Eigen::Vector3d::Zero()),
		  force_world_frame(Eigen::Vector3d::Zero()),
		  moment_world_frame(Eigen::Vector3d::Zero()) {}
};

/**
 * @brief Structure for the description of a spherical joint
 *
 */
struct SphericalJointDescription {
	/// @brief Name of the joint
	string joint_name;
	/// @brief Name of the parent link
	string parent_link_name;
	/// @brief Name of the child link
	string child_link_name;
	/// @brief Index of the first component of the joint in the joint vector
	/// (the indexes of the x, y, z compoenents of the quaternion description
	/// are index, index+1, index+2)
	int index;
	/// @brief Index of the w component of the quaternion description of the
	/// joint
	int w_index;

	SphericalJointDescription(const string joint_name,
							  const string parent_link_name,
							  const string child_link_name, const int index,
							  const int w_index)
		: joint_name(joint_name),
		  parent_link_name(parent_link_name),
		  child_link_name(child_link_name),
		  index(index),
		  w_index(w_index) {}
};

/// @brief Enum for the type of contact supported
enum ContactType { PointContact, SurfaceContact };

/**
 * @brief Structure for the description of a contact model. The contact model
 * represents a contact between the robot and the environment. It is used to
 * compute the grasp matrix. Only point and surface contacts are supported for
 * now.
 *
 */
struct ContactModel {
	/// @brief name of the link at which the contact occurs
	string contact_link_name;
	/// @brief the position of the contact in the link. if possible, it should
	/// be the geometrical center of the contact zone, or the point of contact
	/// in case of point contact
	Vector3d contact_position;
	/// @brief orientation of contact. Assumes the constrained direction is z.
	Matrix3d contact_orientation;
	/// @brief contact type. Only point and surface contacts are supported for
	/// now
	ContactType contact_type;

	ContactModel() = delete;
	ContactModel(const string& link_name, const Vector3d& pos,
				 const Matrix3d& orientation, const ContactType& contact_type) {
		this->contact_link_name = link_name;
		this->contact_position = pos;
		this->contact_orientation = orientation;
		this->contact_type = contact_type;
	}
	~ContactModel() = default;
};

/**
 * @brief This class represents a robot kinematic and dynamic model and provides
 * methods to compute many useful robot quantities, such as positions,
 * velocities, jacobians, operational space matrices, grasp matrices, etc.
 *
 */
class SaiModel {
public:
	/**
	 * @brief Construct a new Sai 2 Model object from a URDF file
	 *
	 * @param path_to_model_file path to the URDF file
	 * @param verbose whether to print debug information
	 */
	SaiModel(const string path_to_model_file, bool verbose = false);
	~SaiModel();

	// disallow empty, copy and asssign constructors
	SaiModel() = delete;
	SaiModel(SaiModel const&) = delete;
	SaiModel& operator=(SaiModel const&) = delete;

	/// @brief getter for joint positions
	const Eigen::VectorXd& q() const { return _q; }
	/// @brief setter for joint positions
	void setQ(const Eigen::VectorXd& q);

	/// @brief getter for joint velocities
	const Eigen::VectorXd& dq() const { return _dq; }
	/// @brief setter for joint velocities
	void setDq(const Eigen::VectorXd& dq);

	/// @brief Getter for the quaternion representation of a spherical joint by
	/// name
	const Eigen::Quaterniond sphericalQuat(const std::string& joint_name) const;
	/// @brief Setter for the quaternion representation of a spherical joint by
	/// name
	void setSphericalQuat(const std::string& joint_name,
						  Eigen::Quaterniond quat);

	/// @brief getter for joint accelerations
	const Eigen::VectorXd& ddq() const { return _ddq; }
	/// @brief setter for joint accelerations
	void setDdq(const Eigen::VectorXd& ddq);

	/// @brief getter for the joint space mass matrix
	const Eigen::MatrixXd& M() const { return _M; }
	/// @brief getter for the inverse of the joint space mass matrix
	const Eigen::MatrixXd& MInv() const { return _M_inv; }

	/// @brief getter for the 3D world gravity vector
	const Eigen::Vector3d worldGravity() const {
		return _T_world_robot.linear() * _rbdl_model->gravity;
	}
	/// @brief setter for the 3D world gravity vector
	void setWorldGravity(const Vector3d& world_gravity) {
		_rbdl_model->gravity =
			_T_world_robot.linear().transpose() * world_gravity;
	}

	/// @brief getter for the joint limits
	/// @return a vector of JointLimit structures, one per joint
	const std::vector<JointLimit>& jointLimits() const { return _joint_limits; }

	/// @brief getter for the upper position of all joints as a VectorXd of size
	/// q_size for continuous joints, the limits are set to
	/// +numeric_limits<double>::max(). for spherical joints, the limits of all
	/// the corresponding quaternion coefficients are set to +1.
	VectorXd jointLimitsPositionLower() const;
	/// @brief getter for the lower position of all joints as a VectorXd of size
	/// q_size for continuous joints, the limits are set to
	/// -numeric_limits<double>::max(). for spherical joints, the limits of all
	/// the corresponding quaternion coefficients are set to -1.
	VectorXd jointLimitsPositionUpper() const;

	/// @brief returns a vector of SphereicalJointDescription structures, one
	/// per spherical joint in the model
	const std::vector<SphericalJointDescription>& sphericalJoints() const {
		return _spherical_joints;
	}

	/// @brief getter for the robot base transform. This is the robot base pose
	/// with respect to the world frame
	const Eigen::Affine3d& TRobotBase() const { return _T_world_robot; }
	/// @brief setter for the robot base transform. This is the robot base pose
	/// with respect to the world frame
	void setTRobotBase(const Affine3d& T);

	/// @brief returns a vector of all the joint names in the model
	std::vector<std::string> jointNames() const;

	/// @brief returns true if the given link is part of the robot
	bool isLinkInRobot(const std::string& link_name) const;

	/**
	 * @brief      update the kinematics (needed to have up to date positions,
	 * velocities and jacobians) for the current robot configuration.
	 */
	void updateKinematics();

	/**
	 * @brief      update the kinematics and dynamics (mass matrix and its
	 *             inverse) for the current robot configuration.
	 */
	void updateModel();

	/**
	 * @brief update the kinematics and the inverse of the mass matrix
	 *        with an externally provided mass matrix (for example from the
	 *        robot's manufacturer API) insetad of computing the mass matrix
	 *        from the urdf description of the robot.
	 *
	 * @param M externally provided mass matrix
	 */
	void updateModel(const Eigen::MatrixXd& M);

	/**
	 * @brief      returns the number of degrees of freedom of the robot
	 *
	 * @return     number of dof of robot
	 */
	const int& dof() const { return _dof; }

	/**
	 * @brief      returns the number of values required for robot joint
	 *             positions description. equals dof unless spherical or
	 * floating joints are present.
	 *
	 * @return     number of values required for robot joint positions
	 * description
	 */
	const int& qSize() const { return _q_size; }

	/**
	 * @brief computes and returns the joint gravity vector for the last updated
	 * configuration. This is the vector of torques required to compensate for
	 * the gravity forces acting on the robot.
	 *
	 * @return the joint gravity vector
	 */
	VectorXd jointGravityVector();

	/**
	 * @brief computes and returns the joint coriolis and centrifugal forces of
	 * the last updated configuration
	 *
	 * @return	 The joint coriolis and centrifugal forces
	 */
	VectorXd coriolisForce();

	/**
	 * @brief Computes and returns the non linear effects of the robot dynamics.
	 * That is to say the gravity plus the coriolis and centrifugal forces.
	 *
	 * @return The non linear effects of the robot dynamics
	 */
	VectorXd coriolisPlusGravity();

	/**
	 * @brief      Computes the matrix C such that the coriolis and centrifucal
	 * forces can be expressed b = C*q_dot
	 *
	 * @param      C     return matrix
	 */
	MatrixXd factorizedChristoffelMatrix();

	/**
	 * @brief Full jacobian for link, relative to base (id=0) in the form  [Jv;
	 * Jw] (angular first, linear next) expressed in robot base frame
	 *
	 * @param      link_name    the name of the link where to compute the
	 *                          jacobian
	 * @param      pos_in_link  the position of the point in the link where the
	 *                          jacobian is computed (in local link frame)
	 * @return     J            Jacobian in the form [Jv, Jw] in robot base
	 * frame
	 */
	MatrixXd J(const string& link_name,
			   const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief Similar to the J function, but the returned jacobian is expressed
	 * in the world frame instead of the robot base frame
	 *
	 * @param link_name the name of the link where to compute the jacobian
	 * @param pos_in_link the position of the point in the link where the
	 * jacobian is computed (in local link frame)
	 * @return Jacobian in the form [Jv, Jw] in world frame
	 */
	MatrixXd JWorldFrame(const string& link_name,
						 const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief Similar to the J function, but the returned jacobian is expressed
	 * in a local frame (with respect to the link frame) instead of the robot
	 * base frame
	 *
	 * @param link_name  the name of the link where to compute the jacobian
	 * @param pos_in_link  the position of the point in the link where the
	 * jacobian is computed (in local link frame)
	 * @param rot_in_link   the rotation of the local frame in the link frame
	 * @return Jacobian in the form [Jv, Jw] in local frame
	 */
	MatrixXd JLocalFrame(
		const string& link_name, const Vector3d& pos_in_link = Vector3d::Zero(),
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;

	/**
	 * @brief Velocity jacobian for link, relative to base (id=0) expressed in
	 * robot base frame
	 *
	 * @param link_name the name of the link where to compute the jacobian
	 * @param pos_in_link the position of the point in the link where the
	 * jacobian is computed (in local link frame)
	 * @return MatrixXd linear velocity jacobian in robot base frame
	 */
	MatrixXd Jv(const string& link_name,
				const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief Similar to the Jv function, but the returned jacobian is expressed
	 * in world frame instead of the robot base frame
	 *
	 * @param link_name  the name of the link where to compute the jacobian
	 * @param pos_in_link  the position of the point in the link where the
	 * jacobian is computed (in local link frame)
	 * @return MatrixXd linear velocity jacobian in world frame
	 */
	MatrixXd JvWorldFrame(const string& link_name,
						  const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief Similar to the Jv function, but the returned jacobian is expressed
	 * in a local frame (with respect to the link frame) instead of the robot
	 * base frame
	 *
	 * @param link_name  the name of the link where to compute the jacobian
	 * @param pos_in_link  the position of the point in the link where the
	 * jacobian is computed (in local link frame)
	 * @param rot_in_link  the rotation of the local frame in the link frame
	 * @return MatrixXd linear velocity jacobian in local frame
	 */
	MatrixXd JvLocalFrame(
		const string& link_name, const Vector3d& pos_in_link = Vector3d::Zero(),
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;

	/**
	 * @brief Angular velocity jacobian for link, relative to base (id=0)
	 * expressed in robot base frame
	 *
	 * @param link_name the name of the link where to compute the jacobian
	 * @return MatrixXd angular velocity jacobian in robot base frame
	 */

	MatrixXd Jw(const string& link_name) const;
	/**
	 * @brief Similar to the Jw function, but the returned jacobian is expressed
	 * in world frame instead of the robot base frame
	 *
	 * @param link_name the name of the link where to compute the jacobian
	 * @return MatrixXd angular velocity jacobian in world frame
	 */

	MatrixXd JwWorldFrame(const string& link_name) const;
	/**
	 * @brief Similar to the Jw function, but the returned jacobian is expressed
	 * in a local frame (with respect to the link frame) instead of the robot
	 * base frame
	 *
	 * @param link_name the name of the link where to compute the jacobian
	 * @param rot_in_link the rotation of the local frame in the link frame
	 * @return MatrixXd angular velocity jacobian in local frame
	 */

	MatrixXd JwLocalFrame(
		const string& link_name,
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;

	/**
	 * @brief Computes the inverse kinematics to get the robot configuration
	 * that matches a set of points on the robot to desired positions. Uses a
	 * modified underlying RBDL function based on an iterative computation using
	 * the damped Levenberg-Marquardt method (also known as Damped Least Squares
	 * method) with output saturation for the joint limits. Always returns the
	 * value from the last iteration, even if the IK did not converge. Uses the
	 * current joint positions as initial guess, and does not update the robot
	 * configuration.
	 *
	 * @param link_names vector of link names that contain the points to match
	 * @param pos_in_links positions of the points in the links
	 * @param desired_pos_world_frame desired positions of the points in world
	 * frame
	 * @return VectorXd set of joint angles that match the desired points as
	 * best as possible
	 */
	VectorXd computeInverseKinematics(
		const vector<string>& link_names, const vector<Vector3d>& pos_in_links,
		const vector<Vector3d>& desired_pos_world_frame);

	/**
	 * @brief Computes the inverse kinematics to get the robot configuration
	 * that matches a set of frames on some robot links to a set of desired
	 * frames. Uses the same underlying function as the previous function, but
	 * with frames instead of points. Uses the current joint positions as
	 * initial guess, and does not update the robot configuration.
	 *
	 * @param link_names vector of link names that contain the frames to match
	 * @param frames_in_links frames in the links
	 * @param desired_frames_locations_in_world_frame desired frames in world
	 * @return VectorXd set of joint angles that match the desired frames as
	 * best as possible
	 */
	VectorXd computeInverseKinematics(
		const vector<string>& link_names,
		const vector<Affine3d>& frames_in_links,
		const vector<Affine3d>& desired_frames_locations_in_world_frame);

	/**
	 * @brief      transformation from base to link frame (possibly a local
	 * frame expressed in link frame), in robot base base coordinates. This
	 * represents the operator that "moves" the base frame to the link frame,
	 * which also means it represents the coordinate transform matrix to express
	 * a point in base frame, if we know it in link frame:  p_base = T * p_link
	 *
	 * @param      link_name    name of the link where to compute the
	 *                          transformation matrix
	 * @param      pos_in_link  The position in local body coordinates
	 * @param      rot_in_link  The rotation in local body coordinates
	 * @return	   Affine3d            The transfrom operator from base frame to
	 * link frame, or the coordinate change matrix from link frame to base frame
	 */
	Affine3d transform(
		const string& link_name, const Vector3d& pos_in_link = Vector3d::Zero(),
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;

	/**
	 * @brief Similar to the transform function, but the returned transformation
	 * is expressed in world frame instead of the robot base frame
	 *
	 * @param link_name the name of the link where to compute the transformation
	 * @param pos_in_link the position in local body coordinates
	 * @param rot_in_link the rotation in local body coordinates
	 * @return Affine3d The transfrom operator from world frame to link frame.
	 */
	Affine3d transformInWorld(
		const string& link_name, const Vector3d& pos_in_link = Vector3d::Zero(),
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;

	/**
	 * @brief Computes the linear and angular velocity of a point in a link. The
	 * returned value is expressed in the base frame and is of the form [v; w]
	 * (linear first, angular next)
	 *
	 * @param link_name the name of the link in which is the point where to
	 * compute the velocity
	 * @param pos_in_link the position of the point in the link, in local link
	 * frame
	 * @return Vector6d linear and angular velocity of the point in the link in
	 * robot base frame
	 */
	Vector6d velocity6d(const string link_name,
						const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief Similar to the velocity6d function, but the returned velocity is
	 * in world frame instead of the robot base frame
	 *
	 * @param link_name the name of the link in which is the point where to
	 * compute the velocity
	 * @param pos_in_link the position of the point in the link, in local link
	 * frame
	 * @return Vector6d linear and angular velocity of the point in the link in
	 * world frame
	 */
	Vector6d velocity6dInWorld(
		const string link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief Computes the linear and angular acceleration of a point in a link.
	 * The returned value is expressed in the base frame and is of the form [a;
	 * alpha] (linear first, angular next). Note: Acceleration computations are
	 * very sensitive to frames being correct. So it is safer to call
	 * UpdateKinematics before calling this.
	 *
	 * @param link_name the name of the link in which is the point where to
	 * compute the acceleration
	 * @param pos_in_link the position of the point in the link, in local link
	 * frame
	 * @return Vector6d linear and angular acceleration of the point in the link
	 * in robot base frame
	 */
	Vector6d acceleration6d(
		const string link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief Similar to the acceleration6d function, but the returned
	 * acceleration is in world frame instead of the robot base frame
	 *
	 * @param link_name	the name of the link in which is the point where to
	 * compute the acceleration
	 * @param pos_in_link the position of the point in the link, in local link
	 * frame
	 * @return Vector6d linear and angular acceleration of the point in the link
	 * in world frame
	 */
	Vector6d acceleration6dInWorld(
		const string link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief Computes the position of a point in a link. The returned value is
	 * in the base frame by default.
	 *
	 * @param link_name the name of the link in which is the point where to
	 * compute the position
	 * @param pos_in_link the position of the point in the link, in local link
	 * frame
	 * @return Vector3d position of the point in the link in robot base frame
	 */
	Vector3d position(const string& link_name,
					  const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief Similar to the position function, but the returned position is in
	 * a world frame instead of the robot base frame
	 *
	 * @param link_name the name of the link in which is the point where to
	 * compute the position
	 * @param pos_in_link the position of the point in the link, in local link
	 * frame
	 * @return Vector3d position of the point in the link in world frame
	 */
	Vector3d positionInWorld(
		const string& link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief      Computes the velocity of point in link, in base coordinates
	 *
	 * @param      link_name    name of the link in which is the point where to
	 *                          compute the velocity
	 * @param      pos_in_link  the position of the point in the link, in local
	 *                          link frame
	 * @return     Vector3d     velocity of the point in the link in robot base
	 */
	Vector3d linearVelocity(
		const string& link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief Similar to the linearVelocity function, but the returned velocity
	 * is in world frame instead of the robot base frame
	 *
	 * @param link_name the name of the link in which is the point where to
	 * compute the velocity
	 * @param pos_in_link the position of the point in the link, in local link
	 * frame
	 * @return Vector3d velocity of the point in the link in world frame
	 */
	Vector3d linearVelocityInWorld(
		const string& link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief      Computes the acceleration of point in link, in base
	 * coordinates Note: Acceleration computations are very sensitive to frames
	 *             being correct. So it is safer to call UpdateKinematics before
	 *             calling this, unless this is called right after a simulator
	 *             integrator state.
	 *
	 * @param      link_name    name of the link in which is the point where to
	 *                          compute the velocity
	 * @param      pos_in_link  the position of the point in the link, in local
	 *                          link frame
	 * @return     Vector3d     acceleration of the point in the link in robot
	 * base frame
	 */
	Vector3d linearAcceleration(
		const string& link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief Similar to the linearAcceleration function, but the returned
	 * acceleration is in world frame instead of the robot base frame
	 *
	 * @param link_name the name of the link in which is the point where to
	 * compute the acceleration
	 * @param pos_in_link the position of the point in the link, in local link
	 * frame
	 * @return Vector3d acceleration of the point in the link in world frame
	 */
	Vector3d linearAccelerationInWorld(
		const string& link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief      Computes the rotation of a link (possibly a local frame
	 * expressed in link frame) with respect to base frame
	 *
	 * @param      link_name    name of the link for which to compute the
	 *                          rotation
	 * @param[in]  rot_in_link  Local frame of interest expressed in link frame
	 * @return     Matrix3d     rotation of the link in robot base frame
	 */
	Matrix3d rotation(const string& link_name,
					  const Matrix3d& rot_in_link = Matrix3d::Identity()) const;

	/**
	 * @brief Similar to the rotation function, but the returned rotation is in
	 * world frame instead of the robot base frame
	 *
	 * @param link_name the name of the link for which to compute the rotation
	 * @param rot_in_link Local frame of interest expressed in link frame
	 * @return Matrix3d rotation of the link in world frame
	 */
	Matrix3d rotationInWorld(
		const string& link_name,
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;

	/**
	 * @brief      Computes the angular Velocity of a link (possibly a local
	 * frame expressed in link frame) with respect to base frame (default)
	 *
	 * @param      link_name    name of the link for which to compute the
	 *                          rotation
	 * @param[in]  rot_in_link  Local frame of interest expressed in link frame
	 * @return     Vector3d     angular velocity of the link in robot base frame
	 */
	Vector3d angularVelocity(const string& link_name) const;

	/**
	 * @brief Similar to the angularVelocity function, but the returned angular
	 * velocity is in world frame instead of the robot base frame
	 *
	 * @param link_name the name of the link for which to compute the rotation
	 * @return Vector3d angular velocity of the link in world frame
	 */
	Vector3d angularVelocityInWorld(const string& link_name) const;

	/**
	 * @brief      Computes the angular Acceleration of a link (possibly a local
	 * frame expressed in link frame) with respect to base frame Note:
	 * Acceleration computations are very sensitive to frames being correct. So
	 * it is safer to call UpdateKinematics before calling this, unless this is
	 * called right after a simulator integrator state.
	 *
	 * @param      link_name    name of the link for which to compute the
	 *                          rotation
	 * @param[in]  rot_in_link  Local frame of interest expressed in link frame
	 * @return     Vector3d     angular acceleration of the link in robot base
	 * frame
	 */
	Vector3d angularAcceleration(const string& link_name) const;

	/**
	 * @brief Similar to the angularAcceleration function, but the returned
	 * angular acceleration is in world frame instead of the robot base frame
	 *
	 * @param link_name the name of the link for which to compute the rotation
	 * @return Vector3d angular acceleration of the link in world frame
	 */
	Vector3d angularAccelerationInWorld(const string& link_name) const;

	/**
	 * @brief      Gives the joint index for a given name. For spherical joints,
	 * returns the first index of the vector part of the quaternion.
	 *
	 * @param      joint_name  name of the joint
	 * @return     the joint index
	 */
	int jointIndex(const string& joint_name) const;

	/**
	 * @brief      Returns the index of the scalar part of the quaternion for a
	 * spherical joint. Throws an error if the joint is not spherical
	 *
	 * @param      joint_name  name of the joint
	 *
	 * @return     the index of the w coefficient of the quaternion
	 */
	int sphericalJointIndexW(const string& joint_name) const;

	/**
	 * @brief returns the joint name for the given index (for spherical joints,
	 * any index corresponding to the position of one of the quaternion
	 * coefficients will return the spherical joint name)
	 *
	 * @param joint_id index of the joint
	 * @return std::string name of the joint
	 */
	std::string jointName(const int joint_id) const;

	/**
	 * @brief      Returns the link directly attached to the given joint id
	 *
	 * @param      joint_name  name of the joint
	 * @return     the child link name
	 */
	std::string childLinkName(const std::string& joint_name) const;

	/**
	 * @brief      Returns the link name parent of the given joint id
	 *
	 * @param      joint_name  name of the joint
	 * @return     the parent link name
	 */
	std::string parentLinkName(const std::string& joint_name) const;

	/**
	 * @brief      Gives the mass properties of a given link
	 *
	 * @param      mass            the returned mass value
	 * @param      center_of_mass  the returned position of the center of mass
	 * in the body's frame
	 * @param      inertia         the returned inertia of the given link
	 * @param      link_name       the name of the considered link
	 */
	LinkMassParams getLinkMassParams(const string& link_name) const;

	/**
	 * @brief      returns the position of the center of mass of the robot in
	 *             robot base frame
	 */
	Vector3d comPosition() const;

	/**
	 * @brief      returns the center of mass velocity Jacobian of the robot in
	 *             robot base frame
	 */
	MatrixXd comJacobian() const;

	/**
	 * @brief      Computes the operational space inertia matrix corresponding
	 * to a given Jacobian
	 *
	 * @param      task_jacobian  The jacobian of the task for which we want the
	 *                            op space mass matrix
	 * @return     The operational space inertia matrix
	 */
	Eigen::MatrixXd taskInertiaMatrix(const MatrixXd& task_jacobian) const;

	/**
	 * @brief      Computes the operational space inertia matrix robust to
	 * singularities by using the pseudo inverse of J*M^{-1}*J^T
	 *
	 * @param      task_jacobian  The jacobian of the task for which we want the
	 *                            op space mass matrix
	 * @return     The operational space inertia matrix
	 */
	MatrixXd taskInertiaMatrixWithPseudoInv(
		const MatrixXd& task_jacobian) const;

	/**
	 * @brief      Computes the dynamically consistent inverse of the jacobian
	 *             for a given task. Recomputes the task inertia at each call
	 *
	 * @param[in]  task_jacobian  The task jacobian
	 * @return     The dynamically consistent inverse jacobian
	 */
	MatrixXd dynConsistentInverseJacobian(const MatrixXd& task_jacobian) const;

	/**
	 * @brief      Computes the dynamically consistent nullspace of the jacobian
	 *
	 * @param[in]  task_jacobian  The task jacobian
	 * @return     The dynamically consistent nullspace matrix
	 */
	MatrixXd nullspaceMatrix(const MatrixXd& task_jacobian) const;

	/**
	 * @brief Computes the operational space matrices (task inertia, dynamically
	 * consistent inverse of the jacobian and nullspace) for a given task. More
	 * efficient than calling the three individual functions because it avoids
	 * repeated computations.
	 *
	 * @param  task_jacobian  Task jacobian
	 * @return OpSpaceMatrices struct containing the task inertia, dynamically
	 * consistent inverse of the jacobian and nullspace
	 */
	OpSpaceMatrices operationalSpaceMatrices(
		const MatrixXd& task_jacobian) const;

	/**
	 @brief Adds an environmental contact to the desired link at the desired
	 contact frame. There can only be one contact per link

	 @param[in]  link link at which the contact happens
	 @param[in]  pos_in_link position of the contact in the link in locallink
	 frame
	 @param[in]  orientation orientation of the contact frame in the link frame.
	 Z axis needs to be aligned with the constrained direction of motion
	 (surface normal)
	 @param[in]  contact_type The contact type (surface or point)
	*/
	void addEnvironmentalContact(
		const string link, const Vector3d pos_in_link = Vector3d::Zero(),
		const Matrix3d orientation = Matrix3d::Identity(),
		const ContactType contact_type = ContactType::SurfaceContact);

	/**
	 @brief Adds a manipulation contact to the desired link at the desired
	 contact frame. There can only be one contact per link

	 @param[in]  link link at which the contact happens
	 @param[in]  pos_in_link position of the contact in the link in locallink
	 frame
	 @param[in]  orientation orientation of the contact frame in the link frame.
	 Z axis needs to be aligned with the constrained direction of motion
	 (surface normal)
	 @param[in]  contact_type The contact type (surface or point)
	*/
	void addManipulationContact(
		const string link, const Vector3d pos_in_link = Vector3d::Zero(),
		const Matrix3d orientation = Matrix3d::Identity(),
		const ContactType contact_type = ContactType::SurfaceContact);

	/**
	 * @brief Updates the environmental contact frame or type at the given link
	 *
	 * @param[in]  link          The link
	 * @param[in]  pos_in_link   The position in link
	 * @param[in]  orientation   The orientation
	 * @param[in]  contact_type  The contact type
	 */
	void updateEnvironmentalContact(const string link,
									const Vector3d pos_in_link,
									const Matrix3d orientation,
									const ContactType contact_type);

	/**
	 * @brief Updates the manipulation contact frame or type at the given link
	 *
	 * @param[in]  link          The link
	 * @param[in]  pos_in_link   The position in link
	 * @param[in]  orientation   The orientation
	 * @param[in]  contact_type  The contact type
	 */
	void updateManipulationContact(const string link,
								   const Vector3d pos_in_link,
								   const Matrix3d orientation,
								   const ContactType contact_type);

	/**
	 * @brief      deletes the environmental contact at a given link
	 *
	 * @param      link_name  the link at which we want to delete the contact
	 */
	void deleteEnvironmentalContact(const string link_name);

	/**
	 * @brief      deletes the manipulation contact at a given link
	 *
	 * @param      link_name  the link at which we want to delete the contact
	 */
	void deleteManipulationContact(const string link_name);

	/**
	 * @brief Computes the manipulation grasp matrix that converts contact
	 * forces expressed in robot frame (default), world frame or local contact
	 * frames into the resultant force expressed in robot frame (default) or
	 * world frame, and the internal froces and moments.
	 *
	 * @param center_point the position (in robot or world frame) of the point
	 * on which we resolve the resultant forces
	 * @param resultant_in_world_frame if true, the resultant force is expressed
	 * in world frame, otherwise in robot frame
	 * @param contact_forces_in_local_frames if true, the contact forces are
	 * expressed in local contact frames, otherwise in robot frame
	 * @return GraspMatrixData the structure containing the grasp matrix, its
	 * inverse and the rotation matrix of the grasp
	 */
	GraspMatrixData manipulationGraspMatrix(
		const Vector3d& center_point,
		const bool resultant_in_world_frame = false,
		const bool contact_forces_in_local_frames = false) const;

	/**
	 * @brief Similar to the manipulationGraspMatrix function, but the
	 * center_point is taken to be the geometric center of the contacts
	 *
	 * @param center_point the position (in robot or world frame) of the point
	 * on which we resolve the resultant forces
	 * @param resultant_in_world_frame if true, the resultant force is expressed
	 * in world frame, otherwise in robot frame
	 */
	GraspMatrixData manipulationGraspMatrixAtGeometricCenter(
		const bool resultant_in_world_frame = false,
		const bool contact_forces_in_local_frames = false) const;

	/**
	 * @brief Computes the environmental grasp matrix that converts contact
	 * forces expressed in robot frame (default), world frame or local contact
	 * frames into the resultant force expressed in robot frame (default) or
	 * world frame, and the internal froces and moments.
	 *
	 * @param center_point the position (in robot or world frame) of the point
	 * on which we resolve the resultant forces
	 * @param resultant_in_world_frame if true, the resultant force is expressed
	 * in world frame, otherwise in robot frame
	 * @param contact_forces_in_local_frames if true, the contact forces are
	 * expressed in local contact frames, otherwise in robot frame
	 * @return GraspMatrixData the structure containing the grasp matrix, its
	 * inverse and the rotation matrix of the grasp
	 */
	GraspMatrixData environmentalGraspMatrix(
		const Vector3d& center_point,
		const bool resultant_in_world_frame = false,
		const bool contact_forces_in_local_frames = false) const;

	/**
	 * @brief Similar to the environmentalGraspMatrix function, but the
	 * center_point is taken to be the geometric center of the contacts
	 *
	 * @param center_point the position (in robot or world frame) of the point
	 * on which we resolve the resultant forces
	 * @param resultant_in_world_frame if true, the resultant force is expressed
	 * in world frame, otherwise in robot frame
	 */
	GraspMatrixData environmentalGraspMatrixAtGeometricCenter(
		const bool resultant_in_world_frame = false,
		const bool contact_forces_in_local_frames = false) const;

	/**
	 * @brief print to terminal the joints of the robot with the corresponding
	 * numbers
	 */
	void displayJoints();

	/**
	 * @brief print to terminal the links of the robot with the corresponding
	 * numbers
	 */
	void displayLinks();

private:
	/**
	 * @brief      update the dynamics (mass matrix and its inverse) for the
	 * current robot configuration.
	 */
	void updateDynamics();

	/**
	 * @brief      update the inverse inertia matrix.
	 */
	void updateInverseInertia();

	/**
	 * @brief Compute the inverse kinematics from the constraint set as defined
	 * in RBDL kinematics module, using a variation of RBDL IK algorithm that
	 * incorporates the robot joint limits. Returns the value from the last
	 * iteration even if the IK did not converge.
	 *
	 * @param cs The constraint set (automatically built by the public interface
	 * function)
	 * @return VectorXd the joint angles from latest iteration
	 */
	VectorXd IKInternal(RigidBodyDynamics::InverseKinematicsConstraintSet& cs);

	/**
	 * @brief      Computes the modified Newton-Euler Algorithm as described in
	 *             ''' De Luca, A., & Ferrajoli, L. (2009, May). A modified
	 *             Newton-Euler method for dynamic computations in robot fault
	 *             detection and control. In Robotics and Automation, 2009.
	 *             ICRA'09. IEEE International Conference on (pp. 3359-3364).
	 *             IEEE. ''' @tau                       return vector
	 *
	 * @param      tau               return vector
	 * @param      consider_gravity  consider or not the acceleration due to
	 *                               gravity at the base
	 * @param      q                 joint positions
	 * @param      dq                joint velocity
	 * @param      dqa               auxiliary joint velocity
	 * @param      ddq               joint acceleration
	 */
	VectorXd modifiedNewtonEuler(const bool consider_gravity, const VectorXd& q,
								 const VectorXd& dq, const VectorXd& dqa,
								 const VectorXd& ddq);

	/**
	 * @brief      Gives the link id for a given name with the right indexing
	 *             for rbdl
	 *
	 * @param      link_name  name of the link
	 *
	 * @return     the link number
	 */
	unsigned int linkIdRbdl(const string& link_name) const;

	/// @brief internal rbdl model
	RigidBodyDynamics::Model* _rbdl_model;

	/// @brief Joint positions. Note: _q size can differ from dof() since
	/// spherical joints use quaternions.
	VectorXd _q;

	/// @brief Joint velocities
	VectorXd _dq;

	/// @brief Joint accelerations
	VectorXd _ddq;

	/// @brief Mass Matrix
	MatrixXd _M;

	/// @brief Inverse of the mass matrix
	MatrixXd _M_inv;

	/// @brief List of active contacts between robot and environment
	vector<ContactModel> _environmental_contacts;

	/// @brief List of active contacts between robot end effectors and
	/// manipulated objects
	vector<ContactModel> _manipulation_contacts;

	/// @brief number of Dof of robot
	int _dof;

	/// @brief number of values required for robot joint positions description
	int _q_size;

	/// @brief Transform from world coordinates to robot base coordinates
	Affine3d _T_world_robot;

	/// @brief map from joint id to joint names
	map<int, string> _joint_id_to_names_map;

	/// @brief map from joint names to child link names
	map<string, string> _joint_names_to_child_link_names_map;

	/// @brief map from joint names to parent link names
	map<string, string> _joint_names_to_parent_link_names_map;

	/// @brief vector of spherical joints
	vector<SphericalJointDescription> _spherical_joints;

	/// @brief map from joint names to joint id (with rbdl indexing)
	map<string, int> _joint_names_to_id_map;

	/// @brief map from link names to link id (with rbdl indexing)
	map<string, int> _link_names_to_id_map;

	/// @brief joint limits for positions, velocity and torque, parsed from URDF
	vector<JointLimit> _joint_limits;
};

/**
 * @brief Computes the pseudo inverse by computing the svd and setting the
 * singular values lower than the tolerance to zero in the inverse
 *
 * @param matrix the input matrix
 * @param tolerance the threshold to ignore singular values
 * @return MatrixXd the pseudo inverse of the input matrix
 */
MatrixXd computePseudoInverse(const MatrixXd& matrix,
							  const double& tolerance = 1e-6);

/**
 * @brief Computes the range space of the input matrix where the singular
 * directions (directions with singular values lower than the tolerance) have
 * been removed. The returned matrix is a of size n x k where n in the number of
 * rows of the input matrix and k its range. Its columns correnpond to a basis
 * of the matrix range
 *
 * @param matrix     the input matrix
 * @param tolerance  the threshold to ignore singular values
 * @return a matrix whose columns form the base of the input matrix range space
 */
MatrixXd matrixRangeBasis(const MatrixXd& matrix,
						  const double& tolerance = 1e-6);

/**
 * @brief      Compute the orientation error from rotation matrices
 *
 * @param      desired_orientation  desired orientation rotation matrix
 * @param      current_orientation  current orientation matrix
 * @return     the orientation error
 */
Vector3d orientationError(const Matrix3d& desired_orientation,
						  const Matrix3d& current_orientation);

/**
 * @brief      Compute the orientation error from quaternions
 *
 * @param      desired_orientation  desired orientation quaternion
 * @param      current_orientation  current orientation quaternion
 * @return     the orientation error
 */
Vector3d orientationError(const Quaterniond& desired_orientation,
						  const Quaterniond& current_orientation);

/// @brief compute the cross product operator of a 3d vector
Matrix3d crossProductOperator(const Vector3d& v);

/**
 * @brief      Computes the grasp matrix and its inverse in the cases where
 *             there are 2, 3 or 4 contacts. the external forces and moments are
 *             assumed to be in base frame. For 2 contact points, the output
 *             resultant (first 6 lines) is given in base frame, and the output
 *             internal tension and moments are given in local frame, and the
 *             description of the local frame is given by R. For 3 and 4
 *             contacts, the output quantities are given in base frame. The
 *             convention for the output is the following order : support
 *             forces, support moments, internal tensions, internal moments the
 *             internal tensions are given in the order 1-2, 1-3, 2-3 in the 3
 *             contact case and 1-2, 1-3, 1-4, 2-3, 2-4, 3-4 in the 4 contact
 *             case. Line contacts are not yet supported.
 *
 * @param      contact_locations  A vector of location of the contact points
 * @param      contact_types      A vector of contact types
 * @return     A GraspMatrixData structure containing the grasp matrix, its
 * inverse and the rotation matrix of the grasp
 */
GraspMatrixData graspMatrixAtGeometricCenter(
	const vector<Vector3d>& contact_locations,
	const vector<ContactType>& contact_types);

} /* namespace SaiModel */

#endif /* SaiModel_H_ */

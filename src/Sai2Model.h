/*
 * Sai2Model.h
 *
 *  Created on: Dec 14, 2016
 *      Author: Mikael Jorda
 */

#ifndef SAI2MODEL_H_
#define SAI2MODEL_H_

#include <rbdl/rbdl.h>

#include "JointLimits.h"

using namespace std;
using namespace Eigen;

namespace Eigen {
typedef Matrix<double, 6, 1> Vector6d;
}  // namespace Eigen

namespace Sai2Model {

struct LinkMassParams {
	double mass;
	Vector3d com_pos;
	Matrix3d inertia;

	LinkMassParams(const double& mass, const Vector3d& com_pos,
				   const Matrix3d& inertia)
		: mass(mass), com_pos(com_pos), inertia(inertia) {}
};

struct OpSpaceMatrices {
	MatrixXd J;
	MatrixXd Lambda;
	MatrixXd Jbar;
	MatrixXd N;

	OpSpaceMatrices(const MatrixXd& J, const MatrixXd& Lambda,
					const MatrixXd& Jbar, const MatrixXd& N)
		: J(J), Lambda(Lambda), Jbar(Jbar), N(N) {}
};

struct GraspMatrixData {
	MatrixXd G;
	MatrixXd G_inv;
	MatrixXd R;
	Vector3d resultant_point;

	GraspMatrixData(const MatrixXd& G, const MatrixXd& G_inv, const MatrixXd& R,
					const VectorXd& resultant_point)
		: G(G), G_inv(G_inv), R(R), resultant_point(resultant_point) {}
};

// Basic data structure for force sensor data
struct ForceSensorData {
	std::string robot_name;	 // name of robot to which sensor is attached
	std::string link_name;	 // name of link to which sensor is attached
	// transform from link to sensor frame. Measured moments are with respect to
	// the sensor frame origin
	Eigen::Affine3d transform_in_link;
	Eigen::Vector3d
		force_local_frame;	// force applied to the environment in sensor frame
	Eigen::Vector3d moment_local_frame;	 // moment applied to the environment
										 // in sensor frame
	Eigen::Vector3d
		force_world_frame;	// force applied to the environment in world frame
	Eigen::Vector3d moment_world_frame;	 // moment applied to the environment
										 // in world frame

	ForceSensorData()
		: robot_name(""),
		  link_name(""),
		  transform_in_link(Eigen::Affine3d::Identity()),
		  force_local_frame(Eigen::Vector3d::Zero()),
		  moment_local_frame(Eigen::Vector3d::Zero()),
		  force_world_frame(Eigen::Vector3d::Zero()),
		  moment_world_frame(Eigen::Vector3d::Zero()) {}
};

struct SphericalJointDescription {
	string name;
	int index;
	int w_index;

	SphericalJointDescription(const string name, const int index,
							  const int w_index)
		: name(name), index(index), w_index(w_index) {}
};

struct SvdData {
	Eigen::MatrixXd U;
	Eigen::VectorXd s;
	Eigen::MatrixXd V;
};

enum ContactType { PointContact, SurfaceContact };

class ContactModel {
	friend class Sai2Model;

public:
	ContactModel() = delete;
	ContactModel(const string& link_name, const Vector3d& pos,
				 const Matrix3d& orientation, const ContactType& contact_type) {
		_link_name = link_name;
		_contact_position = pos;
		_contact_orientation = orientation;
		_contact_type = contact_type;
	}
	~ContactModel() = default;

private:
	/// \brief name of the link at which the contact occurs
	string _link_name;

	/// \brief the position of the contact in the link. if possible, it should
	/// be the geometrical center of the contact zone
	// (center of the line if line contact, center of surface if surface
	// contact, contact point if point contact)
	Vector3d _contact_position;

	/// \brief orientation of contact. Assumes the constrained direction is z.
	/// If there is a line contact, assumes the free rotation is along the x
	/// axis
	Matrix3d _contact_orientation;

	/// \brief number of constrained rotations for the contact. 0 means point
	/// contact, 1 means line contact with line aligned with contact frame X
	/// axis, 2 means plane contact
	ContactType _contact_type;

	/// \brief the directions to be actively controlled in the order [fx, fy,
	/// fz, mx, my, mz] in local frame
	// Vector6bool _active_directions;
};

class Sai2Model {
public:
	Sai2Model(const string path_to_model_file, bool verbose = false);
	~Sai2Model();

	// disallow empty, copy and asssign constructors
	Sai2Model() = delete;
	Sai2Model(Sai2Model const&) = delete;
	Sai2Model& operator=(Sai2Model const&) = delete;

	// getter and setter for joint positions
	const Eigen::VectorXd& q() const { return _q; }
	void setQ(const Eigen::VectorXd& q);

	// getter and setter for joint velocities
	const Eigen::VectorXd& dq() const { return _dq; }
	void setDq(const Eigen::VectorXd& dq);

	// setter and getter for spherical joint by name
	const Eigen::Quaterniond sphericalQuat(const std::string& joint_name) const;
	void setSphericalQuat(const std::string& joint_name,
						  Eigen::Quaterniond quat);

	// getter for joint accelerations
	const Eigen::VectorXd& ddq() const { return _ddq; }
	void setDdq(const Eigen::VectorXd& ddq);

	// getter for mass matrix
	const Eigen::MatrixXd& M() const { return _M; }
	const Eigen::MatrixXd& MInv() const { return _M_inv; }

	// getter and setter for world gravity
	const Eigen::Vector3d worldGravity() const {
		return _T_world_robot.linear() * _rbdl_model->gravity;
	}
	void setWorldGravity(const Vector3d& world_gravity) {
		_rbdl_model->gravity =
			_T_world_robot.linear().transpose() * world_gravity;
	}

	// getter for the joint limits
	const std::vector<JointLimit>& jointLimits() const { return _joint_limits; }

	// getter for the spherical joints
	const std::vector<SphericalJointDescription>& sphericalJoints() const {
		return _spherical_joints;
	}

	// getter and setter for robot base transform
	const Eigen::Affine3d& TRobotBase() const { return _T_world_robot; }
	void setTRobotBase(const Affine3d& T);

	// getter for joint names
	std::vector<std::string> jointNames() const;

	bool isLinkInRobot(const std::string& link_name) const;

	/**
	 * @brief      update the kinematics for the current robot configuration.
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
	 * @brief      Gives the joint gravity torques vector of the last updated
	 *             configuration using the model world gravity
	 *
	 * @param      g     Vector to which the joint gravity torques will be
	 *                   written
	 */
	VectorXd jointGravityVector();

	/**
	 * @brief      Gives the joint coriolis and centrifugal forces of the last
	 *             updated configuration
	 *
	 * @param      b     Vector to which the joint coriolis and centrifugal
	 *                   forces will be written
	 */
	VectorXd coriolisForce();

	/**
	 * @brief      Computes the nonlinear effects for the last updated
	 *             configuration
	 *
	 * @param      h     vector to which the centrifugal + coriolis + gravity
	 *                   forces will be written
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
	 * @brief      Full jacobian for link, relative to base (id=0) in the form
	 *             [Jv; Jw] (angular first, linear next) expressed in base frame
	 * (default), world frame or local frame
	 *
	 * @param      link_name    the name of the link where to compute the
	 *                          jacobian
	 * @param      pos_in_link  the position of the point in the link where the
	 *                          jacobian is computed (in local link frame)
	 * @return     J            Jacobian in the form [Jv, Jw]
	 */
	MatrixXd J(const string& link_name,
			   const Vector3d& pos_in_link = Vector3d::Zero()) const;
	MatrixXd JWorldFrame(const string& link_name,
						 const Vector3d& pos_in_link = Vector3d::Zero()) const;
	MatrixXd JLocalFrame(
		const string& link_name, const Vector3d& pos_in_link = Vector3d::Zero(),
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;

	/**
	 * @brief      Velocity jacobian for point on link, relative to base (id=0)
	 *             expressed in base frame (default), world frame or local frame
	 *
	 * @param      J            Matrix to which the jacobian will be written
	 * @param      link_name    the name of the link where to compute the
	 *                          jacobian
	 * @param      pos_in_link  the position of the point in the link where the
	 *                          jacobian is computed (in local link frame)
	 */
	MatrixXd Jv(const string& link_name,
				const Vector3d& pos_in_link = Vector3d::Zero()) const;
	MatrixXd JvWorldFrame(const string& link_name,
						  const Vector3d& pos_in_link = Vector3d::Zero()) const;
	MatrixXd JvLocalFrame(
		const string& link_name, const Vector3d& pos_in_link = Vector3d::Zero(),
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;
	/**
	 * @brief      Angular velocity jacobian for link, relative to base (id=0)
	 *             expressed in base frame (default), world frame or local frame
	 *
	 * @param      J          Matrix to which the jacobian will be written
	 * @param      link_name  the name of the link where to compute the jacobian
	 */
	MatrixXd Jw(const string& link_name) const;
	MatrixXd JwWorldFrame(const string& link_name) const;
	MatrixXd JwLocalFrame(
		const string& link_name,
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;

	/**
	 * @brief      Computes the inverse kinematics to get the robot
	 * configuration that matches a set of points on the robot to desired
	 * positions or desired frames. Uses the underlying RBDL function based on
	 * an iterative computation using the damped Levenberg-Marquardt method
	 * (also known as Damped Least Squares method) with output saturation for
	 * the joint limits. Always returns the value from the last iteration, even
	 * if the IK did not converge.
	 *
	 * @param      q_result                                The resulting robot
	 * configuration
	 * @param[in]  link_names                              List of links that
	 * contain the points to match
	 * @param[in]  point_positions_in_links                List of positions in
	 * the links for the points to match
	 * @param[in]  desired_point_positions_in_robot_frame  The desired point
	 * positions in robot frame
	 */

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
	 * frame expressed in link frame), in base coordinates (default) or world
	 * coordinates. This represents the operator that "moves" the base frame to
	 * the link frame, which also means it represents the coordinate
	 * transform matrix to express a point in base frame, if we know it in link
	 * frame:  p_base = T * p_link
	 *
	 * @param      link_name    name of the link where to compute the
	 *                          transformation matrix
	 * @param      pos_in_link  The position in local body coordinates
	 * @param[in]  rot_in_link  The rot local body coordinates
	 * @return	   T            The transfrom operator from base frame to link
	 * frame, or the coordinate change matrix from link frame to base frame
	 */
	Affine3d transform(
		const string& link_name, const Vector3d& pos_in_link = Vector3d::Zero(),
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;
	Affine3d transformInWorld(
		const string& link_name, const Vector3d& pos_in_link = Vector3d::Zero(),
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;

	Vector6d velocity6d(const string link_name,
						const Vector3d& pos_in_link = Vector3d::Zero()) const;
	Vector6d velocity6dInWorld(
		const string link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/*
	 *Note: Acceleration computations are very sensitive to frames
	 * being correct. So it is safer to call UpdateKinematics before
	 * calling these, unless these are called right after a simulator
	 * integrator state.
	 */
	Vector6d acceleration6d(
		const string link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;
	Vector6d acceleration6dInWorld(
		const string link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief      Position from base to point in link, in base coordinates
	 *             (defalut) or world coordinates
	 *
	 * @param      pos          Vector of position to which the result is
	 *                          written
	 * @param      link_name    name of the link in which is the point where to
	 *                          compute the position
	 * @param      pos_in_link  the position of the point in the link, in local
	 *                          link frame
	 */
	Vector3d position(const string& link_name,
					  const Vector3d& pos_in_link = Vector3d::Zero()) const;
	Vector3d positionInWorld(
		const string& link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief      Velocity of point in link, in base coordinates (defalut) or
	 *             world coordinates
	 *
	 * @param      vel          Vector of velocities to which the result is
	 *                          written
	 * @param      link_name    name of the link in which is the point where to
	 *                          compute the velocity
	 * @param      pos_in_link  the position of the point in the link, in local
	 *                          link frame
	 */
	Vector3d linearVelocity(
		const string& link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;
	Vector3d linearVelocityInWorld(
		const string& link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief      Acceleration of point in link, in base coordinates (defalut)
	 *             or world coordinates
	 *             Note: Acceleration computations are very sensitive to frames
	 *             being correct. So it is safer to call UpdateKinematics before
	 *             calling this, unless this is called right after a simulator
	 *             integrator state.
	 *
	 * @param      accel        Vector of accelerations to which the result is
	 *                          written
	 * @param      link_name    name of the link in which is the point where to
	 *                          compute the velocity
	 * @param      pos_in_link  the position of the point in the link, in local
	 *                          link frame
	 */
	Vector3d linearAcceleration(
		const string& link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;
	Vector3d linearAccelerationInWorld(
		const string& link_name,
		const Vector3d& pos_in_link = Vector3d::Zero()) const;

	/**
	 * @brief      Rotation of a link (possibly a local frame expressed in link
	 *             frame) with respect to base frame (default) or world frame
	 *
	 * @param      rot          Rotation matrix to which the result is written
	 * @param      link_name    name of the link for which to compute the
	 *                          rotation
	 * @param[in]  rot_in_link  Local frame of interest expressed in link frame
	 */
	Matrix3d rotation(const string& link_name,
					  const Matrix3d& rot_in_link = Matrix3d::Identity()) const;
	Matrix3d rotationInWorld(
		const string& link_name,
		const Matrix3d& rot_in_link = Matrix3d::Identity()) const;

	/**
	 * @brief      Angular Velocity of a link (possibly a local frame expressed
	 *             in link frame) with respect to base frame (default) or world
	 *             frame
	 *
	 * @param      avel         Vector to which the result is written
	 * @param      link_name    name of the link for which to compute the
	 *                          rotation
	 * @param[in]  rot_in_link  Local frame of interest expressed in link frame
	 */
	Vector3d angularVelocity(const string& link_name) const;
	Vector3d angularVelocityInWorld(const string& link_name) const;

	/**
	 * @brief      Angular Acceleration of a link (possibly a local frame
	 *             expressed in link frame) with respect to base frame (default)
	 *             or world frame
	 *             Note: Acceleration computations are very sensitive to frames
	 *             being correct. So it is safer to call UpdateKinematics before
	 *             calling this, unless this is called right after a simulator
	 *             integrator state.
	 *
	 * @param      aaccel       Vector to which the result is written
	 * @param      link_name    name of the link for which to compute the
	 *                          rotation
	 * @param[in]  rot_in_link  Local frame of interest expressed in link frame
	 */
	Vector3d angularAcceleration(const string& link_name) const;
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
	 *
	 * @param      robot_com  the returned center of mass position
	 */
	Vector3d comPosition() const;

	/**
	 * @brief      returns the center of mass velocity Jacobian of the robot in
	 *             robot base frame
	 *
	 * @param      Jv_com  the returned center of mass full jacobian
	 */
	MatrixXd comJacobian() const;

	/**
	 * @brief      Computes the operational space matrix corresponding to a
	 *             given Jacobian
	 *
	 * @param      Lambda         Matrix on which the operational space mass
	 *                            matrix will be written
	 * @param      task_jacobian  The jacobian of the task for which we want the
	 *                            op space mass matrix
	 */
	Eigen::MatrixXd taskInertiaMatrix(const MatrixXd& task_jacobian) const;

	/**
	 * @brief      Computes the operational space matrix robust to singularities
	 *
	 * @param      Lambda         Matrix on which the operational space mass
	 *                            matrix will be written
	 * @param      task_jacobian  The jacobian of the task for which we want the
	 *                            op space mass matrix
	 */
	MatrixXd taskInertiaMatrixWithPseudoInv(
		const MatrixXd& task_jacobian) const;

	/**
	 * @brief      Computes the dynamically consistent inverse of the jacobian
	 *             for a given task. Recomputes the task inertia at each call
	 *
	 * @param      Jbar           Matrix to which the dynamically consistent
	 *                            inverse will be written
	 * @param[in]  task_jacobian  The task jacobian
	 */
	MatrixXd dynConsistentInverseJacobian(const MatrixXd& task_jacobian) const;

	/**
	 * @brief      Computes the dynamically consistent nullspace of the jacobian
	 *
	 * @param[in]  task_jacobian  The task jacobian
	 */
	MatrixXd nullspaceMatrix(const MatrixXd& task_jacobian) const;

	/**
	 * @brief Computes the operational spce matrices (task inertia, dynamically
	 *consistent inverse of the jacobian and nullspace) for a given task. More
	 *efficient than calling the three individual functions.
	 *
	 * @param[in]  task_jacobian  Task jacobian
	 */
	OpSpaceMatrices operationalSpaceMatrices(
		const MatrixXd& task_jacobian) const;

	/**
	 @brief      Adds an environmental (or manipulation) contact to the desired
	 link at the desired contact frame. There can only be one contact per link

	 @param[in]  link               link at which the contact happens
	 @param[in]  pos_in_link        position of the contact in the link in local
									link frame
	 @param[in]  orientation        orientation of the contact frame in the link
									frame. Z axis needs to be aligned with the
									constrained direction of motion (surface
									normal) if line contact, the free rotation
									should be around the X axis
	 @param[in]  contact_type       The contact type (surface or point)
	 @param[in]  active_directions  The active directions of contact (used for
									environemntal contacts only to compute the
									free floating model)
	*/
	void addEnvironmentalContact(
		const string link, const Vector3d pos_in_link = Vector3d::Zero(),
		const Matrix3d orientation = Matrix3d::Identity(),
		const ContactType contact_type = ContactType::SurfaceContact);
	void addManipulationContact(
		const string link, const Vector3d pos_in_link = Vector3d::Zero(),
		const Matrix3d orientation = Matrix3d::Identity(),
		const ContactType contact_type = ContactType::SurfaceContact);

	/**
	 * @brief      Updates the environmental (or manipulation) contact frame or
	 *             type at the given link
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
	void updateManipulationContact(const string link,
								   const Vector3d pos_in_link,
								   const Matrix3d orientation,
								   const ContactType contact_type);

	/**
	 * @brief      deletes the contact at a given link
	 *
	 * @param      link_name  the link at which we want to delete the contact
	 */
	void deleteEnvironmentalContact(const string link_name);
	void deleteManipulationContact(const string link_name);

	/**
	 * @brief      Computes the manipulation grasp matrix that converts contact
	 *             forces expressed in robot frame (default), world frame or
	 *             local contact frames into the resultant force expressed in
	 *             robot frame (default) or world frame, and the internal froces
	 *             and moments
	 *
	 * @param      G             The grasp matrix to be populated
	 * @param      R             The rotation (useful only if 2 contact points
	 *                           to get the direction between these contacts as
	 *                           x vector of the rotation)
	 * @param[in]  center_point  The position (in robot or world frame) of the
	 *                           point on which we resolve the resultant forces
	 */
	GraspMatrixData manipulationGraspMatrix(
		const Vector3d& center_point,
		const bool resultant_in_world_frame = false,
		const bool contact_forces_in_local_frames = false) const;
	GraspMatrixData manipulationGraspMatrixAtGeometricCenter(
		const bool resultant_in_world_frame = false,
		const bool contact_forces_in_local_frames = false) const;

	/**
	 * @brief      Computes the environmental grasp matrix that converts contact
	 *             forces expressed in robot frame (default), world frame or
	 *             local contact frames into the resultant force expressed in
	 *             robot frame (default) or world frame, and the internal froces
	 *             and moments
	 *
	 * @param      G             The grasp matrix to be populated
	 * @param      R             The rotation (useful only if 2 contact points
	 *                           to get the direction between these contacts as
	 *                           x vector of the rotation)
	 * @param[in]  center_point  The position (in robot or world frame) of the
	 *                           point on which we resolve the resultant forces
	 */
	GraspMatrixData environmentalGraspMatrix(
		const Vector3d& center_point,
		const bool resultant_in_world_frame = false,
		const bool contact_forces_in_local_frames = false) const;
	GraspMatrixData environmentalGraspMatrixAtGeometricCenter(
		const bool resultant_in_world_frame = false,
		const bool contact_forces_in_local_frames = false) const;

	/**
	 * @brief displays the joints or links of the robot with the corresponding
	 * numbers
	 */
	void displayJoints();
	void displayLinks();
	
	/**
	 * @brief Computes the joint selection matrix for the joints leading up to the link
	 * 
	 * @param link_name 
	 * @return MatrixXd 
	 */
	MatrixXd linkDependency(const std::string& link_name);

	/**
	 * @brief   Computes ABA
	 * 
	 */
	void forwardDynamics(VectorXd& ddq, const VectorXd& tau);

	/**
	 * @brief Computes \dot{J}\dot{q} 
	 * 
	 * @param link_name 
	 * @param pos_in_link 
	 * @param update_kinematics 
	 * @return Vector6d 
	 */
	Vector6d jDotQDot(const string& link_name, 
					  const Vector3d& pos_in_link = Vector3d::Zero(), 
					  const bool update_kinematics = false);

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

	/// \brief internal rbdl model
	RigidBodyDynamics::Model* _rbdl_model;

	/// \brief Joint positions. Note: _q size can differ from dof() since
	/// spherical joints use quaternions.
	VectorXd _q;

	/// \brief Joint velocities
	VectorXd _dq;

	/// \brief Joint accelerations
	VectorXd _ddq;

	/// \brief Mass Matrix
	MatrixXd _M;

	/// \brief Inverse of the mass matrix
	MatrixXd _M_inv;

	/// \brief List of active contacts between robot and environment
	vector<ContactModel> _environmental_contacts;

	/// \brief List of active contacts between robot end effectors and
	/// manipulated objects
	vector<ContactModel> _manipulation_contacts;

	/// \brief number of Dof of robot
	int _dof;

	/// \brief number of values required for robot joint positions description
	int _q_size;

	/// \brief Transform from world coordinates to robot base coordinates
	Affine3d _T_world_robot;

	/// \brief map from joint id to joint names
	map<int, string> _joint_id_to_names_map;

	/// \brief map from joint names to child link names
	map<string, string> _joint_names_to_child_link_names_map;

	/// \brief vector of spherical joints
	vector<SphericalJointDescription> _spherical_joints;

	/// \brief map from joint names to joint id (with rbdl indexing)
	map<string, int> _joint_names_to_id_map;

	/// \brief map from link names to link id (with rbdl indexing)
	map<string, int> _link_names_to_id_map;

	/// \brief joint limits for positions, velocity and torque, parsed from URDF
	vector<JointLimit> _joint_limits;
};

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
						  const double& tolerance = 1e-3);

/**
 * @brief Computes the svd data for a matrix
 * 
 * @param matrix 	the input matrix
 * @return SvdData 	data structure containing U, S, and Vs
 */
SvdData matrixSvd(const MatrixXd& matrix);

/**
 * @brief      Gives orientation error from rotation matrices
 *
 * @param      delta_phi            Vector on which the orientation error will
 *                                  be written
 * @param      desired_orientation  desired orientation rotation matrix
 * @param      current_orientation  current orientation matrix
 */
Vector3d orientationError(const Matrix3d& desired_orientation,
						  const Matrix3d& current_orientation);

/**
 * @brief      Gives orientation error from quaternions
 *
 * @param      delta_phi            Vector on which the orientation error will
 *                                  be written
 * @param      desired_orientation  desired orientation quaternion
 * @param      current_orientation  current orientation quaternion
 */
Vector3d orientationError(const Quaterniond& desired_orientation,
						  const Quaterniond& current_orientation);

/// \brief compute the cross product operator of a 3d vector
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
 * @param      G                      The grasp matrix that is going to be
 *                                    populated
 * @param      R                      the rotation matrix between the base
 *                                    frame and the frame attached to the object
 *                                    (useful when 2 contacts only)
 * @param      geometric_center       The position (in base frame) of the
 *                                    geometric center (found and returned by
 *                                    the function) on which we resolve the
 *                                    resultant forces and moments
 * @param[in]  contact_locations      The contact locations
 * @param[in]  constrained_rotations  The constrained rotations
 */
GraspMatrixData graspMatrixAtGeometricCenter(
	const vector<Vector3d>& contact_locations,
	const vector<ContactType>& contact_types);

} /* namespace Sai2Model */

#endif /* SAI2MODEL_H_ */

/*
 * Sai2Model.h
 *
 *  Created on: Dec 14, 2016
 *      Author: Mikael Jorda
 */

#ifndef SAI2MODEL_H_
#define SAI2MODEL_H_

#include <rbdl/Model.h>

using namespace std;
using namespace Eigen;

namespace Eigen
{
typedef Matrix< bool, 6, 1 > Vector6bool;
}

namespace Sai2Model
{

enum ContactType {PointContact, SurfaceContact};

class ContactModel
{
    friend class Sai2Model;
// private:
// protected:
public:
    ContactModel();
    ContactModel(const string link_name, 
                 const Vector3d pos, 
                 const Matrix3d orientation,
                 const ContactType contact_type)
                 // const Vector6bool active_directions)
    {
        _link_name = link_name;
        _contact_position = pos;
        _contact_orientation = orientation;
        _contact_type = contact_type;
        // _active_directions = active_directions;
    }
    ~ContactModel(){}

// protected:
// public:

    /// \brief name of the link at which the contact occurs
    string _link_name;

    /// \brief the position of the contact in the link. if possible, it should be the geometrical center of the contact zone 
    // (center of the line if line contact, center of surface if surface contact, contact point if point contact)
    Vector3d _contact_position;

    /// \brief orientation of contact. Assumes the constrained direction is z. If there is a line contact, assumes the free rotation is along the x axis
    Matrix3d _contact_orientation;

    /// \brief number of constrained rotations for the contact. 0 means point contact, 1 means line contact with line aligned with contact frame X axis, 2 means plane contact
    ContactType _contact_type;

    /// \brief the directions to be actively controlled in the order [fx, fy, fz, mx, my, mz] in local frame
    // Vector6bool _active_directions;
};

class Sai2Model
{
public:
    // Sai2Model ();
    Sai2Model (const string path_to_model_file, 
               bool verbose = true, 
               const Affine3d T_world_robot = Affine3d::Identity(),
               const Vector3d world_gravity = Vector3d(0.0,0.0,-9.81));
    ~Sai2Model ();


    /**
     * @brief      update the kinematics for the current robot configuration.
     */
    void updateKinematics();

    /**
     * @brief      update the dynamics for the current robot configuration.
     */
    void updateDynamics();

    /**
     * @brief      update the inverse inertia matrix.
     */
    void updateInverseInertia();

    /**
     * @brief      update the kinematics and dynamics for the current robot configuration. Effectively calls
     *             updateKinematics and updateDynamics
     */
    void updateModel();

    /**
     * @brief      update the kinematics.
     * @param      update_frame     Whether frame locations should be updated
     * @param      update_link_velocities Whether link linear/angular velocities should
     *                                      be updated
     * @param      update_link_acceleration Whether link linear/angular accelerations
     *                                      should be updated
     * @param      use_ddq Whether link accelerations should include J*ddq terms or not
     */
    void updateKinematicsCustom(bool update_frame=true,
                                bool update_link_velocities=true,
                                bool update_link_acceleration=true, //this does not apply gravity
                                bool use_ddq=true);

    /**
     * @brief      returns the number of degrees of freedom of the robot
     *
     * @return     number of dof of robot
     */
    int dof();

    /**
     * @brief      returns the number of values required for robot joint
     *             positions description. equals dof unless spherical or floating
     *             joints are present.
     * 
     * @return     number of values required for robot joint positions description
     */
    int q_size();

    /**
     * @brief      Gives the joint gravity torques vector of the last updated
     *             configuration using the model world gravity
     *
     * @param      g     Vector to which the joint gravity torques will be
     *                   written
     */
    void gravityVector(VectorXd& g);


    /**
     * @brief      Gives the joint gravity torques vector of the last updated
     *             configuration suing a custom world gravity vector
     *
     * @param      g        Vector to which the joint gravity torques will be
     *                      written
     * @param      gravity  the 3d gravity vector of the world in base frame
     */
    void gravityVector(VectorXd& g,
                               const Vector3d& gravity);

    /**
     * @brief      Gives the joint coriolis and centrifugal forces of the last
     *             updated configuration
     *
     * @param      b     Vector to which the joint coriolis and centrifugal
     *                   forces will be written
     */
    void coriolisForce(VectorXd& b);

    /**
     * @brief      Computes the nonlinear effects for the last updated
     *             configuration
     *
     * @param      h     vector to which the centrifugal + coriolis + gravity
     *                   forces will be written
     */
    void coriolisPlusGravity(VectorXd& h);

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
    void modifiedNewtonEuler(VectorXd& tau, 
                                const bool consider_gravity,
                                const VectorXd& q,
                                const VectorXd& dq,
                                const VectorXd& dqa,
                                const VectorXd& ddq);

    /**
     * @brief      Computes the matrix C such that the coriolis and centrifucal forces can be expressed b = C q_dot
     *
     * @param      C     return matrix
     */
    void factorizedChristoffelMatrix(MatrixXd& C);

    /**
     * @brief      Full jacobian for link, relative to base (id=0) in the form
     *             [Jv; Jw] expressed in base frame (default), world frame or
     *             local frame
     *
     * @param      J            Matrix to which the jacobian will be written
     * @param      link_name    the name of the link where to compute the
     *                          jacobian
     * @param      pos_in_link  the position of the point in the link where the
     *                          jacobian is computed (in local link frame)
     */
    void J_0(MatrixXd& J,
                   const string& link_name,
                   const Vector3d& pos_in_link = Vector3d::Zero());
    void J_0WorldFrame(MatrixXd& J,
                   const string& link_name,
                   const Vector3d& pos_in_link = Vector3d::Zero());
    void J_0LocalFrame(MatrixXd& J,
                   const string& link_name,
                   const Vector3d& pos_in_link = Vector3d::Zero(),
                   const Matrix3d& rot_in_link = Matrix3d::Identity());

    /**
     * @brief      Full jacobian for link, relative to base (id=0) in the form
     *             [Jw; Jv] expressed in base frame (default), world frame or
     *             local frame
     *
     * @param      J            Matrix to which the jacobian will be written
     * @param      link_name    the name of the link where to compute the
     *                          jacobian
     * @param      pos_in_link  the position of the point in the link where the
     *                          jacobian is computed (in local link frame)
     */
    void J(MatrixXd& J,
                   const string& link_name,
                   const Vector3d& pos_in_link = Vector3d::Zero());
    void JWorldFrame(MatrixXd& J,
                   const string& link_name,
                   const Vector3d& pos_in_link = Vector3d::Zero());
    void JLocalFrame(MatrixXd& J,
                   const string& link_name,
                   const Vector3d& pos_in_link = Vector3d::Zero(),
                   const Matrix3d& rot_in_link = Matrix3d::Identity());

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
    void Jv(MatrixXd& J,
                    const string& link_name,
                    const Vector3d& pos_in_link = Vector3d::Zero());
    void JvWorldFrame(MatrixXd& J,
                    const string& link_name,
                    const Vector3d& pos_in_link = Vector3d::Zero());
    void JvLocalFrame(MatrixXd& J,
                    const string& link_name,
                    const Vector3d& pos_in_link = Vector3d::Zero(),
                    const Matrix3d& rot_in_link = Matrix3d::Identity());
    /**
     * @brief      Angular velocity jacobian for link, relative to base (id=0)
     *             expressed in base frame (default), world frame or local frame
     *
     * @param      J          Matrix to which the jacobian will be written
     * @param      link_name  the name of the link where to compute the jacobian
     */
    void Jw(MatrixXd& J,
                    const string& link_name);
    void JwWorldFrame(MatrixXd& J,
                    const string& link_name);
    void JwLocalFrame(MatrixXd& J,
                    const string& link_name,
                    const Matrix3d& rot_in_link = Matrix3d::Identity());


    /**
     * @brief      Computes the inverse kinematics to get the robot
     *             configuration that matches a set of points on the robot to
     *             desired positions. Uses the underlying RBDL function based on
     *             an iterative computation using the damped Levenberg-Marquardt
     *             method (also known as Damped Least Squares method)
     *
     * @param      q_result                                The resulting robot configuration
     * @param[in]  link_names                              List of links that contain the points to match
     * @param[in]  point_positions_in_links                List of positions in the links for the points to match
     * @param[in]  desired_point_positions_in_robot_frame  The desired point positions in robot frame
     */
    void computeIK3d(VectorXd& q_result, 
                    const vector<string>& link_names,
                    const vector<Vector3d>& point_positions_in_links,
                    const vector<Vector3d>& desired_point_positions_in_robot_frame);

    /**
     * @brief      Inverse kinematics using a weighted damped Least-Squared
     *             method and considering joint limits by saturating the joint
     *             values to their limits in every iteration of the algorithm
     *
     * @param      q_result                                Resulting robot configuration
     * @param[in]  link_names                              The link names for the points to match
     * @param[in]  point_positions_in_links                The point positions in links
     * @param[in]  desired_point_positions_in_robot_frame  The desired point positions in robot frame
     * @param[in]  q_min                                   Lower joint limits
     * @param[in]  q_max                                   Upper joint limits
     * @param[in]  weights                                 The weights
     */
    void computeIK3d_JL(VectorXd& q_result, 
                    const vector<string>& link_names,
                    const vector<Vector3d>& point_positions_in_links,
                    const vector<Vector3d>& desired_point_positions_in_robot_frame,
                    const VectorXd q_min,
                    const VectorXd q_max,
                    const VectorXd weights
                    );

    /**
     * @brief      Inverse kinematics that matches frames in the robot to
     *             desired configurations (instead of just points) using the
     *             damped least-squared method
     *
     * @param      q_result                                Resulting robot configuration
     * @param[in]  link_names                              The link names for the frames to match
     * @param[in]  frame_in_links                          The frames to match expressed in link frame
     * @param[in]  desired_frame_locations_in_robot_frame  The desired frame locations in robot frame
     */
    void computeIK6d(VectorXd& q_result, 
                    const vector<string>& link_names,
                    const vector<Affine3d>& frame_in_links,
                    const vector<Affine3d>& desired_frame_locations_in_robot_frame);

    /**
     * @brief      transformation from base to link frame (possibly a local frame expressed in link frame), in base
     *             coordinates (default) or world coordinates.
     *
     * @param      T            Transformation matrix to which the result is
     *                          computed
     * @param      link_name    name of the link where to compute the
     *                          transformation matrix
     * @param      pos_in_link  The position in local body coordinates
     * @param[in]  rot_in_link  The rot local body coordinates
     */
    void transform(Affine3d& T,
                           const string& link_name,
                           const Vector3d& pos_in_link = Vector3d::Zero(),
                           const Matrix3d& rot_in_link = Matrix3d::Identity());
    void transformInWorld(Affine3d& T,
                           const string& link_name,
                           const Vector3d& pos_in_link = Vector3d::Zero(),
                           const Matrix3d& rot_in_link = Matrix3d::Identity());

    void velocity6d(VectorXd& vel6d,
                            const string link_name,
                            const Vector3d& pos_in_link = Vector3d::Zero());
    void velocity6dInWorld(VectorXd& vel6d,
                            const string link_name,
                            const Vector3d& pos_in_link = Vector3d::Zero());

    /*
    *Note: Acceleration computations are very sensitive to frames
    * being correct. So it is safer to call UpdateKinematics before
    * calling these, unless these are called right after a simulator
    * integrator state.
    *Note: If these functions are called after calling NonLinearEffects()
    * or other dynamics functions, returned accelerations can include
    * acceleration due to gravity. If this is not desired, first call
    * updateKinematicsCustom(false, false, true, true) which recomputes just the
    * link accelerations in the rbdl model.
    */
    void acceleration6d(VectorXd& vel6d,
                            const string link_name,
                            const Vector3d& pos_in_link = Vector3d::Zero());
    void acceleration6dInWorld(VectorXd& vel6d,
                            const string link_name,
                            const Vector3d& pos_in_link = Vector3d::Zero());

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
    void position(Vector3d& pos,
                          const string& link_name,
                          const Vector3d& pos_in_link = Vector3d::Zero());
    void positionInWorld(Vector3d& pos,
                          const string& link_name,
                          const Vector3d& pos_in_link = Vector3d::Zero());

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
    void linearVelocity(Vector3d& vel,
                          const string& link_name,
                          const Vector3d& pos_in_link = Vector3d::Zero());
    void linearVelocityInWorld(Vector3d& vel,
                          const string& link_name,
                          const Vector3d& pos_in_link = Vector3d::Zero());

    /**
     * @brief      Acceleration of point in link, in base coordinates (defalut)
     *             or world coordinates
     *             Note: Acceleration computations are very sensitive to frames
     *             being correct. So it is safer to call UpdateKinematics before
     *             calling this, unless this is called right after a simulator
     *             integrator state.
     *             Note: If this function is called after calling NonLinearEffects()
     *              or other dynamics functions, returned accelerations can include
     *              acceleration due to gravity. If this is not desired, first call
     *              updateKinematicsCustom(false, false, true, true)
     *
     * @param      accel        Vector of accelerations to which the result is
     *                          written
     * @param      link_name    name of the link in which is the point where to
     *                          compute the velocity
     * @param      pos_in_link  the position of the point in the link, in local
     *                          link frame
     */
    void linearAcceleration(Vector3d& accel,
                              const string& link_name,
                              const Vector3d& pos_in_link = Vector3d::Zero());
    void linearAccelerationInWorld(Vector3d& accel,
                              const string& link_name,
                              const Vector3d& pos_in_link = Vector3d::Zero());

    /**
     * @brief      Rotation of a link (possibly a local frame expressed in link
     *             frame) with respect to base frame (default) or world frame
     *
     * @param      rot          Rotation matrix to which the result is written
     * @param      link_name    name of the link for which to compute the
     *                          rotation
     * @param[in]  rot_in_link  Local frame of interest expressed in link frame
     */
    void rotation(Matrix3d& rot,
                          const string& link_name,
                          const Matrix3d& rot_in_link = Matrix3d::Identity());
    void rotationInWorld(Matrix3d& rot,
                          const string& link_name,
                          const Matrix3d& rot_in_link = Matrix3d::Identity());

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
    void angularVelocity(Vector3d& avel,
                                 const string& link_name,
                                 const Vector3d& pos_in_link = Vector3d::Zero());
    void angularVelocityInWorld(Vector3d& avel,
                                 const string& link_name,
                                 const Vector3d& pos_in_link = Vector3d::Zero());

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
    void angularAcceleration(Vector3d& aaccel,
                                     const string& link_name,
                                     const Vector3d& pos_in_link = Vector3d::Zero());
    void angularAccelerationInWorld(Vector3d& aaccel,
                                     const string& link_name,
                                     const Vector3d& pos_in_link = Vector3d::Zero());

    /**
     * @brief      Gives the link id for a given name with the right indexing
     *             for rbdl
     *
     * @param      link_name  name of the link
     *
     * @return     the link number
     */
    unsigned int linkId(const string& link_name);

    /**
     * @brief      Gives the joint id for a given name with the right indexing
     *             for rbdl
     *
     * @param      joint_name  name of the joint
     *
     * @return     the joint number
     */
    int jointId(const string& joint_name);

    /**
     * @brief      Gives the mass properties of a given link
     *
     * @param      mass            the returned mass value
     * @param      center_of_mass  the position of the center of mass in the
     *                             body's frame
     * @param      inertia         the inertia of the given link
     * @param      link_name       the name of the considered link
     */
    void getLinkMass(double& mass,
                     Vector3d& center_of_mass,
                     Matrix3d& inertia,
                     const string& link_name);

    /**
     * @brief      Gives the mass properties of a given link
     *
     * @param      mass            the returned mass value
     * @param      center_of_mass  the position of the center of mass in the
     *                             body's frame
     * @param      link_name       the name of the considered link
     */
    void getLinkMass(double& mass,
                     Vector3d& center_of_mass,
                     const string& link_name);

    /**
     * @brief      returns the position of the center of mass of the robot in
     *             robot base frame
     *
     * @param      robot_com  the returned center of mass position
     */
    void comPosition(Vector3d& robot_com);

    /**
     * @brief      returns the center of mass velocity Jacobian of the robot in
     *             robot base frame
     *
     * @param      Jv_com  the returned center of mass full jacobian
     */
    void comJacobian(MatrixXd& Jv_com);

    /**
     * @brief      Computes the range space of the Jacobian (potentially
     *             constrained to the nullspace of previous tasks) where the
     *             singular directions have been removed. The computed URange
     *             matrix is a matrix whose columns corresponds to the
     *             directions of the jacobian associated with the singular
     *             values of magnitude higher than than the tolerance
     *
     * @param      URange     Return matrix U
     * @param[in]  J          The jacobian
     * @param[in]  N          The Nullspace of the previous tasks
     * @param[in]  tolerance  The tolerance
     */
    void URangeJacobian(MatrixXd& URange, const MatrixXd& J, const MatrixXd& N, const double tolerance = 1e-3);
    void URangeJacobian(MatrixXd& URange, const MatrixXd& J, const double tolerance = 1e-3);

    /**
     * @brief      Computes the operational space matrix corresponding to a
     *             given Jacobian
     *
     * @param      Lambda         Matrix on which the operational space mass
     *                            matrix will be written
     * @param      task_jacobian  The jacobian of the task for which we want the
     *                            op space mass matrix
     */
    void taskInertiaMatrix(MatrixXd& Lambda,
                           const MatrixXd& task_jacobian);

    /**
     * @brief      Computes the operational space matrix robust to singularities
     *
     * @param      Lambda         Matrix on which the operational space mass
     *                            matrix will be written
     * @param      task_jacobian  The jacobian of the task for which we want the
     *                            op space mass matrix
     */
    void taskInertiaMatrixWithPseudoInv(MatrixXd& Lambda,
                           const MatrixXd& task_jacobian);

    /**
     * @brief      Computes the dynamically consistent inverse of the jacobian
     *             for a given task. Recomputes the task inertia at each call
     *
     * @param      Jbar           Matrix to which the dynamically consistent
     *                            inverse will be written
     * @param[in]  task_jacobian  The task jacobian
     */
    void dynConsistentInverseJacobian(MatrixXd& Jbar,
                                      const MatrixXd& task_jacobian);


    /**
     * @brief      Computes the nullspace matrix for the highest priority task.
     *             Recomputes the dynamically consistent inverse and the task
     *             mass matrix at each call
     *
     * @param      N              Matrix to which the nullspace matrix will be
     *                            written
     * @param[in]  task_jacobian  The task jacobian
     */
    void nullspaceMatrix(MatrixXd& N,
                             const MatrixXd& task_jacobian);

    /**
     * @brief      Computes the nullspace matrix of the task, consistent with
     *             the previous nullspace Recomputes the dynamically consistent
     *             inverse and the task mass matrix at each call
     *
     * @param      N              Matrix to which the nullspace matrix will be
     *                            written
     * @param[in]  task_jacobian  The task jacobian
     * @param[in]  N_prec         The previous nullspace matrix
     */
    void nullspaceMatrix(MatrixXd& N,
                             const MatrixXd& task_jacobian,
                             const MatrixXd& N_prec);

    /**
     * @brief      Computes the operational spce matrices (task inertia,
     *             dynamically consistent inverse of the jacobian and nullspace)
     *             for a given task, for the first task. More efficient than
     *             calling the three individual functions.
     *
     * @param      Lambda         Matrix to which the operational space mass
     *                            matrix will be written
     * @param      Jbar           Matrix to which the dynamically consistent
     *                            inverse of the jacobian will be written
     * @param      N              Matrix to which the nullspace matrix will be
     *                            written
     * @param[in]  task_jacobian  Task jacobian
     */
    void operationalSpaceMatrices(MatrixXd& Lambda, MatrixXd& Jbar, MatrixXd& N,
                                    const MatrixXd& task_jacobian);

    /**
     * @brief      Computes the operational spce matrices (task inertia,
     *             dynamically consistent inverse of the jacobian and nullspace)
     *             for a given task, In the nullspace of the previous task. More
     *             efficient than calling the three individual functions.
     *
     * @param      Lambda         Matrix to which the operational space mass
     *                            matrix will be written
     * @param      Jbar           Matrix to which the dynamically consistent
     *                            inverse of the jacobian will be written
     * @param      N              Matrix to which the nullspace matrix will be
     *                            written
     * @param[in]  task_jacobian  Task jacobian
     * @param[in]  N_prec         Previous nullspace matrix
     */
    void operationalSpaceMatrices(MatrixXd& Lambda, MatrixXd& Jbar, MatrixXd& N,
                                    const MatrixXd& task_jacobian,
                                    const MatrixXd& N_prec);



    /**
     @brief      Adds an environmental (or manipulation) contact to the desired link at the
                 desired contact frame. There can only be one contact per link
    
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
    void addEnvironmentalContact(const string link, 
                    const Vector3d pos_in_link = Vector3d::Zero(),
                    const Matrix3d orientation = Matrix3d::Identity(),
                    const ContactType contact_type = ContactType::SurfaceContact);
    void addManipulationContact(const string link, 
                    const Vector3d pos_in_link = Vector3d::Zero(),
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
    void manipulationGraspMatrix(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     const Vector3d center_point);
    void manipulationGraspMatrixInWorld(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     const Vector3d center_point);
    void manipulationGraspMatrixLocalContactForces(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     const Vector3d center_point);
    void manipulationGraspMatrixLocalContactForcesToWorld(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     const Vector3d center_point);

    /**
     * @brief      Computes the manipulation grasp matrix that converts contact
     *             forces expressed in robot frame (default), world frame or
     *             local contact frames into the resultant force expressed in
     *             robot frame (default) or world frame, and the internal froces
     *             and moments
     *
     * @param      G                 The grasp matrix to be populated
     * @param      R                 The rotation (useful only if 2 contact
     *                               points to get the direction between these
     *                               contacts as x vector of the rotation)
     * @param      geometric_center  The geometric center that is computed and
     *                               at which the resultant forces and moments
     *                               are resolved
     */
    void manipulationGraspMatrixAtGeometricCenter(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     Vector3d& geometric_center);
    void manipulationGraspMatrixAtGeometricCenterInWorld(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     Vector3d& geometric_center);
    void manipulationGraspMatrixAtGeometricCenterLocalContactForces(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     Vector3d& geometric_center);
    void manipulationGraspMatrixAtGeometricCenterLocalContactForcesToWorld(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     Vector3d& geometric_center);

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
    void environmentalGraspMatrix(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     const Vector3d center_point);
    void environmentalGraspMatrixInWorld(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     const Vector3d center_point);
    void environmentalGraspMatrixLocalContactForces(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     const Vector3d center_point);
    void environmentalGraspMatrixLocalContactForcesToWorld(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     const Vector3d center_point);

    /**
     * @brief      Computes the environmental grasp matrix that converts contact
     *             forces expressed in robot frame (default), world frame or
     *             local contact frames into the resultant force expressed in
     *             robot frame (default) or world frame, and the internal froces
     *             and moments
     *
     * @param      G                 The grasp matrix to be populated
     * @param      R                 The rotation (useful only if 2 contact
     *                               points to get the direction between these
     *                               contacts as x vector of the rotation)
     * @param      geometric_center  The geometric center that is computed and
     *                               at which the resultant forces and moments
     *                               are resolved
     */
    void environmentalGraspMatrixAtGeometricCenter(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     Vector3d& geometric_center);
    void environmentalGraspMatrixAtGeometricCenterInWorld(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     Vector3d& geometric_center);
    void environmentalGraspMatrixAtGeometricCenterLocalContactForces(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     Vector3d& geometric_center);
    void environmentalGraspMatrixAtGeometricCenterLocalContactForcesToWorld(MatrixXd& G,
                     MatrixXd& G_inv,
                     Matrix3d& R,
                     Vector3d& geometric_center);


    /**
     * @brief displays the joints or links of the robot with the corresponding numbers
     */
    void displayJoints();
    void displayLinks();

    /// \brief internal rbdl model
    RigidBodyDynamics::Model* _rbdl_model;

    /// \brief Joint positions. Note: _q size can differ from dof() since spherical joints use quaternions.
    VectorXd _q;

    /// \brief Joint velocities
    VectorXd _dq;

    /// \brief Joint accelerations
    VectorXd _ddq;

    /// \brief Mass Matrix
    MatrixXd _M;

    /// \brief Inverse of the mass matrix
    MatrixXd _M_inv;

    /// \brief gravity in base frame
    Vector3d _world_gravity;

    /// \brief List of active contacts between robot and environment
    vector<ContactModel> _environmental_contacts;

    /// \brief List of active contacts between robot end effectors and manipulated objects
    vector<ContactModel> _manipulation_contacts;

    /// \brief number of Dof of robot
    int _dof;

    /// \brief number of values required for robot joint positions description
    int _q_size;

    /// \brief Transform from world coordinates to robot base coordinates
    Affine3d _T_world_robot;

// protected:
    /// \brief map from joint names to joint id
    map<string,int> _joint_names_map;

    /// \brief map from link names to link id
    map<string,int> _link_names_map;
};

 /**
  * @brief      Gives orientation error from rotation matrices
  *
  * @param      delta_phi            Vector on which the orientation error will
  *                                  be written
  * @param      desired_orientation  desired orientation rotation matrix
  * @param      current_orientation  current orientation matrix
  */
void orientationError(Vector3d& delta_phi,
                      const Matrix3d& desired_orientation,
                      const Matrix3d& current_orientation);


/**
 * @brief      Gives orientation error from quaternions
 *
 * @param      delta_phi            Vector on which the orientation error will
 *                                  be written
 * @param      desired_orientation  desired orientation quaternion
 * @param      current_orientation  current orientation quaternion
 */
void orientationError(Vector3d& delta_phi,
                      const Quaterniond& desired_orientation,
                      const Quaterniond& current_orientation);


/// \brief compute the cross product operator of a 3d vector
Matrix3d CrossProductOperator(const Vector3d& v);

/**
 * @brief      Computes the grasp matrix and its inverse in the cases where
 *             there are 2, 3 or 4 contacts. the external forces and moments are
 *             assumed to be in world frame. For 2 contact points, the output
 *             resultant (first 6 lines) is given in world frame, and the output
 *             internal tension and moments are given in local frame, and the
 *             description of the local frame is given by R. For 3 and 4
 *             contacts, the output quantities are given in world frame. The
 *             convention for the output is the following order : support
 *             forces, support moments, internal tensions, internal moments the
 *             internal tensions are given in the order 1-2, 1-3, 2-3 in the 3
 *             contact case and 1-2, 1-3, 1-4, 2-3, 2-4, 3-4 in the 4 contact
 *             case. Line contacts are not yet supported.
 *
 * @param      G                  The grasp matrix that is going to be populated
 * @param      G_inv              The inverse of the grasp matrix
 * @param      R                  the rotation matrix between the world frame
 *                                and the frame attached to the object (useful
 *                                when 2 contacts only)
 * @param      center_point       The position (in world frame) of the point on
 *                                which we resolve the resultant forces and
 *                                moments
 * @param[in]  contact_locations  The contact locations
 * @param[in]  contact_types      The contact types
 * @param[in]  constrained_rotations  The constrained rotations
 */
void graspMatrix(MatrixXd& G,
                 MatrixXd& G_inv,
                 Matrix3d& R,
                 const Vector3d center_point,
                 const vector<Vector3d>& contact_locations,
                 const vector<ContactType> contact_types);

/**
 * @brief      Computes the grasp matrix in the cases where there are 2, 3 or 4
 *             contact points. The resultant is given at the geometric center of
 *             the virtual linkage. the external forces and moments are assumed
 *             to be in world frame
 *
 * @param      G                      The grasp matrix that is going to be
 *                                    populated
 * @param      R                      the rotation matrix between the world
 *                                    frame and the frame attached to the object
 *                                    (useful when 2 contacts only)
 * @param      geometric_center       The position (in world frame) of the
 *                                    geometric center (found and returned by
 *                                    the function) on which we resolve the
 *                                    resultant forces and moments
 * @param[in]  contact_locations      The contact locations
 * @param[in]  constrained_rotations  The constrained rotations
 */
void graspMatrixAtGeometricCenter(MatrixXd& G,
                 MatrixXd& G_inv,
                 Matrix3d& R,
                 Vector3d& geometric_center,
                 const vector<Vector3d>& contact_locations,
                 const vector<ContactType> contact_types);

} /* namespace Sai2Model */

#endif /* RBDLMODEL_H_ */

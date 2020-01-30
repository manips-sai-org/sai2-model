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

namespace Sai2Model
{

enum ContactNature {PointContact, SurfaceContact};

class ContactModel
{
    friend class Sai2Model;
// private:
public:
    ContactModel();
    ContactModel(const std::string link_name, 
                 const Eigen::Vector3d pos, 
                 const Eigen::Matrix3d orientation,
                 const int constained_rot)
    {
        _link_name = link_name;
        _contact_position = pos;
        _contact_orientation = orientation;
        _constrained_rotations = constained_rot;
    }
    ~ContactModel(){}

public:

    /// \brief name of the link at which the contact occurs
    std::string _link_name;

    /// \brief the position of the contact in the link. if possible, it should be the geometrical center of the contact zone 
    // (center of the line if line contact, center of surface if surface contact, contact point if point contact)
    Eigen::Vector3d _contact_position;

    /// \brief orientation of contact. Assumes the constrained direction is z. If there is a line contact, assumes the free rotation is along the x axis
    Eigen::Matrix3d _contact_orientation;

    /// \brief number of constrained rotations for the contact. 0 means point contact, 1 means line contact with line aligned with contact frame X axis, 2 means plane contact
    int _constrained_rotations;
};

class Sai2Model
{
public:
    // Sai2Model ();
    Sai2Model (const std::string path_to_model_file, 
               bool verbose=true, 
               const Eigen::Affine3d T_world_robot = Eigen::Affine3d::Identity(),
               const Eigen::Vector3d world_gravity = Eigen::Vector3d(0.0,0.0,-9.81));
    ~Sai2Model ();


    /**
     * @brief      update the kinematics.
     */
    void updateKinematics();

    /**
     * @brief      update the dynamics.
     */
    void updateDynamics();

    /**
     * @brief      update the inverse inertia matrix.
     */
    void updateInverseInertia();

    /**
     * @brief      update the kinematics and dynamics. Effectively calls
     *             updateKinematics and updateDynamics
     */
    void updateModel();

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
    void gravityVector(Eigen::VectorXd& g);


    /**
     * @brief      Gives the joint gravity torques vector of the last updated
     *             configuration suing a custom world gravity vector
     *
     * @param      g        Vector to which the joint gravity torques will be
     *                      written
     * @param      gravity  the 3d gravity vector of the world in base frame
     */
    void gravityVector(Eigen::VectorXd& g,
                               const Eigen::Vector3d& gravity);

    /**
     * @brief      Gives the joint coriolis and centrifugal forces of the last
     *             updated configuration
     *
     * @param      b     Vector to which the joint coriolis and centrifugal
     *                   forces will be written
     */
    void coriolisForce(Eigen::VectorXd& b);

    /**
     * @brief      Computes the nonlinear effects for the last updated
     *             configuration
     *
     * @param      h     vector to which the centrifugal + coriolis + gravity
     *                   forces will be written
     */
    void coriolisPlusGravity(Eigen::VectorXd& h);

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
    void modifiedNewtonEuler(Eigen::VectorXd& tau, 
                                const bool consider_gravity,
                                const Eigen::VectorXd& q,
                                const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& dqa,
                                const Eigen::VectorXd& ddq);
    void factorizedChristoffelMatrix(Eigen::MatrixXd& C);

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
    void J_0(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link);
    void J_0WorldFrame(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link);
    void J_0LocalFrame(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link,
                   const Eigen::Matrix3d& rot_in_link = Eigen::Matrix3d::Identity());

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
    void J(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link);
    void JWorldFrame(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link);
    void JLocalFrame(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link,
                   const Eigen::Matrix3d& rot_in_link = Eigen::Matrix3d::Identity());

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
    void Jv(Eigen::MatrixXd& J,
                    const std::string& link_name,
                    const Eigen::Vector3d& pos_in_link);
    void JvWorldFrame(Eigen::MatrixXd& J,
                    const std::string& link_name,
                    const Eigen::Vector3d& pos_in_link);
    void JvLocalFrame(Eigen::MatrixXd& J,
                    const std::string& link_name,
                    const Eigen::Vector3d& pos_in_link,
                    const Eigen::Matrix3d& rot_in_link = Eigen::Matrix3d::Identity());
    /**
     * @brief      Angular velocity jacobian for link, relative to base (id=0)
     *             expressed in base frame (default), world frame or local frame
     *
     * @param      J          Matrix to which the jacobian will be written
     * @param      link_name  the name of the link where to compute the jacobian
     */
    void Jw(Eigen::MatrixXd& J,
                    const std::string& link_name);
    void JwWorldFrame(Eigen::MatrixXd& J,
                    const std::string& link_name);
    void JwLocalFrame(Eigen::MatrixXd& J,
                    const std::string& link_name,
                    const Eigen::Matrix3d& rot_in_link = Eigen::Matrix3d::Identity());


    void computeIK3d(VectorXd& q_result, 
                    const vector<string>& link_names,
                    const vector<Vector3d>& point_positions_in_links,
                    const vector<Vector3d>& desired_point_positions_in_robot_frame);

    void computeIK3d_JL(VectorXd& q_result, 
                    const vector<string>& link_names,
                    const vector<Vector3d>& point_positions_in_links,
                    const vector<Vector3d>& desired_point_positions_in_robot_frame,
                    const VectorXd q_min,
                    const VectorXd q_max,
                    const VectorXd weights
                    );

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
    void transform(Eigen::Affine3d& T,
                           const std::string& link_name,
                           const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero(),
                           const Eigen::Matrix3d& rot_in_link = Eigen::Matrix3d::Identity());
    void transformInWorld(Eigen::Affine3d& T,
                           const std::string& link_name,
                           const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero(),
                           const Eigen::Matrix3d& rot_in_link = Eigen::Matrix3d::Identity());

    void velocity6d(Eigen::VectorXd& vel6d,
                            const std::string link_name,
                            const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
    void velocity6dInWorld(Eigen::VectorXd& vel6d,
                            const std::string link_name,
                            const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());

    void acceleration6d(Eigen::VectorXd& vel6d,
                            const std::string link_name,
                            const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
    void acceleration6dInWorld(Eigen::VectorXd& vel6d,
                            const std::string link_name,
                            const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());

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
    void position(Eigen::Vector3d& pos,
                          const std::string& link_name,
                          const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
    void positionInWorld(Eigen::Vector3d& pos,
                          const std::string& link_name,
                          const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());

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
    void linearVelocity(Eigen::Vector3d& vel,
                          const std::string& link_name,
                          const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
    void linearVelocityInWorld(Eigen::Vector3d& vel,
                          const std::string& link_name,
                          const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());

    /**
     * @brief      Acceleration of point in link, in base coordinates (defalut)
     *             or world coordinates
     *
     * @param      accel        Vector of accelerations to which the result is
     *                          written
     * @param      link_name    name of the link in which is the point where to
     *                          compute the velocity
     * @param      pos_in_link  the position of the point in the link, in local
     *                          link frame
     */
    void linearAcceleration(Eigen::Vector3d& accel,
                              const std::string& link_name,
                              const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
    void linearAccelerationInWorld(Eigen::Vector3d& accel,
                              const std::string& link_name,
                              const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());

    /**
     * @brief      Rotation of a link (possibly a local frame expressed in link
     *             frame) with respect to base frame (default) or world frame
     *
     * @param      rot          Rotation matrix to which the result is written
     * @param      link_name    name of the link for which to compute the
     *                          rotation
     * @param[in]  rot_in_link  Local frame of interest expressed in link frame
     */
    void rotation(Eigen::Matrix3d& rot,
                          const std::string& link_name,
                          const Eigen::Matrix3d& rot_in_link = Eigen::Matrix3d::Identity());
    void rotationInWorld(Eigen::Matrix3d& rot,
                          const std::string& link_name,
                          const Eigen::Matrix3d& rot_in_link = Eigen::Matrix3d::Identity());

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
    void angularVelocity(Eigen::Vector3d& avel,
                                 const std::string& link_name,
                                 const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
    void angularVelocityInWorld(Eigen::Vector3d& avel,
                                 const std::string& link_name,
                                 const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());

    /**
     * @brief      Angular Acceleration of a link (possibly a local frame
     *             expressed in link frame) with respect to base frame (default)
     *             or world frame
     *
     * @param      aaccel       Vector to which the result is written
     * @param      link_name    name of the link for which to compute the
     *                          rotation
     * @param[in]  rot_in_link  Local frame of interest expressed in link frame
     */
    void angularAcceleration(Eigen::Vector3d& aaccel,
                                     const std::string& link_name,
                                     const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());
    void angularAccelerationInWorld(Eigen::Vector3d& aaccel,
                                     const std::string& link_name,
                                     const Eigen::Vector3d& pos_in_link = Eigen::Vector3d::Zero());

    /**
     * @brief      Gives the link id for a given name with the right indexing
     *             for rbdl
     *
     * @param      link_name  name of the link
     *
     * @return     the link number
     */
    unsigned int linkId(const std::string& link_name);

    /**
     * @brief      Gives the joint id for a given name with the right indexing
     *             for rbdl
     *
     * @param      joint_name  name of the joint
     *
     * @return     the joint number
     */
    int jointId(const std::string& joint_name);

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
                     Eigen::Vector3d& center_of_mass,
                     Eigen::Matrix3d& inertia,
                     const std::string& link_name);

    /**
     * @brief      Gives the mass properties of a given link
     *
     * @param      mass            the returned mass value
     * @param      center_of_mass  the position of the center of mass in the
     *                             body's frame
     * @param      link_name       the name of the considered link
     */
    void getLinkMass(double& mass,
                     Eigen::Vector3d& center_of_mass,
                     const std::string& link_name);

    /**
     * @brief      returns the position of the center of mass of the robot in
     *             robot base frame
     *
     * @param      robot_com  the returned center of mass position
     */
    void comPosition(Eigen::Vector3d& robot_com);

    /**
     * @brief      returns the center of mass velocity Jacobian of the robot in
     *             robot base frame
     *
     * @param      Jv_com  the returned center of mass full jacobian
     */
    void comJacobian(Eigen::MatrixXd& Jv_com);

    /**
     * @brief      Computes the operational space matrix corresponding to a
     *             given Jacobian
     *
     * @param      Lambda         Matrix on which the operational space mass
     *                            matrix will be written
     * @param      task_jacobian  The jacobian of the task for which we want the
     *                            op space mass matrix
     */
    void taskInertiaMatrix(Eigen::MatrixXd& Lambda,
                           const Eigen::MatrixXd& task_jacobian);

    /**
     * @brief      Computes the operational space matrix robust to singularities
     *
     * @param      Lambda         Matrix on which the operational space mass
     *                            matrix will be written
     * @param      task_jacobian  The jacobian of the task for which we want the
     *                            op space mass matrix
     */
    void taskInertiaMatrixWithPseudoInv(Eigen::MatrixXd& Lambda,
                           const Eigen::MatrixXd& task_jacobian);

    /**
     * @brief      Computes the dynamically consistent inverse of the jacobian
     *             for a given task. Recomputes the task inertia at each call
     *
     * @param      Jbar           Matrix to which the dynamically consistent
     *                            inverse will be written
     * @param[in]  task_jacobian  The task jacobian
     */
    void dynConsistentInverseJacobian(Eigen::MatrixXd& Jbar,
                                      const Eigen::MatrixXd& task_jacobian);


    /**
     * @brief      Computes the nullspace matrix for the highest priority task.
     *             Recomputes the dynamically consistent inverse and the task
     *             mass matrix at each call
     *
     * @param      N              Matrix to which the nullspace matrix will be
     *                            written
     * @param[in]  task_jacobian  The task jacobian
     */
    void nullspaceMatrix(Eigen::MatrixXd& N,
                             const Eigen::MatrixXd& task_jacobian);

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
    void nullspaceMatrix(Eigen::MatrixXd& N,
                             const Eigen::MatrixXd& task_jacobian,
                             const Eigen::MatrixXd& N_prec);

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
    void operationalSpaceMatrices(Eigen::MatrixXd& Lambda, Eigen::MatrixXd& Jbar, Eigen::MatrixXd& N,
                                    const Eigen::MatrixXd& task_jacobian);

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
    void operationalSpaceMatrices(Eigen::MatrixXd& Lambda, Eigen::MatrixXd& Jbar, Eigen::MatrixXd& N,
                                    const Eigen::MatrixXd& task_jacobian,
                                    const Eigen::MatrixXd& N_prec);



    /**
     @brief      Adds an environmental contact to the desired link at the
                 desired contact frame
    
     @param[in]  link                     link at which the contact happens
     @param[in]  pos_in_link              position of the contact in the link in
                                          local link frame
     @param[in]  orientation              orientation of the contact frame in
                                          the link frame. Z axis needs to be
                                          aligned with the constrained direction
                                          of motion (surface normal) if line
                                          contact, the free rotation should be
                                          around the X axis
     @param[in]  constraints_in_rotation  The constraints in rotation. 0 = point
                                          contact, 1 = line contact, 2 = plane
                                          contact
    */
    void addEnvironmentalContact(const std::string link, 
                    const Eigen::Vector3d pos_in_link,
                    const Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity(),
                    const int constraints_in_rotation = 2);
    void addManipulationContact(const std::string link, 
                    const Eigen::Vector3d pos_in_link,
                    const Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity(),
                    const int constraints_in_rotation = 2);

    /**
     * @brief      deletes the contact at a given link
     *
     * @param      link_name  the link at which we want to delete the contact
     */
    void deleteEnvironmentalContact(const std::string link_name);
    void deleteManipulationContact(const std::string link_name);



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
    void manipulationGraspMatrix(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     const Eigen::Vector3d center_point);
    void manipulationGraspMatrixInWorld(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     const Eigen::Vector3d center_point);
    void manipulationGraspMatrixLocalContactForces(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     const Eigen::Vector3d center_point);
    void manipulationGraspMatrixLocalContactForcesToWorld(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     const Eigen::Vector3d center_point);

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
    void manipulationGraspMatrixAtGeometricCenter(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center);
    void manipulationGraspMatrixAtGeometricCenterInWorld(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center);
    void manipulationGraspMatrixAtGeometricCenterLocalContactForces(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center);
    void manipulationGraspMatrixAtGeometricCenterLocalContactForcesToWorld(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center);

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
    void environmentalGraspMatrix(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     const Eigen::Vector3d center_point);
    void environmentalGraspMatrixInWorld(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     const Eigen::Vector3d center_point);
    void environmentalGraspMatrixLocalContactForces(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     const Eigen::Vector3d center_point);
    void environmentalGraspMatrixLocalContactForcesToWorld(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     const Eigen::Vector3d center_point);

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
    void environmentalGraspMatrixAtGeometricCenter(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center);
    void environmentalGraspMatrixAtGeometricCenterInWorld(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center);
    void environmentalGraspMatrixAtGeometricCenterLocalContactForces(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center);
    void environmentalGraspMatrixAtGeometricCenterLocalContactForcesToWorld(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center);


    /**
     * @brief displays the joints or links of the robot with the corresponding numbers
     */
    void displayJoints();
    void displayLinks();

    /// \brief internal rbdl model
    RigidBodyDynamics::Model* _rbdl_model;

    /// \brief Joint positions. Note: _q size can differ from dof() since spherical joints use quaternions.
    Eigen::VectorXd _q;

    /// \brief Joint velocities
    Eigen::VectorXd _dq;

    /// \brief Joint accelerations
    Eigen::VectorXd _ddq;

    /// \brief Mass Matrix
    Eigen::MatrixXd _M;

    /// \brief Inverse of the mass matrix
    Eigen::MatrixXd _M_inv;

    /// \brief gravity in base frame
    Eigen::Vector3d _world_gravity;

    /// \brief List of active contacts between robot and environment
    std::vector<ContactModel> _environmental_contacts;

    /// \brief List of active contacts between robot end effectors and manipulated objects
    std::vector<ContactModel> _manipulation_contacts;

public:
    ///
    /// @brief      compute the cross product operator of a 3d vector
    ///
    /// @param[in]  v     the vector
    ///
    /// @return     the associated 3d anti-symmetric matrix
    ///
    static Eigen::Matrix3d CrossProductOperator(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d v_hat;
        v_hat << 0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0;
        return v_hat;
    }

    /// \brief number of Dof of robot
    int _dof;

    /// \brief number of values required for robot joint positions description
    int _q_size;

    /// \brief Transform from world coordinates to robot base coordinates
    Eigen::Affine3d _T_world_robot;

// protected:
    /// \brief map from joint names to joint id
    std::map<std::string,int> _joint_names_map;

    /// \brief map from link names to link id
    std::map<std::string,int> _link_names_map;
};

 /**
  * @brief      Gives orientation error from rotation matrices
  *
  * @param      delta_phi            Vector on which the orientation error will
  *                                  be written
  * @param      desired_orientation  desired orientation rotation matrix
  * @param      current_orientation  current orientation matrix
  */
void orientationError(Eigen::Vector3d& delta_phi,
                      const Eigen::Matrix3d& desired_orientation,
                      const Eigen::Matrix3d& current_orientation);


/**
 * @brief      Gives orientation error from quaternions
 *
 * @param      delta_phi            Vector on which the orientation error will
 *                                  be written
 * @param      desired_orientation  desired orientation quaternion
 * @param      current_orientation  current orientation quaternion
 */
void orientationError(Eigen::Vector3d& delta_phi,
                      const Eigen::Quaterniond& desired_orientation,
                      const Eigen::Quaterniond& current_orientation);


/// \brief compute the cross product operator of a 3d vector
Eigen::Matrix3d CrossProductOperator(const Eigen::Vector3d& v);

/**
 * @brief      Computes the grasp matrix in the cases where there are 2, 3 or 4
 *             contacts in the _contact member vector. the external forces and
 *             moments are assumed to be in world frame for 2 contact points,
 *             the output resultant (first 6 lines) is given in world frame, and
 *             the output internal tension and moments are given in local frame,
 *             and the description of the local frame is given by R for 3 and 4
 *             contacts, the output quantities are given in world frame the
 *             convention for the output is the following order : support
 *             forces, support moments, internal tensions, internal moments the
 *             internal tensions are given in the order 1-2, 1-3, 2-3 in the 3
 *             contact case and 1-2, 1-3, 1-4, 2-3, 2-4, 3-4 in the 4 contact
 *             case.
 *
 * @param      G                      The grasp matrix that is going to be
 *                                    populated
 * @param      R                      the rotation matrix between the world
 *                                    frame and the frame attached to the object
 *                                    (useful when 2 contacts only)
 * @param      center_point           The position (in world frame) of the point
 *                                    on which we resolve the resultant forces
 *                                    and moments
 * @param[in]  contact_locations      The contact locations
 * @param[in]  constrained_rotations  The constrained rotations
 */
void graspMatrix(Eigen::MatrixXd& G,
                 Eigen::Matrix3d& R,
                 const Eigen::Vector3d center_point,
                 const std::vector<Eigen::Vector3d>& contact_locations,
                 const std::vector<int> constrained_rotations);

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
void graspMatrixAtGeometricCenter(Eigen::MatrixXd& G,
                 Eigen::Matrix3d& R,
                 Eigen::Vector3d& geometric_center,
                 const std::vector<Eigen::Vector3d>& contact_locations,
                 const std::vector<int> constrained_rotations);

} /* namespace Sai2Model */

#endif /* RBDLMODEL_H_ */

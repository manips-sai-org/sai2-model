/*
 * RBDLModel.h
 *
 *  Created on: Dec 14, 2016
 *      Author: Mikael Jorda
 */

#ifndef SAI2_RBDLMODEL_H_
#define SAI2_RBDLMODEL_H_

#include <rbdl/Model.h>

namespace Model
{

class RBDLModel
{
public:
    RBDLModel ();
    RBDLModel (const std::string path_to_model_file, bool verbose);
    ~RBDLModel ();


    /**
     * @brief update the dynamics. call with model joint positions, velocities and acceleration
     * @param q Joint positions
     * @param dq Joint velocities
     * @param ddq Joint accelerations
     */
    void updateModel(const Eigen::VectorXd& q,
                             const Eigen::VectorXd& dq,
                             const Eigen::VectorXd& ddq);

    /**
     * @brief Gives the mass matrix of the last updated configuration
     * @param A matrix to which the mass matrix will be written. Size must be _dof x _dof
     * @param q Joint positions
     */
    void massMatrix(Eigen::MatrixXd& A,
                            const Eigen::VectorXd& q);

    /**
     * @brief Gives the joint gravity torques vector of the last updated configuration
     * @param g Vector to which the joint gravity torques will be written
     * @param q Joint positions
     * @param gravity the 3d gravity vector of the world in base frame
     */
    void gravityVector(Eigen::VectorXd& g,
                               const Eigen::VectorXd& q,
                               const Eigen::Vector3d& gravity = Eigen::Vector3d(0,0,-9.8));

    /**
     * @brief Gives the joint coriolis and centrifugal forces of the last updated configuration
     * @param b Vector to which the joint coriolis and centrifugal forces will be written
     * @param q Joint positions
     * @param dq Joint velocities
     */
    void coriolisForce(Eigen::VectorXd& b,
                               const Eigen::VectorXd& q,
                               const Eigen::VectorXd& dq);

    /**
     * @brief Full jacobian for link, relative to base (id=0) in the form [Jv; Jw]
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     * @param q Joint positions
     */
    void J_0(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link,
                   const Eigen::VectorXd& q);

    /**
     * @brief Full jacobian for link, relative to base (id=0) in the form [Jv; Jw]
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     * @param q Joint positions
     */
    void J(Eigen::MatrixXd& J,
                   const std::string& link_name,
                   const Eigen::Vector3d& pos_in_link,
                   const Eigen::VectorXd& q);


    /**
     * @brief Velocity jacobian for point on link, relative to base (id=0)
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param pos_in_link the position of the point in the link where the jacobian is computed (in local link frame)
     * @param q Joint positions
     */
    void Jv(Eigen::MatrixXd& J,
                    const std::string& link_name,
                    const Eigen::Vector3d& pos_in_link,
                    const Eigen::VectorXd& q);


    /**
     * @brief Angular velocity jacobian for link, relative to base (id=0)
     * @param J Matrix to which the jacobian will be written
     * @param link_name the name of the link where to compute the jacobian
     * @param q Joint positions
     */
    void Jw(Eigen::MatrixXd& J,
                    const std::string& link_name,
                    const Eigen::VectorXd& q);



    /**
     * @brief transformation from base to link, in base coordinates.
     * @param T Transformation matrix to which the result is computed
     * @param link_name name of the link where to compute the transformation matrix
     * @param q Joint positions
     */
    void transform(Eigen::Affine3d& T,
                           const std::string& link_name,
                           const Eigen::VectorXd& q);

    /**
     * @brief Position from base to point in link, in base coordinates
     * @param pos Vector of position to which the result is written
     * @param link_name name of the link in which is the point where to compute the position
     * @param pos_in_link the position of the point in the link, in local link frame
     * @param q Joint positions
     */
    void position(Eigen::Vector3d& pos,
                          const std::string& link_name,
                          const Eigen::Vector3d& pos_in_link,
                          const Eigen::VectorXd& q);

    /**
     * @brief Velocity of point in link, in base coordinates
     * @param vel Vector of velocities to which the result is written
     * @param link_name name of the link in which is the point where to compute the velocity
     * @param pos_in_link the position of the point in the link, in local link frame
     * @param q Joint positions
     * @param dq Joint velocities
     */
    void linearVelocity(Eigen::Vector3d& vel,
                          const std::string& link_name,
                          const Eigen::Vector3d& pos_in_link,
                          const Eigen::VectorXd& q,
                          const Eigen::VectorXd& dq);

    /**
     * @brief Acceleration of point in link, in base coordinates
     * @param accel Vector of accelerations to which the result is written
     * @param link_name name of the link in which is the point where to compute the acceleration
     * @param pos_in_link the position of the point in the link, in local link frame
     * @param q Joint positions
     * @param dq Joint velocities
     * @param ddq Joint accelerations
     */
    void linearAcceleration(Eigen::Vector3d& accel,
                              const std::string& link_name,
                              const Eigen::Vector3d& pos_in_link,
                              const Eigen::VectorXd& q,
                              const Eigen::VectorXd& dq,
                              const Eigen::VectorXd& ddq);

    /**
     * @brief Rotation of a link with respect to base frame
     * @param rot Rotation matrix to which the result is written
     * @param link_name name of the link for which to compute the rotation
     * @param q Joint positions
     */
    void rotation(Eigen::Matrix3d& rot,
                          const std::string& link_name,
                          const Eigen::VectorXd& q);

    /**
     * @brief Angular velocity of a link with respect to base frame
     * @param avel Vector to which the result is written
     * @param link_name name of the link for which to compute the angular velocity
     * @param q Joint positions
     * @param dq Joint velocities
     */
    void angularVelocity(Eigen::Vector3d& avel,
                                 const std::string& link_name,
                                 const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& dq);

    /**
     * @brief Angular acceleration of a link with respect to base frame
     * @param aaccel Vector to which the result is written
     * @param link_name name of the link for which to compute the angular acceleration
     * @param q Joint positions
     * @param dq Joint velocities
     * @param ddq Joint accelerations
     */
    void angularAcceleration(Eigen::Vector3d& aaccel,
                                     const std::string& link_name,
                                     const Eigen::VectorXd& q,
                                     const Eigen::VectorXd& dq,
                                     const Eigen::VectorXd& ddq);

    /**
     * @brief Gives the link id for a given nave with the right indexing for rbdl
     * @param link_name name of the link
     */
    unsigned int linkId(const std::string& link_name);

    /* RBDL Model specific interface */

    /**
     * @brief Gives the mass properties of a given link
     * @param mass the returned mass value
     * @param center_of_mass the position of the center of mass in the body's frame
     * @param inertia the inertia of the given link
     * @param link_name the name of the considered link
     */
    void getLinkMass(double& mass,
                     Eigen::Vector3d& center_of_mass,
                     Eigen::Matrix3d& inertia,
                     const std::string& link_name);

    /**
     * @brief Gives the mass properties of a given link
     * @param mass the returned mass value
     * @param center_of_mass the position of the center of mass in the body's frame
     * @param link_name the name of the considered link
     */
    void getLinkMass(double& mass,
                     Eigen::Vector3d& center_of_mass,
                     const std::string& link_name);

    /// \brief internal rbdl model
    RigidBodyDynamics::Model _rbdl_model;

    ////// Robot State ///////
    /// \brief number of Dof of robot
    int _dof;

    /// \brief Joint positions
    Eigen::VectorXd _q;

    /// \brief Joint velocities
    Eigen::VectorXd _dq;

    /// \brief Joint accelerations
    Eigen::VectorXd _ddq;

    /// \brief Mass Matrix
    Eigen::MatrixXd _M;

    /// \brief Inverse of the mass matrix
    Eigen::MatrixXd _M_inv;

protected:
    /// \brief compute the cross product operator of a 3d vector
    static Eigen::Matrix3d CrossProductOperator(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d v_hat;
        v_hat << 0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0;
        return v_hat;
    }


};

} /* namespace Model */

#endif /* RBDLMODEL_H_ */

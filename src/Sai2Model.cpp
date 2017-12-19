/*
 * Sai2Model.cpp
 * 
 *  Wrapper around RBDL plus functions to facilitate the whole body control framework from Stanford robotics lab
 *
 *  Created on: Dec 14, 2016
 *      Author: Mikael Jorda
 */

#include "Sai2Model.h"

#include <rbdl/rbdl.h>
#include <UrdfToSai2Model.h>

#include <stdexcept>

namespace Sai2Model
{

Sai2Model::Sai2Model (const std::string path_to_model_file, bool verbose)
{

	// parse rbdl model from urdf
	bool success = RigidBodyDynamics::URDFReadFromFile(path_to_model_file.c_str(), _rbdl_model, false, verbose);
	if (!success) 
	{
		std::cout << "Error loading model [" + path_to_model_file + "]" << "\n";
	}

	// set the number of degrees of freedom
	_dof = _rbdl_model->dof_count;

	// TODO : support other initial joint configuration
    // resize state vectors
    _q.setZero(_dof);
    _dq.setZero(_dof);
    _ddq.setZero(_dof);
    _M.setIdentity(_dof,_dof);
    _M_inv.setIdentity(_dof,_dof);

	updateModel();
}


Sai2Model::~Sai2Model ()
{
	delete _rbdl_model;
	_rbdl_model = NULL;
}


void Sai2Model::updateKinematics()
{
	UpdateKinematicsCustom(*_rbdl_model, &_q, &_dq, &_ddq);
}


void Sai2Model::updateDynamics()
{
	if (_M.rows()!=_dof|| _M.cols()!=_dof)
	{_M.setZero(_dof,_dof);}

	CompositeRigidBodyAlgorithm(*_rbdl_model, _q, _M, false);
	_M_inv = _M.inverse();
}

void Sai2Model::updateModel()
{
	updateKinematics();
	updateDynamics();
}

int Sai2Model::dof()
{
	return _dof;
}

void Sai2Model::gravityVector(Eigen::VectorXd& g,
	const Eigen::Vector3d& gravity)
{

	if (g.size() != _dof){ g.resize(_dof); }
	g.setZero();

	std::vector<RigidBodyDynamics::Body>::iterator it_body;
	int body_id;

	for (it_body = _rbdl_model->mBodies.begin(), body_id=0;
	it_body != _rbdl_model->mBodies.end();
	++it_body, ++body_id)
	{
		double mass = it_body->mMass;
		Eigen::MatrixXd Jv = Eigen::MatrixXd::Zero(3, _dof);
		CalcPointJacobian(*_rbdl_model, _q, body_id, it_body->mCenterOfMass, Jv, false);

		g += Jv.transpose() * (-mass * gravity);
	}
}


void Sai2Model::coriolisForce(Eigen::VectorXd& b)
{
	NonlinearEffects(*_rbdl_model,_q,_dq,b);
}

void Sai2Model::J_0(Eigen::MatrixXd& J,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	if (J.rows()!=6 || J.cols()!=_dof) // resize to the right format
	{
		J.setZero(6,_dof);
	}
	Eigen::MatrixXd J_temp = Eigen::MatrixXd::Zero(6,_dof);
	CalcPointJacobian6D (*_rbdl_model, _q, linkId(link_name), pos_in_link, J_temp, false);

	// RBDL gives Jw as the top 3 rows and Jv as the bottom part. We need to swap it here
	J << J_temp.block(3,0,3,_dof),
		 J_temp.block(0,0,3,_dof);
}

void Sai2Model::J(Eigen::MatrixXd& J,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	if (J.rows()!=6 || J.cols()!=_dof) // resize to the right format
	{
		J.setZero(6,_dof);
	}
	CalcPointJacobian6D (*_rbdl_model, _q, linkId(link_name), pos_in_link, J, false);
}



void Sai2Model::Jv(Eigen::MatrixXd& J,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	if (J.rows()!=3 || J.cols()!=_dof)
	{
		J.setZero(3,_dof);
	}

	CalcPointJacobian(*_rbdl_model, _q, linkId(link_name), pos_in_link, J, false);
}



void Sai2Model::Jw(Eigen::MatrixXd& J,
 const std::string& link_name)
{
	// compute the full jacobian at the center of the link and take rotational part
	Eigen::MatrixXd J_temp = Eigen::MatrixXd::Zero(6,_dof);
	CalcPointJacobian6D (*_rbdl_model, _q, linkId(link_name), Eigen::Vector3d::Zero(), J_temp, false);
	J = J_temp.topRows<3>();
}



void Sai2Model::transform(Eigen::Affine3d& T,
 const std::string& link_name)
{
	unsigned int link_id = linkId(link_name);
	Eigen::Vector3d pos_in_body(0,0,0);
	T.linear() = CalcBodyWorldOrientation(*_rbdl_model, _q, link_id, false).transpose();
	T.translation() = CalcBodyToBaseCoordinates(*_rbdl_model, _q, link_id, pos_in_body, false);
}

void Sai2Model::transform(Eigen::Affine3d& T,
 const std::string& link_name,
 const Eigen::Vector3d& pos_in_body)
{
	unsigned int link_id = linkId(link_name);
	T.linear() = CalcBodyWorldOrientation(*_rbdl_model, _q, link_id, false).transpose();
	T.translation() = CalcBodyToBaseCoordinates(*_rbdl_model, _q, link_id, pos_in_body, false);
}

void Sai2Model::position(Eigen::Vector3d& pos,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	pos = CalcBodyToBaseCoordinates(*_rbdl_model, _q, linkId(link_name), pos_in_link, false);
}


void Sai2Model::linearVelocity(Eigen::Vector3d& vel,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	vel = CalcPointVelocity(*_rbdl_model,_q,_dq,linkId(link_name),pos_in_link,false);
}


void Sai2Model::linearAcceleration(Eigen::Vector3d& accel,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	accel = CalcPointAcceleration(*_rbdl_model,_q,_dq,_ddq,linkId(link_name),pos_in_link,false);
}


void Sai2Model::rotation(Eigen::Matrix3d& rot,
	const std::string& link_name)
{
	rot = CalcBodyWorldOrientation(*_rbdl_model, _q, linkId(link_name), false).transpose();
}


void Sai2Model::angularVelocity(Eigen::Vector3d& avel,
 const std::string& link_name)
{
	Eigen::VectorXd v_tmp = Eigen::VectorXd::Zero(6);
	v_tmp = CalcPointVelocity6D(*_rbdl_model,_q,_dq,linkId(link_name),Eigen::Vector3d::Zero(),false);
	avel = v_tmp.head(3);
}


void Sai2Model::angularAcceleration(Eigen::Vector3d& aaccel,
 const std::string& link_name)
{
	Eigen::VectorXd a_tmp = Eigen::VectorXd::Zero(6);
	a_tmp = CalcPointAcceleration6D(*_rbdl_model,_q,_dq,_ddq,linkId(link_name),Eigen::Vector3d::Zero(),false);
	aaccel = a_tmp.head(3);
}


unsigned int Sai2Model::linkId(const std::string& link_name)
{
	auto iter = _rbdl_model->mBodyNameMap.find(link_name);
	unsigned int body_id = iter->second;

	if (iter == _rbdl_model->mBodyNameMap.end()) {
	std::cout << "link ["+link_name+"] does not exists\n";
	}

	return body_id;
}


void Sai2Model::getLinkMass(double& mass,
 Eigen::Vector3d& center_of_mass,
 Eigen::Matrix3d& inertia,
 const std::string& link_name)
{
	RigidBodyDynamics::Body b = _rbdl_model->mBodies[linkId(link_name)];

	mass = b.mMass;
	center_of_mass = b.mCenterOfMass;
	inertia = b.mInertia;
}

void Sai2Model::getLinkMass(double& mass,
 Eigen::Vector3d& center_of_mass,
 const std::string& link_name)
{
	RigidBodyDynamics::Body b = _rbdl_model->mBodies[linkId(link_name)];

	mass = b.mMass;
	center_of_mass = b.mCenterOfMass;
}

// TODO : Untested
void Sai2Model::orientationError(Eigen::Vector3d& delta_phi,
		              const Eigen::Matrix3d& desired_orientation,
		              const Eigen::Matrix3d& current_orientation)
{
	// check that the matrices are valid rotations
	Eigen::Matrix3d Q1 = desired_orientation*desired_orientation.transpose() - Eigen::Matrix3d::Identity();
	Eigen::Matrix3d Q2 = current_orientation*current_orientation.transpose() - Eigen::Matrix3d::Identity();
	if(Q1.norm() > 0.0001 || Q2.norm() > 0.0001)
	{
		throw std::invalid_argument("Invalid rotation matrices in Sai2Model::orientationError");
		return;
	}
	else
	{
		Eigen::Vector3d rc1 = current_orientation.block<3,1>(0,0);
		Eigen::Vector3d rc2 = current_orientation.block<3,1>(0,1);
		Eigen::Vector3d rc3 = current_orientation.block<3,1>(0,2);
		Eigen::Vector3d rd1 = desired_orientation.block<3,1>(0,0);
		Eigen::Vector3d rd2 = desired_orientation.block<3,1>(0,1);
		Eigen::Vector3d rd3 = desired_orientation.block<3,1>(0,2);
		delta_phi = -1.0/2.0*(rc1.cross(rd1) + rc2.cross(rd2) + rc3.cross(rd3));
	}
}

void Sai2Model::orientationError(Eigen::Vector3d& delta_phi,
		              const Eigen::Quaterniond& desired_orientation,
		              const Eigen::Quaterniond& current_orientation)
{
	Eigen::Quaterniond inv_dlambda = desired_orientation*current_orientation.conjugate();
	delta_phi = 2.0*inv_dlambda.vec();
}

// TODO : Untested
void Sai2Model::taskInertiaMatrix(Eigen::MatrixXd& Lambda,
    					   const Eigen::MatrixXd& task_jacobian)
{
	// check matrices have the right size
	if(Lambda.rows() != Lambda.cols())
	{
		throw std::invalid_argument("Lambda matrix not square in Sai2Model::taksInertiaMatrix");
		return;
	}
	else if (Lambda.rows() != task_jacobian.rows())
	{
		throw std::invalid_argument("Rows of Jacobian inconsistent with size of Lambda matrix in Sai2Model::taksInertiaMatrix");
		return;
	}
	else if(task_jacobian.cols() != _dof)
	{
		throw std::invalid_argument("Jacobian size inconsistent with DOF of robot model in Sai2Model::taksInertiaMatrix");
		return;
	}
	// compute task inertia
	else
	{
		Eigen::MatrixXd inv_inertia = task_jacobian*_M_inv*task_jacobian.transpose();
		Lambda = inv_inertia.inverse();
	}
}

// TODO : Untested
void Sai2Model::taskInertiaMatrixWithPseudoInv(Eigen::MatrixXd& Lambda,
    					   const Eigen::MatrixXd& task_jacobian)
{
	// check matrices have the right size
	if (Lambda.rows() != Lambda.cols())
	{
		throw std::invalid_argument("Lambda matrix not square in Sai2Model::taskInertiaMatrixWithPseudoInv");
		return;
	}
	else if (Lambda.rows() != task_jacobian.rows())
	{
		throw std::invalid_argument("Rows of Jacobian inconsistent with size of Lambda matrix in Sai2Model::taskInertiaMatrixWithPseudoInv");
		return;
	}
	else if (task_jacobian.cols() != _dof)
	{
		throw std::invalid_argument("Jacobian size inconsistent with DOF of robot model in Sai2Model::taskInertiaMatrixWithPseudoInv");
		return;
	}

	// compute task inertia
	Eigen::MatrixXd inv_inertia = task_jacobian*_M_inv*task_jacobian.transpose();

	// compute SVD pseudoinverse
	// TODO: make class function?
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(inv_inertia, Eigen::ComputeThinU | Eigen::ComputeThinV);
	const double epsilon = std::numeric_limits<double>::epsilon();
	double tolerance = epsilon * std::max(inv_inertia.cols(), inv_inertia.rows()) * svd.singularValues().array().abs()(0);
	Lambda = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

//TODO : Untested
void Sai2Model::dynConsistentInverseJacobian(Eigen::MatrixXd& Jbar,
									const Eigen::MatrixXd& task_jacobian)
{
	// check matrices have the right size
	if(Jbar.rows() != task_jacobian.cols() || Jbar.cols() != task_jacobian.rows())
	{
		throw std::invalid_argument("Matrix dimmensions inconsistent in Sai2Model::dynConsistentInverseJacobian");
		return;
	}
	// compute Jbar
	else
	{
		Eigen::MatrixXd task_inertia(task_jacobian.rows(),task_jacobian.rows());
		taskInertiaMatrix(task_inertia,task_jacobian);
		Jbar = _M_inv*task_jacobian.transpose()*task_inertia;
	}
}

void Sai2Model::nullspaceMatrix(Eigen::MatrixXd& N,
        					 const Eigen::MatrixXd& task_jacobian)
{
	Eigen::MatrixXd N_prec = Eigen::MatrixXd::Identity(dof(),dof());
	nullspaceMatrix(N,task_jacobian,N_prec);
}

//TODO :: Untested
void Sai2Model::nullspaceMatrix(Eigen::MatrixXd& N,
    					 const Eigen::MatrixXd& task_jacobian,
    					 const Eigen::MatrixXd& N_prec)
{
	// check matrices dimmnsions
	if(N.rows() != N.cols() || N.rows() != _dof)
	{
		throw std::invalid_argument("N matrix dimensions inconsistent in Sai2Model::nullspaceMatrix");
		return;
	}
	else if(N_prec.rows() != N_prec.cols() || N_prec.rows() != _dof)
	{
		throw std::invalid_argument("N_prec matrix dimensions inconsistent in Sai2Model::nullspaceMatrix");
		return;
	}
	else if(task_jacobian.cols() != N.rows())
	{
		throw std::invalid_argument("jacobian matrix dimensions inconsistent with model dof in Sai2Model::nullspaceMatrix");
		return;
	}
	// Compute N
	else
	{
		Eigen::MatrixXd Jbar = Eigen::MatrixXd::Zero(task_jacobian.cols(),task_jacobian.rows());
		dynConsistentInverseJacobian(Jbar,task_jacobian);
		Eigen::MatrixXd Ni = Eigen::MatrixXd::Identity(N.rows(),N.cols());
		Ni = Ni - Jbar*task_jacobian;
		N = Ni*N_prec;
	}
}

// TODO : Untested
void Sai2Model::operationalSpaceMatrices(Eigen::MatrixXd& Lambda, Eigen::MatrixXd& Jbar, Eigen::MatrixXd& N,
                                    const Eigen::MatrixXd& task_jacobian)
{
	Eigen::MatrixXd N_prec = Eigen::MatrixXd::Identity(dof(),dof());
	operationalSpaceMatrices(Lambda,Jbar,N,task_jacobian,N_prec);
}

// TODO : Untested
void Sai2Model::operationalSpaceMatrices(Eigen::MatrixXd& Lambda, Eigen::MatrixXd& Jbar, Eigen::MatrixXd& N,
                                    const Eigen::MatrixXd& task_jacobian,
                                    const Eigen::MatrixXd& N_prec)
{
	// check matrices have the right size
	if(Lambda.rows() != Lambda.cols())
	{
		throw std::invalid_argument("Lambda matrix not square in Sai2Model::operationalSpaceMatrices");
		return;
	}
	else if (Lambda.rows() != task_jacobian.rows())
	{
		throw std::invalid_argument("Rows of Jacobian inconsistent with size of Lambda matrix in Sai2Model::operationalSpaceMatrices");
		return;
	}
	else if(task_jacobian.cols() != _dof)
	{
		throw std::invalid_argument("Jacobian size inconsistent with DOF of robot model in Sai2Model::operationalSpaceMatrices");
		return;
	}
	else if(Jbar.rows() != task_jacobian.cols() || Jbar.cols() != task_jacobian.rows())
	{
		throw std::invalid_argument("Matrix dimmensions inconsistent in Sai2Model::operationalSpaceMatrices");
		return;
	}
	else if(N.rows() != N.cols() || N.rows() != _dof)
	{
		throw std::invalid_argument("N matrix dimensions inconsistent in Sai2Model::operationalSpaceMatrices");
		return;
	}
	else if(N_prec.rows() != N_prec.cols() || N_prec.rows() != _dof)
	{
		throw std::invalid_argument("N_prec matrix dimensions inconsistent in Sai2Model::operationalSpaceMatrices");
		return;
	}
	else if(task_jacobian.cols() != N.rows())
	{
		throw std::invalid_argument("jacobian matrix dimensions inconsistent with model dof in Sai2Model::operationalSpaceMatrices");
		return;
	}
	// Compute the matrices
	else
	{
		Eigen::MatrixXd inv_inertia = task_jacobian*_M_inv*task_jacobian.transpose();
		Lambda = inv_inertia.inverse();
		Jbar = _M_inv*task_jacobian.transpose()*Lambda;
		Eigen::MatrixXd Ni = Eigen::MatrixXd::Identity(N.rows(),N.cols());
		Ni = Ni - Jbar*task_jacobian;
		N = Ni*N_prec;
	}
}

void Sai2Model::GraspMatrix(Eigen::MatrixXd& G,
	Eigen::Matrix3d& R,
	const std::vector<std::string> link_names,
	const std::vector<Eigen::Vector3d> pos_in_links,
	const std::vector<ContactNature> contact_natures,
	const Eigen::Vector3d center_point)
{
	G = Eigen::MatrixXd::Zero(1,1);
	R = Eigen::Matrix3d::Identity();

	// number of contact points
	int n = link_names.size();
	if(n < 2)
	{
		throw std::invalid_argument("invalid number of contact points (2 points min)\n");
	}
	if(n > 4)
	{
		throw std::invalid_argument("invalid number of contact points (4 points max)\n");
	}
	if((pos_in_links.size() != n) || (contact_natures.size() != n))
	{
		throw std::invalid_argument("input vectors for the link names, pos in links and contact natures don't have the same size\n");
	}
	// number of surface contacts (that can apply a moment)
	int k = std::count(contact_natures.begin(), contact_natures.end(), SurfaceContact);

	Eigen::MatrixXd Wf = Eigen::MatrixXd::Zero(6, 3*n);
	Eigen::MatrixXd Wm = Eigen::MatrixXd::Zero(6, 3*k);

	std::vector<Eigen::Vector3d> positions_in_world;

	for(int i=0; i<n; i++)
	{
		Eigen::Vector3d pi;
		position(pi, link_names[i], pos_in_links[i]);
		positions_in_world.push_back(pi);
		Eigen::Vector3d ri = pi-center_point;
		Wf.block<3,3>(0,3*i) = Eigen::Matrix3d::Identity();
		Wf.block<3,3>(3,3*i) = CrossProductOperator(ri);
	}
	for(int i=0; i<k; i++)
	{
		Wm.block<3,3>(3,3*i) = Eigen::Matrix3d::Identity();
	}

	Eigen::MatrixXd E, I;

	switch (n)
	{
		case 2: 
		{
			// resize E
			E = Eigen::MatrixXd::Zero(6,1);
			
			// compute the point to point vectors
			Eigen::Vector3d e12 = positions_in_world[1] - positions_in_world[0];
			e12.normalize();

			// fill in E matrix
			E.block<3,1>(0,0) = -e12;
			E.block<3,1>(3,0) = e12;

			// create Ebar
			Eigen::MatrixXd Ebar = (E.transpose()*E).inverse() * E.transpose();

			// find R
			Eigen::Vector3d x = e12;
			// std::cout << "new x : " << x.transpose() << std::endl;
			// std::cout << "new x cross world x : " << (x.cross(Eigen::Vector3d(1,0,0))).transpose() << std::endl;
			// std::cout << "new x cross world x norm : " << (x.cross(Eigen::Vector3d(1,0,0))).norm() << std::endl;
			// std::cout << "abs : " << std::abs((x.cross(Eigen::Vector3d(1,0,0))).norm()) << std::endl;
			// std::cout << std::endl;
			if(std::abs((x.cross(Eigen::Vector3d(1,0,0))).norm()) < 1e-3) // new x is aligned with world x
			{
				if(x.dot(Eigen::Vector3d(1,0,0)) > 0) // same direction
				{
					R = Eigen::Matrix3d::Identity();
					// std::cout << "R is identity" << std::endl;
				}
				else // rotation around Z axis by 180 degrees
				{
					R << -1, 0, 0,
						 0, -1, 0, 
						 0, 0, 1;
				}
			}
			else
			{
				Eigen::Vector3d y = x.cross(Eigen::Vector3d(1,0,0));
				y.normalize();
				Eigen::Vector3d z = x.cross(y);
				z.normalize();
				R.block<3,1>(0,0) = x;
				R.block<3,1>(0,1) = y;
				R.block<3,1>(0,2) = z;
			}

			Eigen::MatrixXd Rr = Eigen::MatrixXd::Zero(6,6);
			Rr.block<3,3>(0,0) = R;
			Rr.block<3,3>(3,3) = R;

			Wf = Rr.transpose() * Wf;

			switch(k)
			{
				case 0:
				{
					throw std::runtime_error("Case 2-0 not implemented yet\n");
					break;
				}
				case 1: 
				{
					// only 2 internal moments
					I = Eigen::MatrixXd::Zero(2,3);

					I << 0, 1, 0,
					     0, 0, 1;
					I = I*R.transpose();

					Wm = Rr.transpose()*Wm;

					// populate G
					G = Eigen::MatrixXd::Zero(9,9);
					G.block<6,6>(0,0) = Wf;
					G.block<6,3>(0,6) = Wm;
					G.block<1,6>(6,0) = Ebar;
					G.block<2,3>(7,6) = I;
					break;
				}
				case 2: 
				{
					I = Eigen::MatrixXd::Zero(5,6);

					// find I
					I << -0.5, 0, 0, 0.5, 0, 0,
						  0, 1, 0, 0, 0, 0,
						  0, 0, 1, 0, 0, 0,
						  0, 0, 0, 0, 1, 0,
						  0, 0, 0, 0, 0, 1;
					I = I*Rr.transpose();

					Wm = Rr.transpose()*Wm;

					// populate G
					G = Eigen::MatrixXd::Zero(12,12);
					G.block<6,6>(0,0) = Wf;
					G.block<6,6>(0,6) = Wm;
					G.block<1,6>(6,0) = Ebar;
					G.block<5,6>(7,6) = I;
					break;
				}
				default: 
				throw std::runtime_error("Should not arrive here (number of contact points is 2, number of surface contacts incoherent)\n");

			}
			break;

		}

		case 3: 
		{
			// resize E
			E = Eigen::MatrixXd::Zero(9,3);
			
			// compute the point to point vectors
			Eigen::Vector3d e12 = positions_in_world[1] - positions_in_world[0];
			Eigen::Vector3d e13 = positions_in_world[2] - positions_in_world[0];
			Eigen::Vector3d e23 = positions_in_world[2] - positions_in_world[1];

			e12.normalize();
			e13.normalize();
			e23.normalize();

			// fill in E matrix
			E.block<3,1>(0,0) = -e12;
			E.block<3,1>(3,0) = e12;
			E.block<3,1>(0,1) = -e13;
			E.block<3,1>(6,1) = e13;
			E.block<3,1>(3,2) = -e23;
			E.block<3,1>(6,2) = e23;

			// std::cout << "E : \n" << E << std::endl << std::endl;

			// create Ebar
			Eigen::MatrixXd Ebar = (E.transpose()*E).inverse() * E.transpose();

			switch(k)
			{
				case 0:
				{
					// populate G
					G = Eigen::MatrixXd::Zero(9,9);
					G.block<6,9>(0,0) = Wf;
					G.block<3,9>(6,0) = Ebar;
					break;
				}
				case 1: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);

					// populate G
					G = Eigen::MatrixXd::Zero(12,12);
					G.block<6,9>(0,0) = Wf;
					G.block<6,3>(0,9) = Wm;
					G.block<3,9>(6,0) = Ebar;
					G.block<3,3>(9,9) = I;
					break;
				}
				case 2: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);

					// populate G
					G = Eigen::MatrixXd::Zero(15,15);
					G.block<6,9>(0,0) = Wf;
					G.block<6,6>(0,9) = Wm;
					G.block<3,9>(6,0) = Ebar;
					G.block<6,6>(9,9) = I;
					break;
				}
				case 3: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9,9);

					// populate G
					G = Eigen::MatrixXd::Zero(18,18);
					G.block<6,9>(0,0) = Wf;
					G.block<6,9>(0,9) = Wm;
					G.block<3,9>(6,0) = Ebar;
					G.block<9,9>(9,9) = I;
					break;
				}

				default: 
				throw std::runtime_error("Should not arrive here (number of contact points is 3, number of surface contacts incoherent)\n");

			}
			break;

		}		

		case 4: 
		{
			// resize E
			E = Eigen::MatrixXd::Zero(12,6);
			
			// compute the point to point vectors
			Eigen::Vector3d e12 = positions_in_world[1] - positions_in_world[0];
			Eigen::Vector3d e13 = positions_in_world[2] - positions_in_world[0];
			Eigen::Vector3d e14 = positions_in_world[3] - positions_in_world[0];
			Eigen::Vector3d e23 = positions_in_world[2] - positions_in_world[1];
			Eigen::Vector3d e24 = positions_in_world[3] - positions_in_world[1];
			Eigen::Vector3d e34 = positions_in_world[3] - positions_in_world[2];

			e12.normalize();
			e13.normalize();
			e14.normalize();
			e23.normalize();
			e24.normalize();
			e34.normalize();

			// fill in E matrix
			E.block<3,1>(0,0) = -e12;
			E.block<3,1>(3,0) = e12;
			E.block<3,1>(0,1) = -e13;
			E.block<3,1>(6,1) = e13;
			E.block<3,1>(0,2) = -e14;
			E.block<3,1>(9,2) = e14;
			E.block<3,1>(3,3) = -e23;
			E.block<3,1>(6,3) = e23;
			E.block<3,1>(3,4) = -e24;
			E.block<3,1>(9,4) = e24;
			E.block<3,1>(6,5) = -e34;
			E.block<3,1>(9,5) = e34;


			// create Ebar
			Eigen::MatrixXd Ebar = (E.transpose()*E).inverse() * E.transpose();

			switch(k)
			{
				case 0:
				{
					// populate G
					G = Eigen::MatrixXd::Zero(12,12);
					G.block<6,12>(0,0) = Wf;
					G.block<6,12>(6,0) = Ebar;
					break;
				}
				case 1: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);

					// populate G
					G = Eigen::MatrixXd::Zero(15,15);
					G.block<6,12>(0,0) = Wf;
					G.block<6,3>(0,12) = Wm;
					G.block<6,12>(6,0) = Ebar;
					G.block<3,3>(12,12) = I;
					break;
				}
				case 2: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6,6);

					// populate G
					G = Eigen::MatrixXd::Zero(18,18);
					G.block<6,12>(0,0) = Wf;
					G.block<6,6>(0,12) = Wm;
					G.block<6,12>(6,0) = Ebar;
					G.block<6,6>(12,12) = I;
					break;
				}
				case 3: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9,9);

					// populate G
					G = Eigen::MatrixXd::Zero(21,21);
					G.block<6,12>(0,0) = Wf;
					G.block<6,9>(0,12) = Wm;
					G.block<6,12>(6,0) = Ebar;
					G.block<9,9>(12,12) = I;
					break;
				}
				case 4: 
				{
					// compute I
					Eigen::MatrixXd I = Eigen::MatrixXd::Identity(12,12);

					// populate G
					G = Eigen::MatrixXd::Zero(24,24);
					G.block<6,12>(0,0) = Wf;
					G.block<6,12>(0,12) = Wm;
					G.block<6,12>(6,0) = Ebar;
					G.block<12,12>(12,12) = I;
					break;
				}

				default: 
				throw std::runtime_error("Should not arrive here (number of contact points is 4, number of surface contacts incoherent)\n");

			}
			break;

		}

		default:
		throw std::runtime_error("Should not arrive here (number of contact points is not 2, 3 or 4) \n");

	}

}

void Sai2Model::GraspMatrixAtGeometricCenter(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center,
                     const std::vector<std::string> link_names,
                     const std::vector<Eigen::Vector3d> pos_in_links,
                     const std::vector<ContactNature> contact_natures)
{
	// number of contact points
	int n = link_names.size();
	if(n < 2)
	{
		throw std::invalid_argument("invalid number of contact points (2 points min)\n");
	}
	if(n > 4)
	{
		throw std::invalid_argument("invalid number of contact points (4 points max)\n");
	}
	if((pos_in_links.size() != n) || (contact_natures.size() != n))
	{
		throw std::invalid_argument("input vectors for the link names, pos in links and contact natures don't have the same size\n");
	}

	geometric_center.setZero();

	for(int i=0; i<n; i++)
	{
		Eigen::Vector3d pi;
		position(pi, link_names[i], pos_in_links[i]);
		geometric_center += pi;
	}
	geometric_center = geometric_center/(double)n;

	GraspMatrix(G, R, link_names, pos_in_links, contact_natures, geometric_center);
}


} /* namespace Model */

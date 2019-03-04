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

Sai2Model::Sai2Model (const std::string path_to_model_file, 
	                  bool verbose, 
	                  const Eigen::Affine3d T_world_robot,
	                  const Eigen::Vector3d world_gravity)
{

	_rbdl_model = new RigidBodyDynamics::Model();

	// parse rbdl model from urdf
	bool success = RigidBodyDynamics::URDFReadFromFile(path_to_model_file.c_str(), _rbdl_model, _link_names_map, _joint_names_map, false, verbose, world_gravity);
	if (!success) 
	{
		std::cout << "Error loading model [" + path_to_model_file + "]" << "\n";
	}

	// set the base position in the world
	_T_world_robot = T_world_robot;

	// set the number of degrees of freedom
	_dof = _rbdl_model->dof_count;

	// TODO : support other initial joint configuration
    // resize state vectors
    _q.setZero(_dof);
    _dq.setZero(_dof);
    _ddq.setZero(_dof);
    _M.setIdentity(_dof,_dof);
    _M_inv.setIdentity(_dof,_dof);

    _world_gravity = world_gravity;

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

void Sai2Model::gravityVector(Eigen::VectorXd& g)
{
	Eigen::Vector3d gravity = _rbdl_model->gravity;

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
	// actually returns v + g. Need to substract the gravity from it
	NonlinearEffects(*_rbdl_model,_q,_dq,b);

	Eigen::VectorXd g = Eigen::VectorXd::Zero(_dof);
	gravityVector(g);

	b -= g;
}

void Sai2Model::modifiedNewtonEuler(Eigen::VectorXd& u, 
								const bool consider_gravity,
                                const Eigen::VectorXd& q,
                                const Eigen::VectorXd& dq,
                                const Eigen::VectorXd& dqa,
                            	const Eigen::VectorXd& ddq)
{
	u = Eigen::VectorXd::Zero(_dof);

	// std::cout << u.transpose() << std::endl;

	std::vector<Eigen::Vector3d> w, wa, dw, ddO, ddB, f, tau;
	std::vector<Eigen::Vector3d> ripi_list, rib_list, z_list;
	Eigen::Vector3d w_i, wa_i, dw_i, ddO_i, ddB_i, f_i, tau_i;
	Eigen::Vector3d wp, wap, dwp, ddOp, ddBp, fc, tauc;
	Eigen::Vector3d z, r_ipi, r_ipb, r_ib;

	// initial conditions forward recursion
	w_i.setZero();
	wa_i.setZero();
	dw_i.setZero();
	// ddO_i.setZero();
	if(consider_gravity)
	{
		ddO_i = -_rbdl_model->gravity;
	}
	else
	{
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

	for(int i=1; i < _dof+1; i++)
	{
		int parent = _rbdl_model->lambda_q[i];
		std::vector<unsigned int> children = _rbdl_model->mu[i];
		int child;
		if(children.empty())
		{
			child = i;
			r_ipi.setZero();
		}
		else if(children.size() == 1)
		{
			child = _rbdl_model->mu[i][0];	
			r_ipi = _rbdl_model->X_lambda[child].r;
		} 
		else
		{
			throw("tree structures not implemented yet");
		}
		z = _rbdl_model->mJoints[i].mJointAxes->head(3);
		// std::cout << "parent joint axis : " << _rbdl_model->mJoints[i].mJointAxes->transpose() << std::endl;
		// r_ipi = _rbdl_model->X_T[i].r;
		r_ipb = _rbdl_model->mBodies[i].mCenterOfMass;
		r_ib = -r_ipi + r_ipb;

		// transform parent quantities in local frame
		wp = _rbdl_model->X_lambda[i].E*w[parent];
		wap = _rbdl_model->X_lambda[i].E*wa[parent];
		dwp = _rbdl_model->X_lambda[i].E*dw[parent];
		ddOp = _rbdl_model->X_lambda[i].E*ddO[parent];
		// ddBp = _rbdl_model->X_lambda[i].E*w[parent];
		// std::cout << "wp : " << wp.transpose() << std::endl << std::endl;

		// std::cout << "index : " << i << std::endl;
		// std::cout << "parent : " << parent << std::endl;
		// std::cout << "child : " << child << std::endl;
		// std::cout << "z : " << z.transpose() << std::endl;
		// std::cout << "r_ipi : " << r_ipi.transpose() << std::endl;
		// std::cout << "r_ipb : " << r_ipb.transpose() << std::endl;
		// std::cout << "r_ib : " << r_ib.transpose() << std::endl;
		// std::cout << "q : " << q.transpose() << std::endl;
		// std::cout << "dq : " << dq.transpose() << std::endl;

		w_i = wp + dq(parent)*z;
		wa_i = wap + dqa(parent)*z;
		dw_i = dwp + ddq(parent)*z + dq(parent)*wap.cross(z);
		ddO_i = ddOp + dw_i.cross(r_ipi) + w_i.cross(wa_i.cross(r_ipi));
		ddB_i = ddO_i + dw_i.cross(r_ib) + w_i.cross(wa_i.cross(r_ib));

		// std::cout << "w_i : " << w_i.transpose() << std::endl;
		// std::cout << "dw_i : " << dw_i.transpose() << std::endl;
		// std::cout << "ddO_i : " << ddO_i.transpose() << std::endl;
		// std::cout << "ddB_i : " << ddB_i.transpose() << std::endl;
		// std::cout << std::endl << std::endl;

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

	// std::cout << "end of forward recursion" << std::endl << std::endl;
	// std::cout << w << std::endl;

	// backward recursion
	for(int i=_dof; i>0; i--)
	{
		Eigen::Vector3d fip, tauip;
		std::vector<unsigned int> children = _rbdl_model->mu[i];
		if(children.size() == 0)
		{
			fip.setZero();
			tauip.setZero();
		}
		else if(children.size() == 1)
		{
			int child = children[0];
			fip = _rbdl_model->X_lambda[child].E.transpose()*f[child];
			tauip = _rbdl_model->X_lambda[child].E.transpose()*tau[child];
		}
		else
		{
			throw("tree structures not implemented yet");
		}

		double m = _rbdl_model->mBodies[i].mMass;
		Eigen::Matrix3d I = _rbdl_model->mBodies[i].mInertia;

		// std::cout << "index : " << i << std::endl;
		// std::cout << "mass : " << m << std::endl;
		// std::cout << "inertia :\n" << I << std::endl;

		f_i = fip + m*ddB[i];
		tau_i = tauip - f_i.cross(ripi_list[i]+rib_list[i]) + fip.cross(rib_list[i]) + I*dw[i] + wa[i].cross(I*w[i]);
		
		// std::cout << "fip : " << fip.transpose() << std::endl;
		// std::cout << "tauip : " << tauip.transpose() << std::endl;
		// std::cout << "f_i : " << f_i.transpose() << std::endl;
		// std::cout << "tau_i : " << tau_i.transpose() << std::endl;

		// int parent = _rbdl_model->lambda_q[i];
		Eigen::Vector3d zp = z_list[i];
		u(i-1) = tau_i.dot(zp);

		// std::cout << "zp : " << zp.transpose() << std::endl;
		// std::cout << "u_i : " << u(i-1) << std::endl;
		// std::cout << std::endl << std::endl;

		f[i] = f_i;
		tau[i] = tau_i;

	}


}

void Sai2Model::factorizedChristoffelMatrix(Eigen::MatrixXd& C)
{
	// check matrix have the right size
	if(C.rows() != C.cols())
	{
		throw std::invalid_argument("C matrix not square in Sai2Model::factorizedChristoffelMatrix");
		return;
	}
	else if(C.cols() != _dof)
	{
		throw std::invalid_argument("C matrix size inconsistent with DOF of robot model in Sai2Model::factorizedChristoffelMatrix");
		return;
	}
	// compute task inertia
	else
	{
		Eigen::VectorXd vi = Eigen::VectorXd::Zero(_dof);
		Eigen::VectorXd u = Eigen::VectorXd::Zero(_dof);

		for(int i=0; i<_dof; i++)
		{
			vi.setZero();
			u.setZero();
			vi(i) = 1;
			modifiedNewtonEuler(u, false, _q, _dq, vi, Eigen::VectorXd::Zero(_dof));
			C.col(i) = u;
		}
	}
}

void Sai2Model::J_0(Eigen::MatrixXd& J,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	J.setZero(6,_dof);
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
	J.setZero(6,_dof);
	CalcPointJacobian6D (*_rbdl_model, _q, linkId(link_name), pos_in_link, J, false);
}



void Sai2Model::Jv(Eigen::MatrixXd& J,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	J.setZero(3,_dof);
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

void Sai2Model::transformInWorld(Eigen::Affine3d& T,
 const std::string& link_name)
{
	unsigned int link_id = linkId(link_name);
	Eigen::Vector3d pos_in_body(0,0,0);
	T.linear() = CalcBodyWorldOrientation(*_rbdl_model, _q, link_id, false).transpose();
	T.translation() = CalcBodyToBaseCoordinates(*_rbdl_model, _q, link_id, pos_in_body, false);
	T = _T_world_robot*T;
}

void Sai2Model::transformInWorld(Eigen::Affine3d& T,
 const std::string& link_name,
 const Eigen::Vector3d& pos_in_body)
{
	unsigned int link_id = linkId(link_name);
	T.linear() = CalcBodyWorldOrientation(*_rbdl_model, _q, link_id, false).transpose();
	T.translation() = CalcBodyToBaseCoordinates(*_rbdl_model, _q, link_id, pos_in_body, false);
	T = _T_world_robot*T;
}

void Sai2Model::position(Eigen::Vector3d& pos,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	pos = CalcBodyToBaseCoordinates(*_rbdl_model, _q, linkId(link_name), pos_in_link, false);
}

void Sai2Model::positionInWorld(Eigen::Vector3d& pos,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	pos = CalcBodyToBaseCoordinates(*_rbdl_model, _q, linkId(link_name), pos_in_link, false);
	pos = _T_world_robot*pos;

}

void Sai2Model::linearVelocity(Eigen::Vector3d& vel,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	vel = CalcPointVelocity(*_rbdl_model,_q,_dq,linkId(link_name),pos_in_link,false);
}

void Sai2Model::linearVelocityInWorld(Eigen::Vector3d& vel,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	vel = CalcPointVelocity(*_rbdl_model,_q,_dq,linkId(link_name),pos_in_link,false);
	vel = _T_world_robot.linear()*vel;
}

void Sai2Model::linearAcceleration(Eigen::Vector3d& accel,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	accel = CalcPointAcceleration(*_rbdl_model,_q,_dq,_ddq,linkId(link_name),pos_in_link,false);
}

void Sai2Model::linearAccelerationInWorld(Eigen::Vector3d& accel,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link)
{
	accel = CalcPointAcceleration(*_rbdl_model,_q,_dq,_ddq,linkId(link_name),pos_in_link,false);
	accel = _T_world_robot.linear()*accel;
}

void Sai2Model::rotation(Eigen::Matrix3d& rot,
	const std::string& link_name)
{
	rot = CalcBodyWorldOrientation(*_rbdl_model, _q, linkId(link_name), false).transpose();
}

void Sai2Model::rotationInWorld(Eigen::Matrix3d& rot,
	const std::string& link_name)
{
	rot = CalcBodyWorldOrientation(*_rbdl_model, _q, linkId(link_name), false).transpose();
	rot = _T_world_robot.linear()*rot;
}

void Sai2Model::angularVelocity(Eigen::Vector3d& avel,
 const std::string& link_name)
{
	Eigen::VectorXd v_tmp = Eigen::VectorXd::Zero(6);
	v_tmp = CalcPointVelocity6D(*_rbdl_model,_q,_dq,linkId(link_name),Eigen::Vector3d::Zero(),false);
	avel = v_tmp.head(3);
}

void Sai2Model::angularVelocityInWorld(Eigen::Vector3d& avel,
 const std::string& link_name)
{
	Eigen::VectorXd v_tmp = Eigen::VectorXd::Zero(6);
	v_tmp = CalcPointVelocity6D(*_rbdl_model,_q,_dq,linkId(link_name),Eigen::Vector3d::Zero(),false);
	avel = v_tmp.head(3);
	avel = _T_world_robot.linear()*avel;
}

void Sai2Model::angularAcceleration(Eigen::Vector3d& aaccel,
 const std::string& link_name)
{
	Eigen::VectorXd a_tmp = Eigen::VectorXd::Zero(6);
	a_tmp = CalcPointAcceleration6D(*_rbdl_model,_q,_dq,_ddq,linkId(link_name),Eigen::Vector3d::Zero(),false);
	aaccel = a_tmp.head(3);
}

void Sai2Model::angularAccelerationInWorld(Eigen::Vector3d& aaccel,
 const std::string& link_name)
{
	Eigen::VectorXd a_tmp = Eigen::VectorXd::Zero(6);
	a_tmp = CalcPointAcceleration6D(*_rbdl_model,_q,_dq,_ddq,linkId(link_name),Eigen::Vector3d::Zero(),false);
	aaccel = a_tmp.head(3);
	aaccel = _T_world_robot.linear()*aaccel;
}

unsigned int Sai2Model::linkId(const std::string& link_name)
{
	int link_id = -1;
	if(_link_names_map.find(link_name) == _link_names_map.end()) 
	{
		std::cout << "link ["+link_name+"] does not exists\n";
		throw std::runtime_error("link does not exist");
	} else 
	{
		link_id = _link_names_map[link_name];
	}
	return link_id;
}

int Sai2Model::jointId(const std::string& joint_name)
{
	int joint_id = -1;
	if(_joint_names_map.find(joint_name) == _joint_names_map.end())
	{
		std::cout << "joint ["+joint_name+"] does not exists\n";
		throw std::runtime_error("joint does not exist");
	}
	else
	{
		joint_id = _joint_names_map[joint_name];
	}
	return joint_id;
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

void Sai2Model::comPosition(Eigen::Vector3d& robot_com) 
{
	robot_com.setZero();
	double mass;
	double robot_mass = 0.0;
	Eigen::Vector3d center_of_mass_local;
	Eigen::Vector3d center_of_mass_global_frame;
	Eigen::Matrix3d inertia;
	int n_bodies = _rbdl_model->mBodies.size();
	for(int i=0; i<n_bodies; i++) 
	{
		RigidBodyDynamics::Body b = _rbdl_model->mBodies[i];
		
		mass = b.mMass;
		center_of_mass_local = b.mCenterOfMass;

		center_of_mass_global_frame = CalcBodyToBaseCoordinates(*_rbdl_model, _q, i, center_of_mass_local, false);

		robot_com += center_of_mass_global_frame*mass;
		robot_mass += mass;
	}
	robot_com = robot_com/robot_mass;
}

void Sai2Model::comJacobian(Eigen::MatrixXd& Jv_com) {
	Jv_com.setZero(3, _dof);
	Eigen::MatrixXd link_Jv;
	double mass;
	double robot_mass = 0.0;
	Eigen::Vector3d center_of_mass_local;
	Eigen::Matrix3d inertia;
	int n_bodies = _rbdl_model->mBodies.size();
	for(int i=0; i<n_bodies; i++) 
	{
		RigidBodyDynamics::Body b = _rbdl_model->mBodies[i];

		mass = b.mMass;
		center_of_mass_local = b.mCenterOfMass;
		inertia = b.mInertia;

		link_Jv.setZero(3, _dof);
		CalcPointJacobian(*_rbdl_model, _q, i, center_of_mass_local, link_Jv, false);

		Jv_com += link_Jv*mass;
		robot_mass += mass;
	}
	Jv_com = Jv_com/robot_mass;
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
	N = Eigen::MatrixXd::Identity(_dof, _dof);
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

void Sai2Model::addEnvironmentalContact(const std::string link, 
                const Eigen::Vector3d pos_in_link,
                const int constraints_in_rotation,
                const Eigen::Matrix3d orientation)
{
	_environmental_contacts.push_back(ContactModel(link, pos_in_link, constraints_in_rotation, orientation));
}

void Sai2Model::deleteEnvironmentalContact(const std::string link_name)
{
	std::vector<ContactModel> new_contacts;
	for(std::vector<ContactModel>::iterator it = _environmental_contacts.begin(); it!=_environmental_contacts.end(); ++it)
	{
		if(it->_link_name != link_name)
		{
			new_contacts.push_back(*it);
		}
	}
	_environmental_contacts = new_contacts;
}

void Sai2Model::addManipulationContact(const std::string link, 
                const Eigen::Vector3d pos_in_link,
                const int constraints_in_rotation,
                const Eigen::Matrix3d orientation)
{
	_manipulation_contacts.push_back(ContactModel(link, pos_in_link, constraints_in_rotation, orientation));
}

void Sai2Model::deleteManipulationContact(const std::string link_name)
{
	std::vector<ContactModel> new_contacts;
	for(std::vector<ContactModel>::iterator it = _manipulation_contacts.begin(); it!=_manipulation_contacts.end(); ++it)
	{
		if(it->_link_name != link_name)
		{
			new_contacts.push_back(*it);
		}
	}
	_manipulation_contacts = new_contacts;
}

void Sai2Model::manipulationGraspMatrix(Eigen::MatrixXd& G,
                 Eigen::Matrix3d& R,
                 const Eigen::Vector3d center_point)
{
	std::vector<Eigen::Vector3d> contact_locations;
	std::vector<int> constrained_rotations;
	for(int i = 0 ; i < _manipulation_contacts.size() ; i++)
	{
		Eigen::Vector3d pi;
		positionInWorld(pi, _manipulation_contacts[i]._link_name, _manipulation_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		constrained_rotations.push_back(_manipulation_contacts[i]._constrained_rotations);
	}
	graspMatrix(G , R , center_point , contact_locations, constrained_rotations);
}

void Sai2Model::manipulationGraspMatrixAtGeometricCenter(Eigen::MatrixXd& G,
                 Eigen::Matrix3d& R,
                 Eigen::Vector3d& geometric_center)
{
	std::vector<Eigen::Vector3d> contact_locations;
	std::vector<int> constrained_rotations;
	for(int i = 0 ; i < _manipulation_contacts.size() ; i++)
	{
		Eigen::Vector3d pi;
		positionInWorld(pi, _manipulation_contacts[i]._link_name, _manipulation_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		constrained_rotations.push_back(_manipulation_contacts[i]._constrained_rotations);
	}
	graspMatrixAtGeometricCenter(G , R , geometric_center , contact_locations, constrained_rotations);
}

void Sai2Model::environmentalGraspMatrix(Eigen::MatrixXd& G,
                 Eigen::Matrix3d& R,
                 const Eigen::Vector3d center_point)
{
	std::vector<Eigen::Vector3d> contact_locations;
	std::vector<int> constrained_rotations;
	for(int i = 0 ; i < _environmental_contacts.size() ; i++)
	{
		Eigen::Vector3d pi;
		positionInWorld(pi, _environmental_contacts[i]._link_name, _environmental_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		constrained_rotations.push_back(_environmental_contacts[i]._constrained_rotations);
	}
	graspMatrix(G , R , center_point , contact_locations, constrained_rotations);
}

void Sai2Model::environmentalGraspMatrixAtGeometricCenter(Eigen::MatrixXd& G,
                 Eigen::Matrix3d& R,
                 Eigen::Vector3d& geometric_center)
{
	std::vector<Eigen::Vector3d> contact_locations;
	std::vector<int> constrained_rotations;
	for(int i = 0 ; i < _environmental_contacts.size() ; i++)
	{
		Eigen::Vector3d pi;
		positionInWorld(pi, _environmental_contacts[i]._link_name, _environmental_contacts[i]._contact_position);
		contact_locations.push_back(pi);
		constrained_rotations.push_back(_environmental_contacts[i]._constrained_rotations);
	}
	graspMatrixAtGeometricCenter(G , R , geometric_center , contact_locations, constrained_rotations);
}

void Sai2Model::Sai2Model::displayJoints()
{
	std::cout << "\nRobot Joints :" << std::endl;
	for(std::map<std::string,int>::iterator it = _joint_names_map.begin(); it!=_joint_names_map.end(); ++it)
	{
		std::cout << "joint : " << it->first << "\t id : " << it->second << std::endl;
	}
	std::cout << std::endl;
}

void Sai2Model::displayLinks()
{
	std::cout << "\nRobot Joints :" << std::endl;
	for(std::map<std::string,int>::iterator it = _link_names_map.begin(); it!=_link_names_map.end(); ++it)
	{
		std::cout << "link : " << it->first << "\t id : " << it->second << std::endl;
	}
	std::cout << std::endl;
}


// TODO : Untested
void orientationError(Eigen::Vector3d& delta_phi,
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

void orientationError(Eigen::Vector3d& delta_phi,
		              const Eigen::Quaterniond& desired_orientation,
		              const Eigen::Quaterniond& current_orientation)
{
	Eigen::Quaterniond inv_dlambda = desired_orientation*current_orientation.conjugate();
	delta_phi = 2.0*inv_dlambda.vec();
}

Eigen::Matrix3d CrossProductOperator(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d v_hat;
    v_hat << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return v_hat;
}

void graspMatrix(Eigen::MatrixXd& G,
	Eigen::Matrix3d& R,
	const Eigen::Vector3d center_point,
	const std::vector<Eigen::Vector3d>& contact_locations,
	const std::vector<int> constrained_rotations)
{
	G = Eigen::MatrixXd::Zero(1,1);
	R = Eigen::Matrix3d::Identity();

	// number of contact points
	int n = contact_locations.size();
	if(n < 2)
	{
		throw std::invalid_argument("invalid number of contact points (2 points min)\n");
	}
	if(n > 4)
	{
		throw std::invalid_argument("invalid number of contact points (4 points max)\n");
	}
	if(constrained_rotations.size() != n)
	{
		throw std::invalid_argument("argument contact_locations and constrained_rotations need to be of the same length\n");
	}

	// TODO : support line contact
	// number of surface contacts (that can apply a moment)
	int k=0;
	for(int i=0 ; i<n ; i++)
	{
		if(constrained_rotations[i] == 1)
		{
			throw std::invalid_argument("line contact not supported in grasp matrix\n");
		}
		else if(constrained_rotations[i] == 3)
		{
			k++;
		}
		else if(constrained_rotations[i] == 2)
		{
			k++;
		}
		else if(constrained_rotations[i] == 0)
		{}
		else
		{
			throw std::invalid_argument("invalid number of constrained rotations in one of the contacts\n");
		}
	}

	Eigen::MatrixXd Wf = Eigen::MatrixXd::Zero(6, 3*n);
	Eigen::MatrixXd Wm = Eigen::MatrixXd::Zero(6, 3*k);


	for(int i=0; i<n; i++)
	{
		Eigen::Vector3d ri = contact_locations[i] - center_point;
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
			Eigen::Vector3d e12 = contact_locations[1] - contact_locations[0];
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

			// Wf = Rr.transpose() * Wf;

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

					// Wm = Rr.transpose()*Wm;

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

					// Wm = Rr.transpose()*Wm;

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
			Eigen::Vector3d e12 = contact_locations[1] - contact_locations[0];
			Eigen::Vector3d e13 = contact_locations[2] - contact_locations[0];
			Eigen::Vector3d e23 = contact_locations[2] - contact_locations[1];

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
			Eigen::Vector3d e12 = contact_locations[1] - contact_locations[0];
			Eigen::Vector3d e13 = contact_locations[2] - contact_locations[0];
			Eigen::Vector3d e14 = contact_locations[3] - contact_locations[0];
			Eigen::Vector3d e23 = contact_locations[2] - contact_locations[1];
			Eigen::Vector3d e24 = contact_locations[3] - contact_locations[1];
			Eigen::Vector3d e34 = contact_locations[3] - contact_locations[2];

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

void graspMatrixAtGeometricCenter(Eigen::MatrixXd& G,
                     Eigen::Matrix3d& R,
                     Eigen::Vector3d& geometric_center,
                     const std::vector<Eigen::Vector3d>& contact_locations,
					 const std::vector<int> constrained_rotations)
{
	// number of contact points
	int n = contact_locations.size();
	if(n < 2)
	{
		throw std::invalid_argument("invalid number of contact points (2 points min)\n");
	}
	if(n > 4)
	{
		throw std::invalid_argument("invalid number of contact points (4 points max)\n");
	}

	geometric_center.setZero();

	for(int i=0; i<n; i++)
	{
		geometric_center += contact_locations[i];
	}
	geometric_center = geometric_center/(double)n;

	graspMatrix(G, R, geometric_center , contact_locations, constrained_rotations);
}

} /* namespace Model */

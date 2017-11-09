/*
 * RbdlModel.cpp
 * 
 *  Wrapper around RBDL plus functions to facilitate the whole body control framework from Stanford robotics lab
 *
 *  Created on: Dec 14, 2016
 *      Author: Mikael Jorda
 */

#include "RBDLModel.h"

#include <rbdl/rbdl.h>
#include <URDFToRBDLModel.h>

#include <stdexcept>

namespace Model
{

RBDLModel::RBDLModel (const std::string path_to_model_file, bool verbose)
{

	// parse rbdl model from urdf
	bool success = RigidBodyDynamics::URDFReadFromFile(path_to_model_file.c_str(), &_rbdl_model, false, verbose);
	if (!success) 
	{
		std::cout << "Error loading model [" + path_to_model_file + "]" << "\n";
	}

	// set the number of degrees of freedom
	_dof = _rbdl_model.dof_count;

	// TODO : support other initial joint configuration
	updateModel(Eigen::VectorXd::Zero(_dof), Eigen::VectorXd::Zero(_dof), Eigen::VectorXd::Zero(_dof));
}


RBDLModel::~RBDLModel (){}


void RBDLModel::updateModel(const Eigen::VectorXd& q,
	const Eigen::VectorXd& dq,
	const Eigen::VectorXd& ddq)
{
	UpdateKinematicsCustom(_rbdl_model, &q, &dq, &ddq);
}


void RBDLModel::massMatrix(Eigen::MatrixXd& A,
	const Eigen::VectorXd& q)
{
	if (A.rows()!=_dof|| A.cols()!=_dof){
	A.setZero(_dof,_dof);
	}

	CompositeRigidBodyAlgorithm(_rbdl_model, q, A, false);
}


void RBDLModel::gravityVector(Eigen::VectorXd& g,
	const Eigen::VectorXd& q,
	const Eigen::Vector3d& gravity)
{

	if (g.size() != _dof){ g.resize(_dof); }
	g.setZero();

	std::vector<RigidBodyDynamics::Body>::iterator it_body;
	int body_id;

	for (it_body = _rbdl_model.mBodies.begin(), body_id=0;
	it_body != _rbdl_model.mBodies.end();
	++it_body, ++body_id)
	{
		double mass = it_body->mMass;
		Eigen::MatrixXd Jv = Eigen::MatrixXd::Zero(3, _dof);
		CalcPointJacobian(_rbdl_model, q, body_id, it_body->mCenterOfMass, Jv, false);

		g += Jv.transpose() * (-mass * gravity);
	}
}


void RBDLModel::coriolisForce(Eigen::VectorXd& b,
	const Eigen::VectorXd& q,
	const Eigen::VectorXd& dq)
{
	NonlinearEffects(_rbdl_model,q,dq,b);
}

void RBDLModel::J_0(Eigen::MatrixXd& J,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link,
	const Eigen::VectorXd& q)
{
	if (J.rows()!=6 || J.cols()!=_dof) // resize to the right format
	{
		J.setZero(6,_dof);
	}
	Eigen::MatrixXd J_temp = Eigen::MatrixXd::Zero(6,_dof);
	CalcPointJacobian6D (_rbdl_model, q, linkId(link_name), pos_in_link, J_temp, false);

	// RBDL gives Jw as the top 3 rows and Jv as the bottom part. We need to swap it here
	J << J_temp.block(3,0,3,_dof),
		 J_temp.block(0,0,3,_dof);
}

void RBDLModel::J(Eigen::MatrixXd& J,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link,
	const Eigen::VectorXd& q)
{
	if (J.rows()!=6 || J.cols()!=_dof) // resize to the right format
	{
		J.setZero(6,_dof);
	}
	CalcPointJacobian6D (_rbdl_model, q, linkId(link_name), pos_in_link, J, false);
}



void RBDLModel::Jv(Eigen::MatrixXd& J,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link,
	const Eigen::VectorXd& q)
{
	if (J.rows()!=3 || J.cols()!=_dof)
	{
		J.setZero(3,_dof);
	}

	CalcPointJacobian(_rbdl_model, q, linkId(link_name), pos_in_link, J, false);
}



void RBDLModel::Jw(Eigen::MatrixXd& J,
 const std::string& link_name,
 const Eigen::VectorXd& q)
{
	// compute the full jacobian at the center of the link and take rotational part
	Eigen::MatrixXd J_temp = Eigen::MatrixXd::Zero(6,_dof);
	CalcPointJacobian6D (_rbdl_model, q, linkId(link_name), Eigen::Vector3d::Zero(), J_temp, false);
	J = J_temp.topRows<3>();
}



void RBDLModel::transform(Eigen::Affine3d& T,
 const std::string& link_name,
 const Eigen::VectorXd& q)
{
	unsigned int link_id = linkId(link_name);
	Eigen::Vector3d pos_in_body(0,0,0);
	T.linear() = CalcBodyWorldOrientation(_rbdl_model, q, link_id, false).transpose();
	T.translation() = CalcBodyToBaseCoordinates(_rbdl_model, q, link_id, pos_in_body, false);
}


void RBDLModel::position(Eigen::Vector3d& pos,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link,
	const Eigen::VectorXd& q)
{
	pos = CalcBodyToBaseCoordinates(_rbdl_model, q, linkId(link_name), pos_in_link, false);
}


void RBDLModel::linearVelocity(Eigen::Vector3d& vel,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link,
	const Eigen::VectorXd& q,
	const Eigen::VectorXd& dq)
{
	vel = CalcPointVelocity(_rbdl_model,q,dq,linkId(link_name),pos_in_link,false);
}


void RBDLModel::linearAcceleration(Eigen::Vector3d& accel,
	const std::string& link_name,
	const Eigen::Vector3d& pos_in_link,
	const Eigen::VectorXd& q,
	const Eigen::VectorXd& dq,
	const Eigen::VectorXd& ddq)
{
	accel = CalcPointAcceleration(_rbdl_model,q,dq,ddq,linkId(link_name),pos_in_link,false);
}


void RBDLModel::rotation(Eigen::Matrix3d& rot,
	const std::string& link_name,
	const Eigen::VectorXd& q)
{
	rot = CalcBodyWorldOrientation(_rbdl_model, q, linkId(link_name), false).transpose();
}


void RBDLModel::angularVelocity(Eigen::Vector3d& avel,
 const std::string& link_name,
 const Eigen::VectorXd& q,
 const Eigen::VectorXd& dq)
{
	Eigen::VectorXd v_tmp = Eigen::VectorXd::Zero(6);
	v_tmp = CalcPointVelocity6D(_rbdl_model,q,dq,linkId(link_name),Eigen::Vector3d::Zero(),false);
	avel = v_tmp.head(3);
}


void RBDLModel::angularAcceleration(Eigen::Vector3d& aaccel,
 const std::string& link_name,
 const Eigen::VectorXd& q,
 const Eigen::VectorXd& dq,
 const Eigen::VectorXd& ddq)
{
	Eigen::VectorXd a_tmp = Eigen::VectorXd::Zero(6);
	a_tmp = CalcPointAcceleration6D(_rbdl_model,q,dq,ddq,linkId(link_name),Eigen::Vector3d::Zero(),false);
	aaccel = a_tmp.head(3);
}


unsigned int RBDLModel::linkId(const std::string& link_name)
{
	auto iter = _rbdl_model.mBodyNameMap.find(link_name);
	unsigned int body_id = iter->second;

	if (iter == _rbdl_model.mBodyNameMap.end()) {
	std::cout << "link ["+link_name+"] does not exists\n";
	}

	return body_id;
}


void RBDLModel::getLinkMass(double& mass,
 Eigen::Vector3d& center_of_mass,
 Eigen::Matrix3d& inertia,
 const std::string& link_name)
{
	RigidBodyDynamics::Body b = _rbdl_model.mBodies[linkId(link_name)];

	mass = b.mMass;
	center_of_mass = b.mCenterOfMass;
	inertia = b.mInertia;
}

void RBDLModel::getLinkMass(double& mass,
 Eigen::Vector3d& center_of_mass,
 const std::string& link_name)
{
	RigidBodyDynamics::Body b = _rbdl_model.mBodies[linkId(link_name)];

	mass = b.mMass;
	center_of_mass = b.mCenterOfMass;
}


} /* namespace Model */

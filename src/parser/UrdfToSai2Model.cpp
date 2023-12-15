#include "UrdfToSai2Model.h"

#include <assert.h>
#include <rbdl/rbdl.h>
#include <urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h>

#include <fstream>
#include <iostream>
#include <map>
#include <stack>

typedef my_shared_ptr<Sai2Urdfreader::Link> LinkPtr;
typedef const my_shared_ptr<const Sai2Urdfreader::Link> ConstLinkPtr;
typedef my_shared_ptr<Sai2Urdfreader::Joint> JointPtr;
typedef my_shared_ptr<Sai2Urdfreader::ModelInterface> ModelPtr;

using namespace std;

namespace RigidBodyDynamics {

using namespace Math;

typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr> URDFLinkMap;
typedef map<string, JointPtr> URDFJointMap;

bool construct_model(
	Model* rbdl_model, ModelPtr urdf_model,
	std::map<std::string, int>& link_names_to_id_map,
	std::map<std::string, int>& joint_names_to_id_map,
	std::map<std::string, std::string>& joint_names_to_child_link_names_map,
	std::vector<Sai2Model::JointLimit>& joint_limits, bool floating_base,
	bool verbose) {
	LinkPtr urdf_root_link;

	URDFLinkMap link_map;
	link_map = urdf_model->links_;

	URDFJointMap joint_map;
	joint_map = urdf_model->joints_;

	vector<string> joint_names;

	stack<LinkPtr> link_stack;
	stack<int> joint_index_stack;

	// add the bodies in a depth-first order of the model tree
	link_stack.push(link_map[(urdf_model->getRoot()->name)]);

	// add the root body
	ConstLinkPtr& root = urdf_model->getRoot();
	Vector3d root_inertial_rpy;
	Vector3d root_inertial_position;
	Matrix3d root_inertial_inertia;
	double root_inertial_mass;
	link_names_to_id_map[root->name] = 0;

	if (root->inertial) {
		root_inertial_mass = root->inertial->mass;

		root_inertial_position.set(root->inertial->origin.position.x,
								   root->inertial->origin.position.y,
								   root->inertial->origin.position.z);

		root_inertial_inertia(0, 0) = root->inertial->ixx;
		root_inertial_inertia(0, 1) = root->inertial->ixy;
		root_inertial_inertia(0, 2) = root->inertial->ixz;

		root_inertial_inertia(1, 0) = root->inertial->ixy;
		root_inertial_inertia(1, 1) = root->inertial->iyy;
		root_inertial_inertia(1, 2) = root->inertial->iyz;

		root_inertial_inertia(2, 0) = root->inertial->ixz;
		root_inertial_inertia(2, 1) = root->inertial->iyz;
		root_inertial_inertia(2, 2) = root->inertial->izz;

		root->inertial->origin.rotation.getRPY(
			root_inertial_rpy[0], root_inertial_rpy[1], root_inertial_rpy[2]);

		Body root_link = Body(root_inertial_mass, root_inertial_position,
							  root_inertial_inertia);

		Joint root_joint(JointTypeFixed);
		if (floating_base) {
			root_joint = JointTypeFloatingBase;
		}

		SpatialTransform root_joint_frame = SpatialTransform();

		if (verbose) {
			cout << "+ Adding Root Body " << endl;
			cout << "  joint frame: " << root_joint_frame << endl;
			if (floating_base) {
				cout << "  joint type : floating" << endl;
			} else {
				cout << "  joint type : fixed" << endl;
			}
			cout << "  body inertia: " << endl << root_link.mInertia << endl;
			cout << "  body mass   : " << root_link.mMass << endl;
			cout << "  body name   : " << root->name << endl;
		}

		rbdl_model->AppendBody(root_joint_frame, root_joint, root_link,
							   root->name);
	}

	if (link_stack.top()->child_joints.size() > 0) {
		joint_index_stack.push(0);
	}

	while (link_stack.size() > 0) {
		LinkPtr cur_link = link_stack.top();
		unsigned int joint_idx = joint_index_stack.top();

		if (joint_idx < cur_link->child_joints.size()) {
			JointPtr cur_joint = cur_link->child_joints[joint_idx];

			// increment joint index
			joint_index_stack.pop();
			joint_index_stack.push(joint_idx + 1);

			link_stack.push(link_map[cur_joint->child_link_name]);
			joint_index_stack.push(0);

			if (verbose) {
				for (unsigned int i = 1; i < joint_index_stack.size() - 1;
					 i++) {
					cout << "  ";
				}
				cout << "joint '" << cur_joint->name << "' child link '"
					 << link_stack.top()->name << "' type = " << cur_joint->type
					 << endl;
			}

			joint_names.push_back(cur_joint->name);
		} else {
			link_stack.pop();
			joint_index_stack.pop();
		}
	}

	unsigned int j;
	unsigned int current_moveable_joint_id = 1; // start with 1 because 0 is the root joint
	for (j = 0; j < joint_names.size(); j++) {
		JointPtr urdf_joint = joint_map[joint_names[j]];
		LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
		LinkPtr urdf_child = link_map[urdf_joint->child_link_name];
		if (urdf_joint->type != Sai2Urdfreader::Joint::FIXED &&
			urdf_joint->type != Sai2Urdfreader::Joint::UNKNOWN) {
			joint_names_to_id_map[urdf_joint->name] = current_moveable_joint_id;
			current_moveable_joint_id++;
		}

		// determine where to add the current joint and child body
		unsigned int rbdl_parent_id = 0;

		if (urdf_parent->name != "base_joint" &&
			rbdl_model->mBodies.size() != 1)
			rbdl_parent_id = rbdl_model->GetBodyId(urdf_parent->name.c_str());

		if (rbdl_parent_id == std::numeric_limits<unsigned int>::max())
			cerr << "Error while processing joint '" << urdf_joint->name
				 << "': parent link '" << urdf_parent->name
				 << "' could not be found." << endl;

		// cout << "joint: " << urdf_joint->name << "\tparent = " <<
		// urdf_parent->name << " child = " << urdf_child->name << " parent_id =
		// " << rbdl_parent_id << endl;

		// create the joint
		Joint rbdl_joint;
		if (urdf_joint->type == Sai2Urdfreader::Joint::REVOLUTE ||
			urdf_joint->type == Sai2Urdfreader::Joint::CONTINUOUS) {
			rbdl_joint =
				Joint(SpatialVector(urdf_joint->axis.x, urdf_joint->axis.y,
									urdf_joint->axis.z, 0., 0., 0.));
		} else if (urdf_joint->type == Sai2Urdfreader::Joint::PRISMATIC) {
			rbdl_joint =
				Joint(SpatialVector(0., 0., 0., urdf_joint->axis.x,
									urdf_joint->axis.y, urdf_joint->axis.z));
		} else if (urdf_joint->type == Sai2Urdfreader::Joint::SPHERICAL) {
			rbdl_joint = Joint(JointTypeSpherical);
		} else if (urdf_joint->type == Sai2Urdfreader::Joint::FIXED) {
			rbdl_joint = Joint(JointTypeFixed);
		} else if (urdf_joint->type == Sai2Urdfreader::Joint::FLOATING) {
			// todo: what order of DoF should be used?
			rbdl_joint = Joint(SpatialVector(0., 0., 0., 1., 0., 0.),
							   SpatialVector(0., 0., 0., 0., 1., 0.),
							   SpatialVector(0., 0., 0., 0., 0., 1.),
							   SpatialVector(1., 0., 0., 0., 0., 0.),
							   SpatialVector(0., 1., 0., 0., 0., 0.),
							   SpatialVector(0., 0., 1., 0., 0., 0.));
		} else if (urdf_joint->type == Sai2Urdfreader::Joint::PLANAR) {
			// todo: which two directions should be used that are perpendicular
			// to the specified axis?
			cerr << "Error while processing joint '" << urdf_joint->name
				 << "': planar joints not yet supported!" << endl;
			return false;
		}

		// compute the joint transformation
		Vector3d joint_rpy;
		Vector3d joint_translation;
		urdf_joint->parent_to_joint_origin_transform.rotation.getRPY(
			joint_rpy[0], joint_rpy[1], joint_rpy[2]);
		joint_translation.set(
			urdf_joint->parent_to_joint_origin_transform.position.x,
			urdf_joint->parent_to_joint_origin_transform.position.y,
			urdf_joint->parent_to_joint_origin_transform.position.z);
		SpatialTransform rbdl_joint_frame =
			Xrot(joint_rpy[0], Vector3d(1., 0., 0.)) *
			Xrot(joint_rpy[1], Vector3d(0., 1., 0.)) *
			Xrot(joint_rpy[2], Vector3d(0., 0., 1.)) *
			Xtrans(Vector3d(joint_translation));

		// assemble the body
		Vector3d link_inertial_position;
		Vector3d link_inertial_rpy;
		Matrix3d link_inertial_inertia = Matrix3d::Zero();
		double link_inertial_mass = 0.;

		// but only if we actually have inertial data
		if (urdf_child->inertial) {
			link_inertial_mass = urdf_child->inertial->mass;

			link_inertial_position.set(urdf_child->inertial->origin.position.x,
									   urdf_child->inertial->origin.position.y,
									   urdf_child->inertial->origin.position.z);
			urdf_child->inertial->origin.rotation.getRPY(link_inertial_rpy[0],
														 link_inertial_rpy[1],
														 link_inertial_rpy[2]);

			link_inertial_inertia(0, 0) = urdf_child->inertial->ixx;
			link_inertial_inertia(0, 1) = urdf_child->inertial->ixy;
			link_inertial_inertia(0, 2) = urdf_child->inertial->ixz;

			link_inertial_inertia(1, 0) = urdf_child->inertial->ixy;
			link_inertial_inertia(1, 1) = urdf_child->inertial->iyy;
			link_inertial_inertia(1, 2) = urdf_child->inertial->iyz;

			link_inertial_inertia(2, 0) = urdf_child->inertial->ixz;
			link_inertial_inertia(2, 1) = urdf_child->inertial->iyz;
			link_inertial_inertia(2, 2) = urdf_child->inertial->izz;

			if (link_inertial_rpy != Vector3d(0., 0., 0.)) {
				cerr << "Error while processing body '" << urdf_child->name
					 << "': rotation of body frames not yet supported. Please "
						"rotate the joint frame instead."
					 << endl;
				return false;
			}
		}

		Body rbdl_body = Body(link_inertial_mass, link_inertial_position,
							  link_inertial_inertia);

		if (verbose) {
			cout << "+ Adding Body " << endl;
			cout << "  parent_id  : " << rbdl_parent_id << endl;
			cout << "  joint frame: " << rbdl_joint_frame << endl;
			cout << "  joint dofs : " << rbdl_joint.mDoFCount << endl;
			for (unsigned int j = 0; j < rbdl_joint.mDoFCount; j++) {
				cout << "    " << j << ": "
					 << rbdl_joint.mJointAxes[j].transpose() << endl;
			}
			cout << "  body inertia: " << endl << rbdl_body.mInertia << endl;
			cout << "  body mass   : " << rbdl_body.mMass << endl;
			cout << "  body name   : " << urdf_child->name << endl;
		}

		if (urdf_joint->type == Sai2Urdfreader::Joint::FLOATING) {
			Matrix3d zero_matrix = Matrix3d::Zero();
			Body null_body(0., Vector3d::Zero(3), zero_matrix);
			Joint joint_txtytz(JointTypeTranslationXYZ);
			string trans_body_name = urdf_child->name + "_Translate";
			rbdl_model->AddBody(rbdl_parent_id, rbdl_joint_frame, joint_txtytz,
								null_body, trans_body_name);

			Joint joint_euler_zyx(JointTypeEulerXYZ);
			rbdl_model->AppendBody(SpatialTransform(), joint_euler_zyx,
								   rbdl_body, urdf_child->name);
		} else {
			rbdl_model->AddBody(rbdl_parent_id, rbdl_joint_frame, rbdl_joint,
								rbdl_body, urdf_child->name);
		}
		link_names_to_id_map[urdf_child->name] =
			rbdl_model->GetBodyId(urdf_child->name.c_str());
		joint_names_to_child_link_names_map[urdf_joint->name] =
			urdf_child->name;

		// get the joint limits
		if (urdf_joint->limits) {
			if (urdf_joint->type != Sai2Urdfreader::Joint::REVOLUTE &&
				urdf_joint->type != Sai2Urdfreader::Joint::PRISMATIC) {
				cerr << "error while processing limits on joint '"
					 << urdf_joint->name
					 << "': limits are only supported on revolute and "
						"prismatic joints."
					 << endl;
				return false;
			}
			if (urdf_joint->limits->upper <= urdf_joint->limits->lower) {
				cerr << "error while processing limits on joint '"
					 << urdf_joint->name
					 << "': lower limit cannot be equal or higher than upper "
						"limit"
					 << endl;
				return false;
			}
			const auto& lim = urdf_joint->limits;
			joint_limits.push_back(Sai2Model::JointLimit(
				urdf_joint->name, rbdl_model->mJoints[j + 1].q_index,
				lim->lower, lim->upper, lim->velocity, lim->effort));
		}
	}

	return true;
}

RBDL_DLLAPI bool URDFReadFromFile(
	const char* filename, Model* model,
	std::map<std::string, int>& link_names_to_id_map,
	std::map<std::string, int>& joint_names_to_id_map,
	std::map<std::string, std::string>& joint_names_to_child_link_names_map,
	std::vector<Sai2Model::JointLimit>& joint_limits, bool floating_base,
	bool verbose) {
	ifstream model_file(filename);
	if (!model_file) {
		cerr << "Error opening file '" << filename << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
							std::istreambuf_iterator<char>());

	model_file.close();

	return URDFReadFromString(model_xml_string.c_str(), model,
							  link_names_to_id_map, joint_names_to_id_map,
							  joint_names_to_child_link_names_map, joint_limits,
							  floating_base, verbose);
}

RBDL_DLLAPI bool URDFReadFromString(
	const char* model_xml_string, Model* model,
	std::map<std::string, int>& link_names_to_id_map,
	std::map<std::string, int>& joint_names_to_id_map,
	std::map<std::string, std::string>& joint_names_to_child_link_names_map,
	std::vector<Sai2Model::JointLimit>& joint_limits, bool floating_base,
	bool verbose) {
	assert(model);

	ModelPtr urdf_model = Sai2Urdfreader::parseURDF(model_xml_string);

	if (!construct_model(model, urdf_model, link_names_to_id_map,
						 joint_names_to_id_map,
						 joint_names_to_child_link_names_map, joint_limits,
						 floating_base, verbose)) {
		cerr << "Error constructing model from urdf file." << endl;
		return false;
	}

	return true;
}
}  // namespace RigidBodyDynamics

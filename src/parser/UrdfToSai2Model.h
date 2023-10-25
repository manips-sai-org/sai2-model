#ifndef SAI2_URDF_TO_RBDLMODEL_H
#define SAI2_URDF_TO_RBDLMODEL_H

#include <rbdl/rbdl_config.h>

#include "JointLimits.h"
#include "urdf_parser/urdf_parser.h"

namespace RigidBodyDynamics {

struct Model;

RBDL_DLLAPI bool URDFReadFromFile(
	const char* filename, Model* model,
	std::map<std::string, int>& link_names_to_id_map,
	std::map<std::string, int>& joint_names_to_id_map,
	std::map<std::string, std::string>& joint_names_to_child_link_names_map,
	std::vector<Sai2Model::JointLimit>& joint_limits, bool floating_base,
	bool verbose = false);

RBDL_DLLAPI bool URDFReadFromString(
	const char* model_xml_string, Model* model,
	std::map<std::string, int>& link_names_to_id_map,
	std::map<std::string, int>& joint_names_to_id_map,
	std::map<std::string, std::string>& joint_names_to_child_link_names_map,
	std::vector<Sai2Model::JointLimit>& joint_limits, bool floating_base,
	bool verbose = false);
}  // namespace RigidBodyDynamics

/* _RBDL_URDFREADER_H */
#endif

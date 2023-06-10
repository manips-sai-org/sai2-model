#ifndef SAI2_URDF_TO_RBDLMODEL_H
#define SAI2_URDF_TO_RBDLMODEL_H

#include <rbdl/rbdl_config.h>
#include "urdf_parser/urdf_parser.h"
#include "JointLimits.h"


namespace RigidBodyDynamics {

struct Model;

RBDL_DLLAPI bool URDFReadFromFile(
	const char* filename, Model* model,
	std::map<std::string, int>& link_names_to_id_map,
	std::map<std::string, int>& joint_names_to_id_map,
    std::vector<Sai2Model::JointLimit>& joint_limits,
    bool floating_base,
	bool verbose = false,
	Eigen::Vector3d world_gravity = Eigen::Vector3d(0.0, 0.0, -9.81));

RBDL_DLLAPI bool URDFReadFromString(
	const char* model_xml_string, Model* model,
	std::map<std::string, int>& link_names_to_id_map,
	std::map<std::string, int>& joint_names_to_id_map,
    std::vector<Sai2Model::JointLimit>& joint_limits,
    bool floating_base,
	bool verbose = false,
	Eigen::Vector3d world_gravity = Eigen::Vector3d(0.0, 0.0, -9.81));
}  // namespace RigidBodyDynamics

/* _RBDL_URDFREADER_H */
#endif

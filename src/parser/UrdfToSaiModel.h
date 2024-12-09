#ifndef SAI_URDF_TO_RBDLMODEL_H
#define SAI_URDF_TO_RBDLMODEL_H

#include <rbdl/rbdl_config.h>

#include "JointLimits.h"
#include "urdf_parser/urdf_parser.h"

namespace RigidBodyDynamics {

struct Model;

/**
 * @brief Populates a rbdl model from a URDF file. Effectively reads the urdf
 * file, converts it as a string and calls URDFReadFromString.
 *
 * @param filename The path to the URDF file
 * @param model The model to be populated
 * @param link_names_to_id_map A map from link names to their corresponding id
 * in rbdl, populated by the function
 * @param joint_names_to_id_map A map from joint names to their corresponding id
 * in rbdl, populated by the function
 * @param initial_joint_positions A map from joint ids to their initial
 * positions, populated by the function
 * @param joint_names_to_child_link_names_map A map from joint names to their
 * child link names, populated by the function
 * @param joint_names_to_parent_link_names_map A map from joint names to their
 * parent link names, populated by the function
 * @param joint_limits A vector of joint limits for the robot joints, populated
 * by the function
 * @param floating_base Whether the model has a floating base
 * @param verbose Whether to print debug information
 * @return RBDL_DLLAPI
 */
RBDL_DLLAPI bool URDFReadFromFile(
	const char* filename, Model* model,
	std::map<std::string, int>& link_names_to_id_map,
	std::map<std::string, int>& joint_names_to_id_map,
	std::map<int, double>& initial_joint_positions,
	std::map<std::string, std::string>& joint_names_to_child_link_names_map,
	std::map<std::string, std::string>& joint_names_to_parent_link_names_map,
	std::vector<SaiModel::JointLimit>& joint_limits, bool floating_base,
	bool verbose = false);

/**
 * @brief Populates a rbdl model from a URDF string.
 *
 * @param model_xml_string  The URDF string
 * @param model  The model to be populated
 * @param link_names_to_id_map  A map from link names to their corresponding id
 * in rbdl, populated by the function
 * @param joint_names_to_id_map  A map from joint names to their corresponding
 * id in rbdl, populated by the function
 * @param initial_joint_positions  A map from joint ids to their initial
 * positions, populated by the function
 * @param joint_names_to_child_link_names_map  A map from joint names to their
 * child link names, populated by the function
 * @param joint_names_to_parent_link_names_map  A map from joint names to their
 * parent link names, populated by the function
 * @param joint_limits  A vector of joint limits for the robot joints, populated
 * by the function
 * @param floating_base  Whether the model has a floating base
 * @param verbose  Whether to print debug information
 * @return RBDL_DLLAPI
 */
RBDL_DLLAPI bool URDFReadFromString(
	const char* model_xml_string, Model* model,
	std::map<std::string, int>& link_names_to_id_map,
	std::map<std::string, int>& joint_names_to_id_map,
	std::map<int, double>& initial_joint_positions,
	std::map<std::string, std::string>& joint_names_to_child_link_names_map,
	std::map<std::string, std::string>& joint_names_to_parent_link_names_map,
	std::vector<SaiModel::JointLimit>& joint_limits, bool floating_base,
	bool verbose = false);
}  // namespace RigidBodyDynamics

/* _RBDL_URDFREADER_H */
#endif

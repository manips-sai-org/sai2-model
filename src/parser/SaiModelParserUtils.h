#ifndef SAI_MODEL_PARSER_UTILS_H_
#define SAI_MODEL_PARSER_UTILS_H_

#include <map>
#include <string>

namespace SaiModel {

extern std::map<std::string, std::string> URDF_FOLDERS;

/**
 * @brief This function takes a string as input, and if it starts with one of
 * the keys (in between brackets and prefixed by the character $) in the
 * URDF_FOLDERS map, it replaces the key with the corresponding value.
 * 
 * For example, if the URDF_FOLDERS map is {"SOME_PATH": "/path/to/some/folder"},
 * and the input string is "${SOME_PATH}/urdf_file.urdf", the output will be
 * "/path/to/some/folder/urdf_file.urdf".
 *
 * @param path the input string
 * @return the potentially replaced string
 */
std::string ReplaceUrdfPathPrefix(const std::string& path);

}  // namespace SaiModel

#endif	// SAI_MODEL_PARSER_UTILS_H_
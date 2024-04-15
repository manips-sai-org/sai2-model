#ifndef SAI2_MODEL_PARSER_UTILS_H_
#define SAI2_MODEL_PARSER_UTILS_H_

#include <map>
#include <string>

namespace Sai2Model {

extern std::map<std::string, std::string> URDF_FOLDERS;

std::string ReplaceUrdfPathPrefix(const std::string& path);

}  // namespace Sai2Model

#endif	// SAI2_MODEL_PARSER_UTILS_H_
#include "Sai2ModelParserUtils.h"

#include "Sai2Model_UrdfFolder.h"

namespace Sai2Model {

std::map<std::string, std::string> URDF_FOLDERS;

// \cond
// automatic initialization of the URDF folders
struct UrdfFolderInitializer {
	UrdfFolderInitializer() {
		URDF_FOLDERS["SAI2_MODEL_URDF_FOLDER"] =
			std::string(SAI2_MODEL_URDF_FOLDER);
	}
};
static UrdfFolderInitializer urdfFolderInitializer;
// \endcond

std::string ReplaceUrdfPathPrefix(const std::string& path) {
	for (const auto& folder : URDF_FOLDERS) {
		const std::string macroPrefix = std::string("${") + folder.first + "}";
		if (path.find(macroPrefix) == 0) {
			return folder.second + path.substr(macroPrefix.length());
		}
	}
	return path;
}
}  // namespace Sai2Model
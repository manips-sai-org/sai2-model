#ifndef SAI2_URDF_TO_RBDLMODEL_H
#define SAI2_URDF_TO_RBDLMODEL_H

#include <rbdl/rbdl_config.h>
#include "urdf_parser/urdf_parser.h"


namespace RigidBodyDynamics {

struct Model;

	RBDL_DLLAPI bool URDFReadFromFile (const char* filename, Model* model, bool floating_base, bool verbose = false);
	RBDL_DLLAPI bool URDFReadFromString (const char* model_xml_string, Model* model, bool floating_base, bool verbose = false);


}

/* _RBDL_URDFREADER_H */
#endif

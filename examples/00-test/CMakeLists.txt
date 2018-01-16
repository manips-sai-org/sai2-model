# add the include directory to the include paths
INCLUDE_DIRECTORIES (
	${SAI2-MODELINCLUDE_DIRS})

# create an executable
ADD_EXECUTABLE (00-test main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (00-test
	${SAI2-MODEL_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
FILE(COPY rpbot.urdf rrbot.urdf DESTINATION ${RESOURCE_DIR})

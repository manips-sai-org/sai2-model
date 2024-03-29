# add the include directory to the include paths
SET(EXAMPLE_NAME 04-joint_limits)

# create an executable
ADD_EXECUTABLE (${EXAMPLE_NAME} main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (${EXAMPLE_NAME}
	${SAI2-MODEL_EXAMPLES_LIBRARIES})

set(SAI2-MODEL_EXAMPLES_LIBRARIES ${SAI2-MODEL_LIBRARIES}
                                  ${SAI2-URDF_LIBRARIES})

set(EXAMPLES_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DEXAMPLES_FOLDER="${EXAMPLES_FOLDER}")

add_subdirectory(01-create_model_from_file)
add_subdirectory(02-update_model)
add_subdirectory(03-spherical_joints)
add_subdirectory(04-joint_limits)
add_subdirectory(05-grasp_matrix)
add_subdirectory(06-robot_arms)

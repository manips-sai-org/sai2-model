find_package(GTest REQUIRED)

set(CMAKE_TEST_OUTPUT_DIRECTORY Tests_Sai2Model)

# add unit tests
add_executable(tests
    Tests_Sai2Model.cpp
)

target_link_libraries(tests
    PRIVATE ${SAI2-MODEL_LIBRARIES}
    PRIVATE ${SAI2-URDF_LIBRARIES}
	PRIVATE GTest::GTest
	PRIVATE GTest::Main
)

# copy the tests urdfs
add_custom_command(
  TARGET tests
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  "${CMAKE_CURRENT_SOURCE_DIR}/urdf"
  "${CMAKE_CURRENT_BINARY_DIR}/urdf"
)

# Enable testing with CTest
enable_testing()

# Add a test that runs the test executable
add_test(NAME Tests_Sai2Model
    COMMAND tests
)
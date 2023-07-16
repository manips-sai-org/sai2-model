#include <Sai2Model.h>
#include <gtest/gtest.h>

namespace Sai2Model {

using namespace Eigen;

const std::string rrp_urdf = "./urdf/rrpbot.urdf";
const std::string rpspr_urdf = "./urdf/rpsprbot.urdf";

class Sai2ModelTest : public ::testing::Test {
protected:
	void SetUp() override {
		model_rrpbot = new Sai2Model(rrp_urdf);
		model_rpsprbot = new Sai2Model(rpspr_urdf);
	}

	void TearDown() override {
		delete model_rrpbot;
		delete model_rpsprbot;
	}

	Sai2Model* model_rrpbot;
	Sai2Model* model_rpsprbot;
};

template <typename DerivedA, typename DerivedB>
bool checkEigenMatricesEqual(const Eigen::MatrixBase<DerivedA>& expected,
							 const Eigen::MatrixBase<DerivedB>& actual,
							 double epsilon = 1e-4) {
	if (expected.rows() != actual.rows()) {
		std::cout
			<< "not the same number of rows between expected and actual Matrix"
			<< endl;
		return false;
	}
	if (expected.cols() != actual.cols()) {
		std::cout
			<< "not the same number of cols between expected and actual Matrix"
			<< endl;
		return false;
	}

	bool equal = true;
	for (int i = 0; i < expected.rows(); ++i) {
		for (int j = 0; j < expected.cols(); ++j) {
			if (fabs(expected(i, j) - actual(i, j)) > epsilon) {
				equal = false;
				std::cout << "Mismatch found at index (" << i << ", " << j
						  << "):\n"
						  << "Expected: " << expected(i, j) << "\n"
						  << "Actual:   " << actual(i, j) << "\n";
			}
		}
	}
	return equal;
}

TEST_F(Sai2ModelTest, ConstructDescruct) {}

TEST_F(Sai2ModelTest, DofAndQsize) {
	EXPECT_EQ(model_rrpbot->dof(), 3);
	EXPECT_EQ(model_rrpbot->qSize(), 3);
	EXPECT_EQ(model_rpsprbot->dof(), 7);
	EXPECT_EQ(model_rpsprbot->qSize(), 8);
}

TEST_F(Sai2ModelTest, SetAndGetQ) {
	// invalid q size
	VectorXd q = VectorXd::Zero(model_rrpbot->qSize() + 1);
	EXPECT_THROW(model_rrpbot->setQ(q), std::invalid_argument);

	// invalid quaternion
	q.setZero(model_rpsprbot->qSize());
	EXPECT_THROW(model_rpsprbot->setQ(q), std::invalid_argument);

	// valid q
	q.setZero(model_rrpbot->qSize());
	q << 0.1, 0.2, 0.3;
	model_rrpbot->setQ(q);
	EXPECT_EQ(q, model_rrpbot->q());

	// valid quaternion
	q.setZero(model_rpsprbot->qSize());
	q << 0.1, 0.2, sqrt(0.1), sqrt(0.2), sqrt(0.3), 0.3, 0.4, sqrt(0.4);
	model_rpsprbot->setQ(q);
	EXPECT_EQ(q, model_rpsprbot->q());
}

TEST_F(Sai2ModelTest, SetAndGetDq) {
	// invalid size
	VectorXd dq = VectorXd::Zero(model_rrpbot->dof() + 1);
	EXPECT_THROW(model_rrpbot->setDq(dq), std::invalid_argument);

	// valid dq
	dq.setZero(model_rrpbot->dof());
	dq << 1.0, 2.0, 3.0;
	model_rrpbot->setDq(dq);
	EXPECT_EQ(dq, model_rrpbot->dq());
}

TEST_F(Sai2ModelTest, SetAndGetDdq) {
	// invalid size
	VectorXd ddq = VectorXd::Zero(model_rrpbot->dof() + 1);
	EXPECT_THROW(model_rrpbot->setDdq(ddq), std::invalid_argument);

	// valid ddq
	ddq.setZero(model_rrpbot->dof());
	ddq << 1.0, 2.0, 3.0;
	model_rrpbot->setDdq(ddq);
	EXPECT_EQ(ddq, model_rrpbot->ddq());
}

TEST_F(Sai2ModelTest, SphericalQuat) {
	std::string valid_sph_joint = "j2";
	std::string invalid_sph_joint = "j1";
	Quaterniond quat(1.0, 0.0, 1.0, 0.0);

	// if joint is not spherical or not existing throw error
	EXPECT_THROW(model_rpsprbot->sphericalQuat(invalid_sph_joint),
				 std::invalid_argument);
	EXPECT_THROW(model_rpsprbot->setSphericalQuat(invalid_sph_joint, quat),
				 std::invalid_argument);

	// valid spherical joint
	model_rpsprbot->setSphericalQuat(valid_sph_joint, quat);
	EXPECT_EQ(quat, model_rpsprbot->sphericalQuat(valid_sph_joint));
}

TEST_F(Sai2ModelTest, WorldGravity) {
	Vector3d expected_world_gravity = Vector3d(0, 0, -9.81);
	EXPECT_TRUE(checkEigenMatricesEqual(expected_world_gravity,
										model_rrpbot->worldGravity()));

	expected_world_gravity << 1.0, 2.0, 3.0;
	model_rrpbot->setWorldGravity(expected_world_gravity);
	EXPECT_TRUE(checkEigenMatricesEqual(expected_world_gravity,
										model_rrpbot->worldGravity()));
}

TEST_F(Sai2ModelTest, TRobotBase) {
	Affine3d expected_T_base = Affine3d::Identity();
	EXPECT_TRUE(checkEigenMatricesEqual(expected_T_base.linear(),
										model_rrpbot->TRobotBase().linear()));
	EXPECT_TRUE(
		checkEigenMatricesEqual(expected_T_base.translation(),
								model_rrpbot->TRobotBase().translation()));

	expected_T_base.linear() =
		AngleAxisd(M_PI / 3, Vector3d::UnitY()).toRotationMatrix();
	expected_T_base.translation() = Vector3d(0.1, 0.2, 0.3);
	model_rrpbot->setTRobotBase(expected_T_base);
	EXPECT_TRUE(checkEigenMatricesEqual(expected_T_base.linear(),
										model_rrpbot->TRobotBase().linear()));
	EXPECT_TRUE(
		checkEigenMatricesEqual(expected_T_base.translation(),
								model_rrpbot->TRobotBase().translation()));
}

TEST_F(Sai2ModelTest, IsLinkInRobot) {
	EXPECT_TRUE(model_rrpbot->isLinkInRobot("link0"));
	EXPECT_TRUE(model_rrpbot->isLinkInRobot("link1"));
	EXPECT_TRUE(model_rrpbot->isLinkInRobot("link2"));
	EXPECT_FALSE(model_rrpbot->isLinkInRobot("link3"));
}

TEST_F(Sai2ModelTest, SphericalJointsDescriptions) {
	EXPECT_TRUE(model_rrpbot->sphericalJoints().empty());
	EXPECT_FALSE(model_rpsprbot->sphericalJoints().empty());
	EXPECT_EQ(model_rpsprbot->sphericalJoints().size(), 1);

	EXPECT_EQ(model_rpsprbot->sphericalJoints().at(0).name, "j2");
	EXPECT_EQ(model_rpsprbot->sphericalJoints().at(0).index, 2);
	EXPECT_EQ(model_rpsprbot->sphericalJoints().at(0).w_index, 7);
}

TEST_F(Sai2ModelTest, JointName) {
	EXPECT_THROW(model_rrpbot->jointName(model_rrpbot->qSize()),
				 invalid_argument);
	std::vector<std::string> expected_joint_names = {"j0", "j1", "j2"};
	for (int i = 0; i < expected_joint_names.size(); ++i) {
		EXPECT_EQ(model_rrpbot->jointName(i), expected_joint_names[i]);
	}
	EXPECT_EQ(model_rpsprbot->jointName(2), "j2");
	EXPECT_EQ(model_rpsprbot->jointName(3), "j2");
	EXPECT_EQ(model_rpsprbot->jointName(4), "j2");
	EXPECT_EQ(model_rpsprbot->jointName(7), "j2");
}

TEST_F(Sai2ModelTest, JointNames) {
	const auto joint_names = model_rrpbot->jointNames();
	EXPECT_EQ(joint_names.size(), model_rrpbot->dof());
	EXPECT_EQ(model_rpsprbot->jointNames().size(), model_rpsprbot->dof() - 2);
	std::vector<std::string> expected_joint_names = {"j0", "j1", "j2"};
	for (int i = 0; i < joint_names.size(); ++i) {
		EXPECT_EQ(joint_names[i], expected_joint_names[i]);
	}
}

TEST_F(Sai2ModelTest, JointLimits) {
	const auto joint_limits = model_rrpbot->jointLimits();
	EXPECT_EQ(joint_limits.size(), model_rrpbot->qSize());
	EXPECT_EQ(model_rpsprbot->jointLimits().size(),
			  model_rpsprbot->qSize() - 4);
	std::vector<JointLimit> expected_joint_limits = {
		JointLimit("j0", 0, -1.5707, 1.5707, 1.5709, 150),
		JointLimit("j1", 1, -2.5707, 2.5707, 1.5709, 250),
		JointLimit("j2", 2, 0, 2, 1, 200),
	};
	for (int i = 0; i < joint_limits.size(); ++i) {
		EXPECT_EQ(joint_limits[i].joint_name,
				  expected_joint_limits[i].joint_name);
		EXPECT_EQ(joint_limits[i].joint_index,
				  expected_joint_limits[i].joint_index);
		EXPECT_EQ(joint_limits[i].position_lower,
				  expected_joint_limits[i].position_lower);
		EXPECT_EQ(joint_limits[i].position_upper,
				  expected_joint_limits[i].position_upper);
		EXPECT_EQ(joint_limits[i].velocity, expected_joint_limits[i].velocity);
		EXPECT_EQ(joint_limits[i].effort, expected_joint_limits[i].effort);
	}
}

TEST_F(Sai2ModelTest, JointIndex) {
	EXPECT_THROW(model_rrpbot->jointIndex("j3"), invalid_argument);
	for (int i = 0; i < model_rrpbot->qSize(); ++i) {
		EXPECT_EQ(model_rrpbot->jointIndex(model_rrpbot->jointName(i)), i);
	}
	// spherical joint
	EXPECT_EQ(model_rpsprbot->jointIndex("j2"), 2);
}

TEST_F(Sai2ModelTest, SphericalJointIndexW) {
	EXPECT_THROW(model_rpsprbot->sphericalJointIndexW("non_existing_joint"),
				 invalid_argument);
	EXPECT_THROW(model_rpsprbot->sphericalJointIndexW("j0"), invalid_argument);
	EXPECT_EQ(model_rpsprbot->sphericalJointIndexW("j2"), 7);
}

TEST_F(Sai2ModelTest, UpdateKinematics) {
	std::string eef_link = "link2";
	Vector3d pos_in_link = Vector3d::Zero();
	// check the position of a link and the jacobian and the mass matrix
	int dof = model_rrpbot->dof();
	VectorXd q = model_rrpbot->q();
	MatrixXd Jv = MatrixXd::Zero(3, dof);
	model_rrpbot->Jv(Jv, eef_link, pos_in_link);
	Vector3d eef_pos = Vector3d::Zero();
	model_rrpbot->position(eef_pos, eef_link, pos_in_link);

	MatrixXd expected_Jv = MatrixXd::Zero(3, dof);
	expected_Jv << 0, 0, 0, -2, -1, 0, 0, 0, 1;
	Vector3d expected_eef_pos = Vector3d(0, 0, 2);
	MatrixXd expected_M =
		MatrixXd::Identity(model_rrpbot->dof(), model_rrpbot->dof());
	expected_M << 5.3, 2.2, 0, 2.2, 1.2, 0, 0, 0, 1;
	MatrixXd expected_M_inv = expected_M.inverse();
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M, model_rrpbot->M()));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M_inv, model_rrpbot->MInv()));

	// Set a new configuration
	q << 0.1, 0.2, 0.3;
	model_rrpbot->setQ(q);
	model_rrpbot->Jv(Jv, eef_link, pos_in_link);
	model_rrpbot->position(eef_pos, eef_link, pos_in_link);
	// nothing changes until we call update kinematics
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M, model_rrpbot->M()));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M_inv, model_rrpbot->MInv()));

	// Update the kinematics
	model_rrpbot->updateKinematics();
	model_rrpbot->Jv(Jv, eef_link, pos_in_link);
	model_rrpbot->position(eef_pos, eef_link, pos_in_link);
	// now the positions and jacobians are updated
	expected_Jv << 0, 0, 0, -2.23694, -1.24194, -0.29552, -0.48401, -0.384176,
		0.955336;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	expected_eef_pos << 0, -0.48401, 2.23694;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));
	// but not the mass matrix
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M, model_rrpbot->M()));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M_inv, model_rrpbot->MInv()));
}

TEST_F(Sai2ModelTest, UpdateModel) {
	std::string eef_link = "link2";
	Vector3d pos_in_link = Vector3d::Zero();
	// check the position of a link and the jacobian and the mass matrix
	int dof = model_rrpbot->dof();
	VectorXd q = model_rrpbot->q();
	MatrixXd Jv = MatrixXd::Zero(3, dof);
	model_rrpbot->Jv(Jv, eef_link, pos_in_link);
	Vector3d eef_pos = Vector3d::Zero();
	model_rrpbot->position(eef_pos, eef_link, pos_in_link);

	MatrixXd expected_Jv = MatrixXd::Zero(3, dof);
	expected_Jv << 0, 0, 0, -2, -1, 0, 0, 0, 1;
	Vector3d expected_eef_pos = Vector3d(0, 0, 2);
	MatrixXd expected_M =
		MatrixXd::Identity(model_rrpbot->dof(), model_rrpbot->dof());
	expected_M << 5.3, 2.2, 0, 2.2, 1.2, 0, 0, 0, 1;
	MatrixXd expected_M_inv = expected_M.inverse();
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M, model_rrpbot->M()));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M_inv, model_rrpbot->MInv()));

	// Set a new configuration
	q << 0.1, 0.2, 0.3;
	model_rrpbot->setQ(q);
	model_rrpbot->Jv(Jv, eef_link, pos_in_link);
	model_rrpbot->position(eef_pos, eef_link, pos_in_link);
	// nothing changes until we call update model
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M, model_rrpbot->M()));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M_inv, model_rrpbot->MInv()));

	// Update the model (kinematics + mass matrix)
	model_rrpbot->updateModel();
	model_rrpbot->Jv(Jv, eef_link, pos_in_link);
	model_rrpbot->position(eef_pos, eef_link, pos_in_link);
	// now the positions, jacobians and mass matrix are updated
	expected_Jv << 0, 0, 0, -2.23694, -1.24194, -0.29552, -0.48401, -0.384176,
		0.955336;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	expected_eef_pos << 0, -0.48401, 2.23694;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));
	expected_M << 6.53817, 3.16409, 0.198669, 3.16409, 1.89, 0, 0.198669, 0,
		1.0;
	expected_M_inv = expected_M.inverse();
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M, model_rrpbot->M()));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M_inv, model_rrpbot->MInv()));

	// update model with externally provided mass matrix
	q << 0, 0, 0;
	model_rrpbot->setQ(q);
	// incorrect size
	MatrixXd external_M = MatrixXd::Identity(model_rrpbot->dof()-1, model_rrpbot->dof()-1);
	EXPECT_THROW(model_rrpbot->updateModel(external_M), invalid_argument);
	// non symmetric
	external_M.setIdentity(model_rrpbot->dof(), model_rrpbot->dof());
	external_M(0,1) = 1.0;
	EXPECT_THROW(model_rrpbot->updateModel(external_M), invalid_argument);
	// non positive definite
	external_M.setIdentity(model_rrpbot->dof(), model_rrpbot->dof());
	external_M(0,0) = -1.0;
	EXPECT_THROW(model_rrpbot->updateModel(external_M), invalid_argument);
	// valid external M
	external_M.setIdentity(model_rrpbot->dof(), model_rrpbot->dof());
	external_M(0,0) = 3.5;
	external_M(0,1) = 1.5;
	external_M(1,0) = 1.5;
	MatrixXd external_M_inv = external_M.inverse();
	model_rrpbot->updateModel(external_M);
	model_rrpbot->Jv(Jv, eef_link, pos_in_link);
	model_rrpbot->position(eef_pos, eef_link, pos_in_link);
	expected_Jv << 0, 0, 0, -2, -1, 0, 0, 0, 1;
	expected_eef_pos = Vector3d(0, 0, 2);
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	EXPECT_TRUE(checkEigenMatricesEqual(external_M, model_rrpbot->M()));
	EXPECT_TRUE(checkEigenMatricesEqual(external_M_inv, model_rrpbot->MInv()));

}

// TODO
TEST_F(Sai2ModelTest, IK) {}

TEST_F(Sai2ModelTest, JointGravity) {
	VectorXd joint_gravity;
	model_rrpbot->jointGravityVector(joint_gravity);
	VectorXd expected_gravity = VectorXd::Zero(model_rrpbot->dof());
	expected_gravity << 0, 0, 9.81;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_gravity, joint_gravity));

	// change joint positions
	model_rrpbot->setQ(Vector3d(0.1, 0.2, 0.3));
	model_rrpbot->updateKinematics();
	model_rrpbot->jointGravityVector(joint_gravity);
	expected_gravity << -5.7275, -3.76877, 9.37185;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_gravity, joint_gravity));

	// change gravity magnitude and direction
	model_rrpbot->setWorldGravity(Vector3d(1.0, -2.0, 3.0));
	model_rrpbot->jointGravityVector(joint_gravity);
	expected_gravity << -4.71236, -1.33135, -3.45705;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_gravity, joint_gravity));
}

TEST_F(Sai2ModelTest, Coriolis) {}
TEST_F(Sai2ModelTest, CoriolisPlusGravity) {}
TEST_F(Sai2ModelTest, factorizedChristoffelMatrix) {}

TEST_F(Sai2ModelTest, Jacobian) {}
TEST_F(Sai2ModelTest, Jv) {}
TEST_F(Sai2ModelTest, Jw) {}

TEST_F(Sai2ModelTest, ComputePosition) {}
TEST_F(Sai2ModelTest, ComputeVelocity) {}
TEST_F(Sai2ModelTest, ComputeAcceleration) {}

TEST_F(Sai2ModelTest, ComputeOrientation) {}
TEST_F(Sai2ModelTest, ComputeAngularVelocity) {}
TEST_F(Sai2ModelTest, ComputeAngularAcceleration) {}

TEST_F(Sai2ModelTest, ComputeTransform) {}

TEST_F(Sai2ModelTest, Velocity6d) {}
TEST_F(Sai2ModelTest, Acceleration6d) {}

TEST_F(Sai2ModelTest, LinkMass) {}
TEST_F(Sai2ModelTest, ComPosition) {}
TEST_F(Sai2ModelTest, ComJacobian) {}

TEST_F(Sai2ModelTest, MatrixRange) {}

TEST_F(Sai2ModelTest, TaskInertia) {}
TEST_F(Sai2ModelTest, TaskInertiaPseudoInv) {}
TEST_F(Sai2ModelTest, DynConsistentJacobian) {}
TEST_F(Sai2ModelTest, Nullspace) {}
TEST_F(Sai2ModelTest, OpSpaceMatrices) {}

TEST_F(Sai2ModelTest, OrientationError) {}
TEST_F(Sai2ModelTest, CrossProductOperator) {}
TEST_F(Sai2ModelTest, GraspMatrixAtGeometricCenter) {}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

}  // namespace Sai2Model
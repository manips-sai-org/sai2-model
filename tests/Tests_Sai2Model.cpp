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
			if (fabs(expected(i, j) - actual(i, j) > epsilon)) {
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
	// Check if the specified link is part of the robot
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

TEST_F(Sai2ModelTest, JointNames) {}
TEST_F(Sai2ModelTest, JointLimits) {}
TEST_F(Sai2ModelTest, JointIndex) {}
TEST_F(Sai2ModelTest, SphericalJointIndexW) {}
TEST_F(Sai2ModelTest, JointName) {}

TEST_F(Sai2ModelTest, UpdateKinematics) {
	std::string eef_link = "link2";
	Vector3d pos_in_link = Vector3d::Zero();
	// check the position of a link and the jacobian
	int dof = model_rrpbot->dof();
	VectorXd q = model_rrpbot->q();
	MatrixXd Jv = MatrixXd::Zero(3, dof);
	model_rrpbot->Jv(Jv, eef_link, pos_in_link);
	Vector3d eef_pos = Vector3d::Zero();
	model_rrpbot->position(eef_pos, eef_link, pos_in_link);

	MatrixXd expected_Jv = MatrixXd::Zero(3, dof);
	expected_Jv << 0, 1, 0, -1, 0, 0, 0, 0, 1;
	Vector3d expected_eef_pos = Vector3d(0, 0, 1);
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));

	// Set a new configuration
	q << 0.1, 0.2, 0.3;
	model_rrpbot->setQ(q);
	model_rrpbot->Jv(Jv, eef_link, pos_in_link);
	model_rrpbot->position(eef_pos, eef_link, pos_in_link);
	// nothing changes until we call update kinematics
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));

	// Update the kinematics
	model_rrpbot->updateKinematics();
	model_rrpbot->Jv(Jv, eef_link, pos_in_link);
	model_rrpbot->position(eef_pos, eef_link, pos_in_link);
	// now the positions and jacobians are updated
	expected_Jv << 0, 1.27409, 0.198669, -1.26772, 0.025784, -0.0978434,
		-0.127196, -0.25698, 0.97517;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	expected_eef_pos << 0.25827013003357957, -0.12719641350943242,
		1.2677214253623608;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));
}

TEST_F(Sai2ModelTest, UpdateModel) {}

TEST_F(Sai2ModelTest, IK) {}

TEST_F(Sai2ModelTest, JointGravity) {}
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
TEST_F(Sai2ModelTest, crossProductOperator) {}
TEST_F(Sai2ModelTest, GraspMatrixAtGeometricCenter) {}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

}  // namespace Sai2Model
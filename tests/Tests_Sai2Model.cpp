#include <Sai2Model.h>
#include <gtest/gtest.h>

namespace Sai2Model {

using namespace Eigen;

const std::string rr_urdf = "./urdf/rrbot.urdf";
const std::string rrp_urdf = "./urdf/rrpbot.urdf";
const std::string rpspr_urdf = "./urdf/rpsprbot.urdf";

class Sai2ModelTest : public ::testing::Test {
protected:
	void SetUp() override {
		model_rrbot = new Sai2Model(rr_urdf);
		model_rrpbot = new Sai2Model(rrp_urdf);
		model_rpsprbot = new Sai2Model(rpspr_urdf);
	}

	void TearDown() override {
		delete model_rrbot;
		delete model_rrpbot;
		delete model_rpsprbot;
	}

	void MoveModelsBaseFrame() {
		Affine3d new_T_robot_base = Affine3d::Identity();
		new_T_robot_base.translation() = Vector3d(0.5, -0.3, 1.2);
		new_T_robot_base.linear() =
			AngleAxisd(M_PI / 2, Vector3d::UnitX()).toRotationMatrix();
		model_rrpbot->setTRobotBase(new_T_robot_base);
		model_rpsprbot->setTRobotBase(new_T_robot_base);
	}

	void SetNewQSphericalModel() {
		VectorXd q = model_rpsprbot->q();
		q(0) = 0.2;
		q(1) = 0.5;
		q(5) = -0.2;
		q(6) = 0.4;
		model_rpsprbot->setQ(q);
		model_rpsprbot->setSphericalQuat(
			"j2", Eigen::Quaterniond(AngleAxisd(
					  M_PI / 4,
					  Vector3d(1.0 / sqrt(3), 1.0 / sqrt(3), 1.0 / sqrt(3)))));
	}

	Sai2Model* model_rrbot;
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
						  << "): "
						  << "Expected: " << expected(i, j) << " "
						  << "Actual:   " << actual(i, j) << std::endl;
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
	MatrixXd Jv = model_rrpbot->Jv(eef_link, pos_in_link);
	Vector3d eef_pos = model_rrpbot->position(eef_link, pos_in_link);

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
	Jv = model_rrpbot->Jv(eef_link, pos_in_link);
	eef_pos = model_rrpbot->position(eef_link, pos_in_link);
	// nothing changes until we call update kinematics
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M, model_rrpbot->M()));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M_inv, model_rrpbot->MInv()));

	// Update the kinematics
	model_rrpbot->updateKinematics();
	Jv = model_rrpbot->Jv(eef_link, pos_in_link);
	eef_pos = model_rrpbot->position(eef_link, pos_in_link);
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
	MatrixXd Jv = model_rrpbot->Jv(eef_link, pos_in_link);
	Vector3d eef_pos = model_rrpbot->position(eef_link, pos_in_link);

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
	Jv = model_rrpbot->Jv(eef_link, pos_in_link);
	eef_pos = model_rrpbot->position(eef_link, pos_in_link);
	// nothing changes until we call update model
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jv, Jv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_eef_pos, eef_pos));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M, model_rrpbot->M()));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_M_inv, model_rrpbot->MInv()));

	// Update the model (kinematics + mass matrix)
	model_rrpbot->updateModel();
	Jv = model_rrpbot->Jv(eef_link, pos_in_link);
	eef_pos = model_rrpbot->position(eef_link, pos_in_link);
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
	MatrixXd external_M =
		MatrixXd::Identity(model_rrpbot->dof() - 1, model_rrpbot->dof() - 1);
	EXPECT_THROW(model_rrpbot->updateModel(external_M), invalid_argument);
	// non symmetric
	external_M.setIdentity(model_rrpbot->dof(), model_rrpbot->dof());
	external_M(0, 1) = 1.0;
	EXPECT_THROW(model_rrpbot->updateModel(external_M), invalid_argument);
	// non positive definite
	external_M.setIdentity(model_rrpbot->dof(), model_rrpbot->dof());
	external_M(0, 0) = -1.0;
	EXPECT_THROW(model_rrpbot->updateModel(external_M), invalid_argument);
	// valid external M
	external_M.setIdentity(model_rrpbot->dof(), model_rrpbot->dof());
	external_M(0, 0) = 3.5;
	external_M(0, 1) = 1.5;
	external_M(1, 0) = 1.5;
	MatrixXd external_M_inv = external_M.inverse();
	model_rrpbot->updateModel(external_M);
	Jv = model_rrpbot->Jv(eef_link, pos_in_link);
	eef_pos = model_rrpbot->position(eef_link, pos_in_link);
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
	VectorXd joint_gravity = model_rrpbot->jointGravityVector();
	VectorXd expected_gravity = VectorXd::Zero(model_rrpbot->dof());
	expected_gravity << 0, 0, 9.81;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_gravity, joint_gravity));

	// change robot base transform
	MoveModelsBaseFrame();
	joint_gravity = model_rrpbot->jointGravityVector();
	expected_gravity << -29.43, -9.81, 0;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_gravity, joint_gravity));

	// change gravity magnitude and direction
	model_rrpbot->setWorldGravity(Vector3d(1.0, -2.0, 3.0));
	EXPECT_TRUE(checkEigenMatricesEqual(Vector3d(1.0, -2.0, 3.0),
										model_rrpbot->worldGravity()));
	joint_gravity = model_rrpbot->jointGravityVector();
	expected_gravity << 9, 3, -2;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_gravity, joint_gravity));

	// change joint positions
	model_rrpbot->setQ(Vector3d(0.1, 0.2, 0.3));
	model_rrpbot->updateKinematics();
	joint_gravity = model_rrpbot->jointGravityVector();
	expected_gravity << 10.8635, 4.49416, -1.02411;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_gravity, joint_gravity));
}

TEST_F(Sai2ModelTest, Coriolis) {
	// this is zero when joint velocities are zero
	VectorXd coriolis = model_rrpbot->coriolisForce();
	VectorXd expected_coriolis = VectorXd::Zero(model_rrpbot->dof());
	EXPECT_TRUE(checkEigenMatricesEqual(expected_coriolis, coriolis));

	// set nonzero joint velocities
	VectorXd dq = VectorXd::Zero(model_rrpbot->dof());
	dq << 1, 2, 3;
	model_rrpbot->setDq(dq);
	model_rrpbot->updateKinematics();
	coriolis = model_rrpbot->coriolisForce();
	expected_coriolis << 36, 18, -10;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_coriolis, coriolis));

	// change robot configuration
	VectorXd q = VectorXd::Zero(model_rrpbot->dof());
	q << 0.1, 0.2, 0.3;
	model_rrpbot->setQ(q);
	model_rrpbot->updateKinematics();
	coriolis = model_rrpbot->coriolisForce();
	expected_coriolis << 38.975, 23.6583, -12.6801;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_coriolis, coriolis));
}

TEST_F(Sai2ModelTest, CoriolisPlusGravity) {
	VectorXd coriolis = VectorXd::Zero(model_rrpbot->dof());
	VectorXd gravity = VectorXd::Zero(model_rrpbot->dof());
	VectorXd coriolis_plus_gravity = VectorXd::Zero(model_rrpbot->dof());
	for (int i = 0; i < 10; i++) {
		Vector3d world_gravity = Vector3d::Random();
		VectorXd q = VectorXd::Random(model_rrpbot->qSize());
		VectorXd dq = VectorXd::Random(model_rrpbot->dof());
		model_rrpbot->setQ(q);
		model_rrpbot->setDq(dq);
		model_rrpbot->setWorldGravity(world_gravity);
		model_rrpbot->updateModel();
		coriolis = model_rrpbot->coriolisForce();
		gravity = model_rrpbot->jointGravityVector();
		coriolis_plus_gravity = model_rrpbot->coriolisPlusGravity();
		EXPECT_TRUE(
			checkEigenMatricesEqual(coriolis + gravity, coriolis_plus_gravity));
	}
}

// TODO: looks like this function does not work as intended
// TEST_F(Sai2ModelTest, factorizedChristoffelMatrix) {
// 	VectorXd coriolis = VectorXd::Zero(model_rrpbot->dof());
// 	MatrixXd C = MatrixXd::Zero(model_rrpbot->dof(), model_rrpbot->dof());
// 	for (int i = 0; i < 10; i++) {
// 		VectorXd q = VectorXd::Random(model_rrpbot->qSize());
// 		VectorXd dq = VectorXd::Random(model_rrpbot->dof());
// 		model_rrpbot->setQ(q);
// 		model_rrpbot->setDq(dq);
// 		model_rrpbot->updateModel();
// 		model_rrpbot->coriolisForce(coriolis);
// 		model_rrpbot->factorizedChristoffelMatrix(C);
// 		EXPECT_TRUE(checkEigenMatricesEqual(coriolis, C * model_rrpbot->dq()));
// 	}
// }

TEST_F(Sai2ModelTest, Jv) {
	const int dof = model_rrpbot->dof();
	const std::string link_name = "link2";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	const Matrix3d rot_in_link =
		AngleAxisd(M_PI / 4, Vector3d::UnitY()).toRotationMatrix();
	MoveModelsBaseFrame();
	MatrixXd Jv = MatrixXd::Zero(3, dof);
	MatrixXd expected_J = MatrixXd::Zero(3, dof);

	// initial configuration
	Jv = model_rrpbot->Jv(link_name, pos_in_link);
	expected_J << 0, 0, 0, -2.5, -1.5, 0, 0.2, 0.2, 1;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_J, Jv));

	// other joint configuration
	VectorXd q = VectorXd::Zero(dof);
	q << 0.2, 0.3, 1.1;
	model_rrpbot->setQ(q);
	model_rrpbot->updateKinematics();
	Jv = model_rrpbot->Jv(link_name, pos_in_link);
	expected_J << 0, 0, 0, -3.35767, -2.3776, -0.479426, -1.26966, -1.07099,
		0.877583;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_J, Jv));

	// jacobian in world frame and in local frame
	Jv = model_rrpbot->JvWorldFrame(link_name, pos_in_link);
	expected_J << 0, 0, 0, 1.26966, 1.07099, -0.877583, -3.35767, -2.3776,
		-0.479426;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_J, Jv));

	Jv = model_rrpbot->JvLocalFrame(link_name, pos_in_link, rot_in_link);
	expected_J << -0.350386, -0.141421, -0.707107, -3.55534, -2.6, 0, 0.350386,
		0.141421, 0.707107;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_J, Jv));
}

TEST_F(Sai2ModelTest, Jw) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	const Matrix3d rot_in_link =
		AngleAxisd(M_PI / 4, Vector3d::UnitY()).toRotationMatrix();
	MoveModelsBaseFrame();
	MatrixXd Jw = MatrixXd::Zero(3, dof);
	MatrixXd expected_J = MatrixXd::Zero(3, dof);

	// initial configuration
	Jw = model_rpsprbot->Jw(link_name);
	expected_J << 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_J, Jw));

	// other joint configuration
	SetNewQSphericalModel();
	model_rpsprbot->updateKinematics();
	Jw = model_rpsprbot->Jw(link_name);
	expected_J << 1, 0, 0.804738, -0.310617, 0.505879, 0, -0.310617, 0, 0,
		0.557506, 0.688194, -0.464302, 0, 0.688194, 0, 0, -0.203923, 0.655672,
		0.726987, 0, 0.655672;

	EXPECT_TRUE(checkEigenMatricesEqual(expected_J, Jw));

	// jacobian in world frame and in local frame
	Jw = model_rpsprbot->JwWorldFrame(link_name);
	expected_J << 1, 0, 0.804738, -0.310617, 0.505879, 0, -0.310617, 0, 0,
		0.203923, -0.655672, -0.726987, 0, -0.655672, 0, 0, 0.557506, 0.688194,
		-0.464302, 0, 0.688194;

	EXPECT_TRUE(checkEigenMatricesEqual(expected_J, Jw));

	Jw = model_rpsprbot->JwLocalFrame(link_name, rot_in_link);
	expected_J << -0.166249, 0, 0.375928, 0, -0.926649, 0, 0, -0.310617, 0, 0,
		1, 0, 0, 1, 0.935884, 0, 0.926649, 0, 0.375928, 0, 0;

	EXPECT_TRUE(checkEigenMatricesEqual(expected_J, Jw));
}

TEST_F(Sai2ModelTest, Jacobian) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	const Matrix3d rot_in_link =
		AngleAxisd(M_PI / 4, Vector3d::UnitY()).toRotationMatrix();
	MoveModelsBaseFrame();
	MatrixXd J = MatrixXd::Zero(6, dof);
	MatrixXd Jv = MatrixXd::Zero(3, dof);
	MatrixXd Jw = MatrixXd::Zero(3, dof);
	for (int i = 0; i < 10; i++) {
		VectorXd q = VectorXd::Random(model_rpsprbot->qSize());
		q(7) = sqrt(1 - q(2) * q(2) - q(3) * q(3) - q(4) * q(4));
		model_rpsprbot->setQ(q);
		model_rpsprbot->updateKinematics();

		// jacobians in robot base frame
		Jv = model_rpsprbot->Jv(link_name, pos_in_link);
		Jw = model_rpsprbot->Jw(link_name);
		J = model_rpsprbot->J(link_name, pos_in_link);
		EXPECT_TRUE(checkEigenMatricesEqual(Jv, J.block(0, 0, 3, dof)));
		EXPECT_TRUE(checkEigenMatricesEqual(Jw, J.block(3, 0, 3, dof)));

		// jacobians in world frame
		Jv = model_rpsprbot->JvWorldFrame(link_name, pos_in_link);
		Jw = model_rpsprbot->JwWorldFrame(link_name);
		J = model_rpsprbot->JWorldFrame(link_name, pos_in_link);
		EXPECT_TRUE(checkEigenMatricesEqual(Jv, J.block(0, 0, 3, dof)));
		EXPECT_TRUE(checkEigenMatricesEqual(Jw, J.block(3, 0, 3, dof)));

		// jacobians in local frame
		Jv = model_rpsprbot->JvLocalFrame(link_name, pos_in_link, rot_in_link);
		Jw = model_rpsprbot->JwLocalFrame(link_name, rot_in_link);
		J = model_rpsprbot->JLocalFrame(link_name, pos_in_link, rot_in_link);
		EXPECT_TRUE(checkEigenMatricesEqual(Jv, J.block(0, 0, 3, dof)));
		EXPECT_TRUE(checkEigenMatricesEqual(Jw, J.block(3, 0, 3, dof)));
	}
}

TEST_F(Sai2ModelTest, ComputePosition) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	MoveModelsBaseFrame();

	Vector3d pos = Vector3d::Zero();
	Vector3d expected_pos(0.1, 0.2, 2.5);
	pos = model_rpsprbot->position(link_name, pos_in_link);
	EXPECT_TRUE(checkEigenMatricesEqual(expected_pos, pos));

	// change joint config
	SetNewQSphericalModel();
	model_rpsprbot->updateKinematics();
	pos = model_rpsprbot->position(link_name, pos_in_link);
	expected_pos << 0.786664, -0.56765, 2.43082;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_pos, pos));

	pos = model_rpsprbot->positionInWorld(link_name, pos_in_link);
	expected_pos << 1.28666, -2.73082, 0.63235;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_pos, pos));
}

TEST_F(Sai2ModelTest, ComputeVelocity) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	MoveModelsBaseFrame();

	Vector3d vel = Vector3d::Zero();
	Vector3d expected_vel = Vector3d::Zero();
	vel = model_rpsprbot->linearVelocity(link_name, pos_in_link);
	EXPECT_TRUE(checkEigenMatricesEqual(expected_vel, vel));

	// change joint config
	SetNewQSphericalModel();
	VectorXd dq = VectorXd::Zero(model_rpsprbot->dof());
	dq << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
	model_rpsprbot->setDq(dq);
	model_rpsprbot->updateKinematics();
	vel = model_rpsprbot->linearVelocity(link_name, pos_in_link);
	expected_vel << 0.793804, -0.215092, 0.104006;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_vel, vel));
	// compare with J * dq
	MatrixXd Jv = MatrixXd::Zero(3, dof);
	Jv = model_rpsprbot->Jv(link_name, pos_in_link);
	EXPECT_TRUE(checkEigenMatricesEqual(vel, Jv * model_rpsprbot->dq()));

	// compute values in world frame
	vel = model_rpsprbot->linearVelocityInWorld(link_name, pos_in_link);
	expected_vel << 0.793804, -0.104006, -0.215092;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_vel, vel));
	// compare with J * dq
	Jv = model_rpsprbot->JvWorldFrame(link_name, pos_in_link);
	EXPECT_TRUE(checkEigenMatricesEqual(vel, Jv * model_rpsprbot->dq()));
}

TEST_F(Sai2ModelTest, ComputeAcceleration) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	MoveModelsBaseFrame();

	Vector3d acc = Vector3d::Zero();
	Vector3d expected_acc = Vector3d::Zero();
	acc = model_rpsprbot->linearAcceleration(link_name, pos_in_link);
	EXPECT_TRUE(checkEigenMatricesEqual(expected_acc, acc));

	// change joint config
	SetNewQSphericalModel();
	VectorXd dq = VectorXd::Zero(model_rpsprbot->dof());
	VectorXd ddq = VectorXd::Zero(model_rpsprbot->dof());
	dq << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
	ddq << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7;
	model_rpsprbot->setDq(dq);
	model_rpsprbot->setDdq(ddq);
	model_rpsprbot->updateKinematics();
	acc = model_rpsprbot->linearAcceleration(link_name, pos_in_link);
	expected_acc << 8.52875, -1.75075, 0.761736;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_acc, acc));

	// compute values in world frame
	acc = model_rpsprbot->linearAccelerationInWorld(link_name, pos_in_link);
	expected_acc << 8.52875, -0.761736, -1.75075;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_acc, acc));
}

TEST_F(Sai2ModelTest, ComputeOrientation) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Matrix3d rot_in_link =
		AngleAxisd(M_PI / 4, Vector3d::UnitY()).toRotationMatrix();
	MoveModelsBaseFrame();

	Matrix3d rot = Matrix3d::Identity();
	Matrix3d expected_rot = Matrix3d::Identity();
	rot = model_rpsprbot->rotation(link_name, rot_in_link);
	expected_rot << 0.707107, 0, 0.707107, 0, 1, 0, -0.707107, 0, 0.707107;

	EXPECT_TRUE(checkEigenMatricesEqual(expected_rot, rot));

	// change joint config
	SetNewQSphericalModel();
	model_rpsprbot->updateKinematics();
	rot = model_rpsprbot->rotation(link_name, rot_in_link);
	expected_rot << -0.166249, -0.310617, 0.935884, 0.639827, 0.688194,
		0.342068, -0.750322, 0.655672, 0.0843298;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_rot, rot));

	rot = model_rpsprbot->rotationInWorld(link_name, rot_in_link);
	expected_rot << -0.166249, -0.310617, 0.935884, 0.750322, -0.655672,
		-0.0843298, 0.639827, 0.688194, 0.342068;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_rot, rot));
}

TEST_F(Sai2ModelTest, ComputeAngularVelocity) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	MoveModelsBaseFrame();

	Vector3d angvel = Vector3d::Zero();
	Vector3d expected_angvel = Vector3d::Zero();
	angvel = model_rpsprbot->angularVelocity(link_name);
	EXPECT_TRUE(checkEigenMatricesEqual(expected_angvel, angvel));

	// change joint config
	SetNewQSphericalModel();
	VectorXd dq = VectorXd::Zero(model_rpsprbot->dof());
	dq << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
	model_rpsprbot->setDq(dq);
	model_rpsprbot->updateKinematics();
	angvel = model_rpsprbot->angularVelocity(link_name);
	expected_angvel << 0.252682, 0.692114, 1.02356;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_angvel, angvel));
	// compare with J * dq
	MatrixXd Jw = MatrixXd::Zero(3, dof);
	Jw = model_rpsprbot->Jw(link_name);
	EXPECT_TRUE(checkEigenMatricesEqual(angvel, Jw * model_rpsprbot->dq()));

	// compute values in world frame
	angvel = model_rpsprbot->angularVelocityInWorld(link_name);
	expected_angvel << 0.252682, -1.02356, 0.692114;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_angvel, angvel));
	// compare with J * dq
	Jw = model_rpsprbot->JwWorldFrame(link_name);
	EXPECT_TRUE(checkEigenMatricesEqual(angvel, Jw * model_rpsprbot->dq()));
}

TEST_F(Sai2ModelTest, ComputeAngularAcceleration) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	MoveModelsBaseFrame();

	Vector3d angacc = Vector3d::Zero();
	Vector3d expected_angacc = Vector3d::Zero();
	angacc = model_rpsprbot->angularAcceleration(link_name);
	EXPECT_TRUE(checkEigenMatricesEqual(expected_angacc, angacc));

	// change joint config
	SetNewQSphericalModel();
	VectorXd dq = VectorXd::Zero(model_rpsprbot->dof());
	VectorXd ddq = VectorXd::Zero(model_rpsprbot->dof());
	dq << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
	ddq << 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7;
	model_rpsprbot->setDq(dq);
	model_rpsprbot->setDdq(ddq);
	model_rpsprbot->updateKinematics();
	angacc = model_rpsprbot->angularAcceleration(link_name);
	expected_angacc << 2.60408, 7.21827, 11.5524;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_angacc, angacc));

	// compute values in world frame
	angacc = model_rpsprbot->angularAccelerationInWorld(link_name);
	expected_angacc << 2.60408, -11.5524, 7.21827;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_angacc, angacc));
}

TEST_F(Sai2ModelTest, ComputeTransform) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	const Matrix3d rot_in_link =
		AngleAxisd(M_PI / 4, Vector3d::UnitY()).toRotationMatrix();
	MoveModelsBaseFrame();
	Affine3d T = Affine3d::Identity();
	Vector3d pos = Vector3d::Zero();
	Matrix3d ori = Matrix3d::Identity();
	for (int i = 0; i < 10; i++) {
		VectorXd q = VectorXd::Random(model_rpsprbot->qSize());
		q(7) = sqrt(1 - q(2) * q(2) - q(3) * q(3) - q(4) * q(4));
		model_rpsprbot->setQ(q);
		model_rpsprbot->updateKinematics();

		// in robot base frame
		pos = model_rpsprbot->position(link_name, pos_in_link);
		ori = model_rpsprbot->rotation(link_name, rot_in_link);
		T = model_rpsprbot->transform(link_name, pos_in_link, rot_in_link);
		EXPECT_TRUE(checkEigenMatricesEqual(pos, T.translation()));
		EXPECT_TRUE(checkEigenMatricesEqual(ori, T.linear()));

		// in world frame
		pos = model_rpsprbot->positionInWorld(link_name, pos_in_link);
		ori = model_rpsprbot->rotationInWorld(link_name, rot_in_link);
		T = model_rpsprbot->transformInWorld(link_name, pos_in_link,
											 rot_in_link);
		EXPECT_TRUE(checkEigenMatricesEqual(pos, T.translation()));
		EXPECT_TRUE(checkEigenMatricesEqual(ori, T.linear()));
	}
}

TEST_F(Sai2ModelTest, Velocity6d) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	MoveModelsBaseFrame();
	Vector3d linvel = Vector3d::Zero();
	Vector3d angvel = Vector3d::Zero();
	VectorXd vel6d = VectorXd::Zero(6);
	for (int i = 0; i < 10; i++) {
		VectorXd q = VectorXd::Random(model_rpsprbot->qSize());
		q(7) = sqrt(1 - q(2) * q(2) - q(3) * q(3) - q(4) * q(4));
		VectorXd dq = VectorXd::Random(model_rpsprbot->dof());
		model_rpsprbot->setQ(q);
		model_rpsprbot->setDq(dq);
		model_rpsprbot->updateKinematics();

		// in robot base frame
		linvel = model_rpsprbot->linearVelocity(link_name, pos_in_link);
		angvel = model_rpsprbot->angularVelocity(link_name);
		vel6d = model_rpsprbot->velocity6d(link_name, pos_in_link);
		EXPECT_TRUE(checkEigenMatricesEqual(linvel, vel6d.head(3)));
		EXPECT_TRUE(checkEigenMatricesEqual(angvel, vel6d.tail(3)));

		// in world frame
		linvel = model_rpsprbot->linearVelocityInWorld(link_name, pos_in_link);
		angvel = model_rpsprbot->angularVelocityInWorld(link_name);
		vel6d = model_rpsprbot->velocity6dInWorld(link_name, pos_in_link);
		EXPECT_TRUE(checkEigenMatricesEqual(linvel, vel6d.head(3)));
		EXPECT_TRUE(checkEigenMatricesEqual(angvel, vel6d.tail(3)));
	}
}

TEST_F(Sai2ModelTest, Acceleration6d) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	MoveModelsBaseFrame();
	Vector3d linaccel = Vector3d::Zero();
	Vector3d angaccel = Vector3d::Zero();
	VectorXd accel6d = VectorXd::Zero(6);
	for (int i = 0; i < 10; i++) {
		VectorXd q = VectorXd::Random(model_rpsprbot->qSize());
		q(7) = sqrt(1 - q(2) * q(2) - q(3) * q(3) - q(4) * q(4));
		VectorXd dq = VectorXd::Random(model_rpsprbot->dof());
		VectorXd ddq = VectorXd::Random(model_rpsprbot->dof());
		model_rpsprbot->setQ(q);
		model_rpsprbot->setDq(dq);
		model_rpsprbot->setDdq(ddq);
		model_rpsprbot->updateKinematics();

		// in robot base frame
		linaccel = model_rpsprbot->linearAcceleration(link_name, pos_in_link);
		angaccel = model_rpsprbot->angularAcceleration(link_name);
		accel6d = model_rpsprbot->acceleration6d(link_name, pos_in_link);
		EXPECT_TRUE(checkEigenMatricesEqual(linaccel, accel6d.head(3)));
		EXPECT_TRUE(checkEigenMatricesEqual(angaccel, accel6d.tail(3)));

		// in world frame
		linaccel = model_rpsprbot->linearAccelerationInWorld(
			link_name, pos_in_link);
		angaccel =
			model_rpsprbot->angularAccelerationInWorld(link_name);
		accel6d = model_rpsprbot->acceleration6dInWorld(link_name,
														pos_in_link);
		EXPECT_TRUE(checkEigenMatricesEqual(linaccel, accel6d.head(3)));
		EXPECT_TRUE(checkEigenMatricesEqual(angaccel, accel6d.tail(3)));
	}
}

TEST_F(Sai2ModelTest, LinkMass) {
	LinkMassParams mass_params = model_rrbot->getLinkMassParams("link1");
	EXPECT_DOUBLE_EQ(1.5, mass_params.mass);
	EXPECT_TRUE(
		checkEigenMatricesEqual(Vector3d(0.1, 0.2, 0.3), mass_params.com_pos));
	EXPECT_TRUE(checkEigenMatricesEqual(0.1 * Matrix3d::Identity(),
										mass_params.inertia));
}

TEST_F(Sai2ModelTest, ComPosition) {
	Vector3d robot_com, expected_com;
	SetNewQSphericalModel();
	model_rpsprbot->updateKinematics();
	robot_com = model_rpsprbot->comPosition();
	expected_com << 0.161881, -0.38698, 1.40872;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_com, robot_com));
}

TEST_F(Sai2ModelTest, ComJacobian) {
	MatrixXd com_jacobian = MatrixXd::Zero(3, model_rpsprbot->dof());
	MatrixXd expected_com_jacobian = com_jacobian;
	SetNewQSphericalModel();
	model_rpsprbot->updateKinematics();
	com_jacobian = model_rpsprbot->comJacobian();
	expected_com_jacobian << 0, 0, 0.0993975, 0.257516, 0, 0.202352, 0,
		-1.40872, -0.158935, -0.220222, 0.178402, 0, -0.185721, 0, -0.38698,
		0.784053, -0.209815, -0.0652553, 0, 0.290795, 0;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_com_jacobian, com_jacobian));
}

TEST_F(Sai2ModelTest, TaskInertia) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	MatrixXd Lambda, J;
	J.setZero(6, dof);
	Lambda.setZero(6, 6);
	SetNewQSphericalModel();
	model_rpsprbot->updateModel();

	J = model_rpsprbot->J(link_name, pos_in_link);
	Lambda = model_rpsprbot->taskInertiaMatrix(J);
	MatrixXd expected_lambda = MatrixXd::Zero(6, 6);
	expected_lambda << 2.45598, -0.388523, -0.565437, -0.591744, -1.4949,
		0.525617, -0.388523, 3.1234, 0.987834, 2.07603, 1.21213, -1.48404,
		-0.565437, 0.987834, 3.02436, 1.04979, 2.05066, -0.684001, -0.591744,
		2.07603, 1.04979, 2.19218, 1.52106, -1.12447, -1.4949, 1.21213, 2.05066,
		1.52106, 2.4057, -1.01646, 0.525617, -1.48404, -0.684001, -1.12447,
		-1.01646, 1.0047;

	EXPECT_TRUE(checkEigenMatricesEqual(expected_lambda, Lambda));
}

TEST_F(Sai2ModelTest, TaskInertiaPseudoInv) {
	const int dof = model_rrbot->dof();
	const std::string link_name = "link1";
	const Vector3d pos_in_link(0.0, 0.0, 0.3);
	MatrixXd Lambda, Lambda_pseudo_inv;
	Lambda.setZero(2, 2);
	Lambda_pseudo_inv.setZero(2, 2);
	MatrixXd Jv, Jv_2d;
	Jv.setZero(3, dof);
	Jv_2d.setZero(2, dof);

	// default configuration is singular
	Jv = model_rrbot->Jv(link_name, pos_in_link);
	Jv_2d = Jv.block(1, 0, 2, dof);
	Lambda = model_rrbot->taskInertiaMatrix(Jv_2d);
	Lambda_pseudo_inv = model_rrbot->taskInertiaMatrixWithPseudoInv(Jv_2d);
	MatrixXd expected_Lambda_pseudo_inv = MatrixXd::Zero(2, 2);
	expected_Lambda_pseudo_inv << 1.59467, 0, 0, 0;
	EXPECT_TRUE(isnan(Lambda(0, 0)));
	EXPECT_TRUE(
		checkEigenMatricesEqual(expected_Lambda_pseudo_inv, Lambda_pseudo_inv));

	// non singular configuration
	VectorXd q = model_rrbot->q();
	q(1) += 0.5;
	model_rrbot->setQ(q);
	model_rrbot->updateModel();
	Jv = model_rrbot->Jv(link_name, pos_in_link);
	Jv_2d = Jv.block(1, 0, 2, dof);
	Lambda = model_rrbot->taskInertiaMatrix(Jv_2d);
	Lambda_pseudo_inv = model_rrbot->taskInertiaMatrixWithPseudoInv(Jv_2d);

	expected_Lambda_pseudo_inv << 1.6, 0.816951, 0.816951, 5.90864;
	EXPECT_TRUE(
		checkEigenMatricesEqual(expected_Lambda_pseudo_inv, Lambda_pseudo_inv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Lambda_pseudo_inv, Lambda));
}

TEST_F(Sai2ModelTest, DynConsistentJacobian) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	MatrixXd Jbar, J;
	J.setZero(6, dof);
	Jbar.setZero(dof, 6);
	SetNewQSphericalModel();
	model_rpsprbot->updateModel();

	J = model_rpsprbot->J(link_name, pos_in_link);
	Jbar = model_rpsprbot->dynConsistentInverseJacobian(J);
	MatrixXd expected_Jbar = MatrixXd::Zero(dof, 6);
	expected_Jbar << 0.157377, -0.414018, -0.373932, -0.49165, -0.460724,
		0.268614, -0.238489, 0.372479, 0.403845, 0.483299, 0.510137, -0.263651,
		-0.126647, 0.333176, 0.300917, 1.20039, 0.928268, -0.420086, 1.11148,
		0.320198, -0.568935, -0.141244, -0.860789, 0.0720711, -0.0796136,
		0.209443, 0.189164, 0.754595, -0.231231, 0.5911, 0.624475, -0.571149,
		0.576222, -0.409894, -0.10998, 0.315857, -1.0626, -0.448799, 0.452785,
		-0.322088, 1.40587, 0.667037;

	EXPECT_TRUE(checkEigenMatricesEqual(expected_Jbar, Jbar));
}

TEST_F(Sai2ModelTest, Nullspace) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	MatrixXd N, J;
	J.setZero(6, dof);
	N.setZero(dof, dof);
	SetNewQSphericalModel();
	model_rpsprbot->updateModel();

	J = model_rpsprbot->J(link_name, pos_in_link);
	N = model_rpsprbot->nullspaceMatrix(J);
	MatrixXd expected_N = MatrixXd::Zero(dof, dof);
	expected_N << 0.272981, 0.284225, 0, 0.0223339, 0, 0, 0, 0.651376, 0.678205,
		0, 0.0532922, 0, 0, 0, -0.219678, -0.228727, 0, -0.017973, 0, 0, 0,
		0.596632, 0.621207, 0, 0.0488134, 0, 0, 0, -0.138096, -0.143784, 0,
		-0.0112983, 0, 0, 0, -0.651376, -0.678205, 0, -0.0532922, 0, 0, 0,
		-0.51184, -0.532922, 0, -0.0418761, 0, 0, 0;

	EXPECT_TRUE(checkEigenMatricesEqual(expected_N, N));
}

TEST_F(Sai2ModelTest, OpSpaceMatrices) {
	const int dof = model_rpsprbot->dof();
	const std::string link_name = "link4";
	const Vector3d pos_in_link(0.1, 0.2, 0.5);
	MatrixXd Lambda, Jbar, N, J;
	J.setZero(3, dof);
	Jbar.setZero(dof, 3);
	N.setZero(dof, dof);
	Lambda.setZero(3, 3);
	for (int i = 0; i < 10; i++) {
		VectorXd q = VectorXd::Random(model_rpsprbot->qSize());
		q(7) = sqrt(1 - q(2) * q(2) - q(3) * q(3) - q(4) * q(4));
		model_rpsprbot->setQ(q);
		model_rpsprbot->updateModel();

		J = model_rpsprbot->Jv(link_name, pos_in_link);
		Lambda = model_rpsprbot->taskInertiaMatrix(J);
		Jbar = model_rpsprbot->dynConsistentInverseJacobian(J);
		N = model_rpsprbot->nullspaceMatrix(J);
		OpSpaceMatrices op_space_matrices =
			model_rpsprbot->operationalSpaceMatrices(J);

		EXPECT_TRUE(checkEigenMatricesEqual(Lambda, op_space_matrices.Lambda));
		EXPECT_TRUE(checkEigenMatricesEqual(Jbar, op_space_matrices.Jbar));
		EXPECT_TRUE(checkEigenMatricesEqual(N, op_space_matrices.N));
	}
}

TEST_F(Sai2ModelTest, MatrixRange) {
	MatrixXd J = model_rrbot->Jv("link1");
	MatrixXd JRange = matrixRangeBasis(J);
	MatrixXd expected_Range = MatrixXd::Zero(3, 1);
	expected_Range(1, 0) = -1;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Range, JRange));
}

TEST_F(Sai2ModelTest, OrientationError) {
	Vector3d delta_phi = Vector3d::Zero();
	Vector3d expected_delta_phi = Vector3d::Zero();
	Matrix3d rotation1 = Matrix3d::Identity();
	Matrix3d rotation2 = Matrix3d::Identity();

	// incorrect rotation matrix
	rotation1(0, 1) = 0.3;
	EXPECT_THROW(orientationError(rotation1, rotation2),
				 invalid_argument);

	// zero error
	rotation1(0, 1) = 0.0;
	delta_phi = orientationError(rotation1, rotation2);
	EXPECT_TRUE(checkEigenMatricesEqual(expected_delta_phi, delta_phi));
	delta_phi =
		orientationError(Quaterniond(rotation1), Quaterniond(rotation2));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_delta_phi, delta_phi));

	// some error
	rotation1 = AngleAxisd(M_PI / 4, Vector3d::UnitX()).toRotationMatrix() *
				AngleAxisd(M_PI / 6, Vector3d::UnitY()).toRotationMatrix();
	delta_phi = orientationError(rotation1, rotation2);
	expected_delta_phi << -0.65974, -0.426777, -0.176777;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_delta_phi, delta_phi));
	delta_phi =
		orientationError(Quaterniond(rotation1), Quaterniond(rotation2));
	// different error but colinear
	expected_delta_phi << -0.739288, -0.478235, -0.198092;
	EXPECT_TRUE(checkEigenMatricesEqual(expected_delta_phi, delta_phi));
}

TEST_F(Sai2ModelTest, CrossProductOperator) {
	Vector3d v = Vector3d::Random();
	Matrix3d v_cross = Matrix3d::Zero();
	v_cross << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
	EXPECT_TRUE(checkEigenMatricesEqual(v_cross, crossProductOperator(v)));
}

TEST_F(Sai2ModelTest, GraspMatrixAtGeometricCenter) {
	// 2 point contact case
	std::vector<Vector3d> contact_locations = {
		Vector3d(0.1, 0.2, 0.5),
		Vector3d(-1.1, 0.7, -0.5),
	};
	std::vector<ContactType> contact_types = {
		PointContact,
		PointContact,
	};
	MatrixXd expected_G, expected_Ginv;
	Matrix3d expected_R;
	Vector3d expected_geometric_center;
	GraspMatrixData gm_data =
		graspMatrixAtGeometricCenter(contact_locations, contact_types);
	expected_G.setZero(6, 6);
	expected_Ginv.setZero(6, 6);

	expected_G << 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1,
		-0.559017, -0.268328, 0.536656, 0.559017, 0.268328, -0.536656, -0,
		0.733485, 0.366742, 0, -0.733485, -0.366742, 0.365826, -0.152428,
		0.304855, -0.365826, 0.152428, -0.304855;

	expected_Ginv << 0.5, 0, 0, -1.11803, -0, -0.731653, 0, 0.5, 0, -0.536656,
		1.46697, 0.304855, 0, 0, 0.5, 1.07331, 0.733485, -0.609711, 0.5, 0, 0,
		1.11803, 0, -0.731653, 0, 0.5, 0, 0.536656, -1.46697, 0.304855, 0, 0,
		0.5, -1.07331, -0.733485, -0.609711;

	expected_R << -0.731653, 0, -0.681677, 0.304855, -0.894427, -0.327205,
		-0.609711, -0.447214, 0.65441;

	expected_geometric_center << -0.5, 0.45, 0;

	EXPECT_TRUE(checkEigenMatricesEqual(expected_G, gm_data.G));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Ginv, gm_data.G_inv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_R, gm_data.R));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_geometric_center,
										gm_data.resultant_point));

	// 2 surface contact
	contact_types = {
		SurfaceContact,
		SurfaceContact,
	};
	gm_data = graspMatrixAtGeometricCenter(contact_locations, contact_types);
	expected_G.setZero(12, 12);
	expected_Ginv.setZero(12, 12);

	expected_G << 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -0.5, -0.25, 0, 0.5,
		0.25, 1, 0, 0, 1, 0, 0, 0.5, 0, -0.6, -0.5, 0, 0.6, 0, 1, 0, 0, 1, 0,
		0.25, 0.6, 0, -0.25, -0.6, 0, 0, 0, 1, 0, 0, 1, 0.365826, -0.152428,
		0.304855, -0.365826, 0.152428, -0.304855, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0.365826, -0.152428, 0.304855, -0.365826, 0.152428, -0.304855, 0,
		0, 0, 0, 0, 0, 0, -0.894427, -0.447214, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		-0.681677, -0.327205, 0.65441, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		-0.894427, -0.447214, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.681677, -0.327205,
		0.65441;

	expected_Ginv << 0.5, 0, 0, 0, 0.371747, 0.185874, 0.731653, 0, 0.415626, 0,
		0.415626, 0, 0, 0.5, 0, -0.371747, 0, 0.446097, -0.304855, 0, 0.1995,
		-0.545342, 0.1995, -0.545342, 0, 0, 0.5, -0.185874, -0.446097, 0,
		0.609711, 0, -0.399001, -0.272671, -0.399001, -0.272671, 0.5, 0, 0, -0,
		-0.371747, -0.185874, -0.731653, 0, -0.415626, -0, -0.415626, -0, 0,
		0.5, 0, 0.371747, -0, -0.446097, 0.304855, 0, -0.1995, 0.545342,
		-0.1995, 0.545342, 0, 0, 0.5, 0.185874, 0.446097, -0, -0.609711, 0,
		0.399001, 0.272671, 0.399001, 0.272671, 0, 0, 0, 0.267658, -0.111524,
		0.223048, 0, 0.731653, 0, -0.681677, 0, 0, 0, 0, 0, -0.111524,
		0.0464684, -0.0929368, 0, -0.304855, -0.894427, -0.327205, 0, 0, 0, 0,
		0, 0.223048, -0.0929368, 0.185874, 0, 0.609711, -0.447214, 0.65441, 0,
		0, 0, 0, 0, 0.267658, -0.111524, 0.223048, 0, -0.731653, 0, 0, 0,
		-0.681677, 0, 0, 0, -0.111524, 0.0464684, -0.0929368, 0, 0.304855, 0, 0,
		-0.894427, -0.327205, 0, 0, 0, 0.223048, -0.0929368, 0.185874, 0,
		-0.609711, 0, 0, -0.447214, 0.65441;

	EXPECT_TRUE(checkEigenMatricesEqual(expected_G, gm_data.G));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Ginv, gm_data.G_inv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_R, gm_data.R));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_geometric_center,
										gm_data.resultant_point));

	// 3 contacts, one surface
	contact_locations = {
		Vector3d(0.1, 0.2, 0.5),
		Vector3d(-1.1, 0.7, -0.5),
		Vector3d(-0.4, 1.6, 0.0),
	};
	contact_types = {
		PointContact,
		PointContact,
		SurfaceContact,
	};

	gm_data = graspMatrixAtGeometricCenter(contact_locations, contact_types);
	expected_G.setZero(12, 12);
	expected_Ginv.setZero(12, 12);

	expected_G << 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, -0.5, -0.633333, 0, 0.5,
		-0.133333, 0, -0, 0.766667, 1, 0, 0, 0.5, 0, -0.566667, -0.5, 0,
		0.633333, 0, 0, -0.0666667, 0, 1, 0, 0.633333, 0.566667, 0, 0.133333,
		-0.633333, 0, -0.766667, 0.0666667, 0, 0, 0, 1, 0.369247, -0.0101124,
		0.297654, -0.375595, 0.248363, -0.31942, 0.00634781, -0.238251,
		0.0217658, 0, 0, 0, 0.0450953, -0.456166, 0.0681652, 0.169937,
		-0.017152, 0.137862, -0.215032, 0.473318, -0.206027, 0, 0, 0,
		-0.0880912, 0.0792948, -0.0763877, -0.227992, -0.412676, -0.154492,
		0.316083, 0.333381, 0.23088, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		1;

	expected_Ginv << 0.333333, 0, 0, 0.0926461, 0.319119, 0.351428, 0.731653,
		0.318788, 0, -0.0926461, -0.319119, -0.351428, 0, 0.333333, 0,
		-0.231937, -0.00828156, 0.247832, -0.304855, -0.892607, 0, 0.231937,
		0.00828156, -0.247832, 0, 0, 0.333333, -0.398786, -0.372158, -0.0843646,
		0.609711, 0.318788, 0, 0.398786, 0.372158, 0.0843646, 0.333333, 0, 0,
		0.148803, -0.474833, 0.196917, -0.731653, 0, -0.562254, -0.148803,
		0.474833, -0.196917, 0, 0.333333, 0, 0.210941, 0.021822, -0.295514,
		0.304855, 0, -0.722897, -0.210941, -0.021822, 0.295514, 0, 0, 0.333333,
		-0.244735, 0.595636, -0.170625, -0.609711, 0, -0.40161, 0.244735,
		-0.595636, 0.170625, 0.333333, 0, 0, -0.241449, 0.155715, -0.548345, 0,
		-0.318788, 0.562254, 0.241449, -0.155715, 0.548345, 0, 0.333333, 0,
		0.0209956, -0.0135404, 0.0476822, 0, 0.892607, 0.722897, -0.0209956,
		0.0135404, -0.0476822, 0, 0, 0.333333, 0.64352, -0.223478, 0.254989, 0,
		-0.318788, 0.40161, -0.64352, 0.223478, -0.254989, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1;

	expected_R.setIdentity();
	expected_geometric_center << -0.466667, 0.833333, 0;

	EXPECT_TRUE(checkEigenMatricesEqual(expected_G, gm_data.G));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Ginv, gm_data.G_inv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_R, gm_data.R));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_geometric_center,
										gm_data.resultant_point));

	// 4 surface contacts
	contact_locations = {
		Vector3d(0.1, 0.2, 0.5),
		Vector3d(-1.1, 0.7, -0.5),
		Vector3d(-0.4, 1.6, 0.0),
		Vector3d(-0.6, 2.6, 1.0),
	};
	contact_types = {
		SurfaceContact,
		SurfaceContact,
		SurfaceContact,
		SurfaceContact,
	};

	gm_data = graspMatrixAtGeometricCenter(contact_locations, contact_types);
	expected_G.setZero(24, 24);
	expected_Ginv.setZero(24, 24);

	expected_G << 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, -0.25, -1.075, 0, 0.75, -0.575, 0, 0.25, 0.325, 0, -0.75,
		1.325, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0.25, 0, -0.6, -0.75, 0, 0.6,
		-0.25, 0, -0.1, 0.75, 0, 0.1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1.075,
		0.6, 0, 0.575, -0.6, 0, -0.325, 0.1, 0, -1.325, -0.1, 0, 0, 0, 1, 0, 0,
		1, 0, 0, 1, 0, 0, 1, 0.377062, -0.00824457, 0.289692, -0.359027,
		0.268573, -0.328715, -0.0382846, -0.222271, 0.105765, 0.0202501,
		-0.0380577, -0.0667415, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0268039,
		-0.229673, 0.253064, 0.142026, -0.117018, 0.106796, -0.209622, 0.531822,
		-0.568809, 0.0943999, -0.185131, 0.208948, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0.120967, -0.45763, -0.345996, 0.00463782, 0.147107, 0.0959672,
		0.126929, -0.160838, 0.47906, -0.252533, 0.471361, -0.229031, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, -0.100724, -0.00569851, -0.0940131, -0.432,
		-0.214748, 0.198992, 0.893101, 0.330254, -0.147172, -0.360377,
		-0.109807, 0.0421928, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.0153849,
		0.152878, 0.0763381, 0.259484, -0.391266, -0.525577, -0.750072,
		-0.0433575, 0.261567, 0.505973, 0.281746, 0.187672, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, -0.0849719, 0.0662935, 0.131923, -0.244561, 0.0192546,
		0.299911, 0.637213, -0.0269633, -0.851381, -0.30768, -0.0585847,
		0.419547, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

	expected_Ginv << 0.25, 0, 0, 0.0106179, 0.31036, 0.345919, 0.731653,
		0.318788, 0.274563, 0, 0, 0, -0.0106179, -0.31036, -0.345919,
		-0.0106179, -0.31036, -0.345919, -0.0106179, -0.31036, -0.345919,
		-0.0106179, -0.31036, -0.345919, 0, 0.25, 0, -0.0448191, 0.0994754,
		0.167468, -0.304855, -0.892607, -0.941357, 0, 0, 0, 0.0448191,
		-0.0994754, -0.167468, 0.0448191, -0.0994754, -0.167468, 0.0448191,
		-0.0994754, -0.167468, 0.0448191, -0.0994754, -0.167468, 0, 0, 0.25,
		-0.218205, -0.31712, -0.110093, 0.609711, 0.318788, -0.196116, 0, 0, 0,
		0.218205, 0.31712, 0.110093, 0.218205, 0.31712, 0.110093, 0.218205,
		0.31712, 0.110093, 0.218205, 0.31712, 0.110093, 0.25, 0, 0, 0.0366046,
		-0.356477, 0.0513981, -0.731653, 0, 0, -0.562254, -0.202278, 0,
		-0.0366046, 0.356477, -0.0513981, -0.0366046, 0.356477, -0.0513981,
		-0.0366046, 0.356477, -0.0513981, -0.0366046, 0.356477, -0.0513981, 0,
		0.25, 0, 0.156076, -0.116973, -0.158461, 0.304855, 0, 0, -0.722897,
		-0.768658, 0, -0.156076, 0.116973, 0.158461, -0.156076, 0.116973,
		0.158461, -0.156076, 0.116973, 0.158461, -0.156076, 0.116973, 0.158461,
		0, 0, 0.25, -0.148942, 0.37486, 0.080368, -0.609711, 0, 0, -0.40161,
		-0.606835, 0, 0.148942, -0.37486, -0.080368, 0.148942, -0.37486,
		-0.080368, 0.148942, -0.37486, -0.080368, 0.148942, -0.37486, -0.080368,
		0.25, 0, 0, 0.00289362, -0.196951, -0.130954, 0, -0.318788, 0, 0.562254,
		0, 0.140028, -0.00289362, 0.196951, 0.130954, -0.00289362, 0.196951,
		0.130954, -0.00289362, 0.196951, 0.130954, -0.00289362, 0.196951,
		0.130954, 0, 0.25, 0, 0.0574299, 0.00637253, 0.0331659, 0, 0.892607, 0,
		0.722897, 0, -0.70014, -0.0574299, -0.00637253, -0.0331659, -0.0574299,
		-0.00637253, -0.0331659, -0.0574299, -0.00637253, -0.0331659,
		-0.0574299, -0.00637253, -0.0331659, 0, 0, 0.25, 0.0758163, -0.0704963,
		-0.00926614, 0, -0.318788, 0, 0.40161, 0, -0.70014, -0.0758163,
		0.0704963, 0.00926614, -0.0758163, 0.0704963, 0.00926614, -0.0758163,
		0.0704963, 0.00926614, -0.0758163, 0.0704963, 0.00926614, 0.25, 0, 0,
		-0.0501161, 0.243068, -0.266363, 0, 0, -0.274563, 0, 0.202278,
		-0.140028, 0.0501161, -0.243068, 0.266363, 0.0501161, -0.243068,
		0.266363, 0.0501161, -0.243068, 0.266363, 0.0501161, -0.243068,
		0.266363, 0, 0.25, 0, -0.168687, 0.0111247, -0.0421735, 0, 0, 0.941357,
		0, 0.768658, 0.70014, 0.168687, -0.0111247, 0.0421735, 0.168687,
		-0.0111247, 0.0421735, 0.168687, -0.0111247, 0.0421735, 0.168687,
		-0.0111247, 0.0421735, 0, 0, 0.25, 0.291331, 0.0127555, 0.0389915, 0, 0,
		0.196116, 0, 0.606835, 0.70014, -0.291331, -0.0127555, -0.0389915,
		-0.291331, -0.0127555, -0.0389915, -0.291331, -0.0127555, -0.0389915,
		-0.291331, -0.0127555, -0.0389915, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

	expected_geometric_center << -0.5, 1.275, 0.25;

	EXPECT_TRUE(checkEigenMatricesEqual(expected_G, gm_data.G));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_Ginv, gm_data.G_inv));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_R, gm_data.R));
	EXPECT_TRUE(checkEigenMatricesEqual(expected_geometric_center,
										gm_data.resultant_point));
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

}  // namespace Sai2Model
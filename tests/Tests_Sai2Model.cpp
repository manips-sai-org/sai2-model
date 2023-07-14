#include <Sai2Model.h>
#include <gtest/gtest.h>

namespace Sai2Model {

const std::string rrp_urdf = "./urdf/rrpbot.urdf";
const std::string rpspr_urdf = "./urdf/rpsprbot.urdf";




class Sai2ModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Set up common objects for all test cases
        model_rrpbot = new Sai2Model(rrp_urdf);
        model_rpsprbot = new Sai2Model(rpspr_urdf
		);
    }

    void TearDown() override {
        // Clean up common objects after all test cases
        delete model_rrpbot;
        delete model_rpsprbot;
    }

    // Declare variables used by multiple test cases
    Sai2Model* model_rrpbot;
    Sai2Model* model_rpsprbot;
};

// Test the constructor
TEST_F(Sai2ModelTest, Construct) {
	// check all initial values
}

// Test the setQ() and getQ() functions
TEST_F(Sai2ModelTest, SetAndGetQ) {
    // Set a new configuration
    Eigen::VectorXd q(3);
    q << 0.1, 0.2, 0.3;
    model_rrpbot->setQ(q);

    // Get the configuration and compare with the set value
    Eigen::VectorXd q_new = model_rrpbot->q();
    EXPECT_EQ(q, q_new);
}

// Test the setDq() and getDq() functions
TEST_F(Sai2ModelTest, SetAndGetDq) {
    // Set a new joint velocity
    Eigen::VectorXd dq(3);
    dq << 1.0, 2.0, 3.0;
    model_rrpbot->setDq(dq);

    // Get the joint velocity and compare with the set value
    Eigen::VectorXd dq_new = model_rrpbot->dq();
    EXPECT_EQ(dq, dq_new);
}

// Test the sphericalQuat() and setSphericalQuat() functions
TEST_F(Sai2ModelTest, SphericalQuat) {
    // Set a new quaternion for the spherical joint
    Eigen::Quaterniond quat(1.0, 0.0, 0.0, 0.0);
    model_rpsprbot->setSphericalQuat("j2", quat);

    // Get the quaternion and compare with the set value
    Eigen::Quaterniond quat_new = model_rpsprbot->sphericalQuat("j2");
    EXPECT_EQ(quat, quat_new);
}

// Test the isLinkInRobot() function
TEST_F(Sai2ModelTest, IsLinkInRobot) {
    // Check if the specified link is part of the robot
    EXPECT_TRUE(model_rrpbot->isLinkInRobot("link0"));
    EXPECT_TRUE(model_rrpbot->isLinkInRobot("link1"));
    EXPECT_TRUE(model_rrpbot->isLinkInRobot("link2"));
    EXPECT_FALSE(model_rrpbot->isLinkInRobot("link3"));
}

// Test the updateKinematics() function
TEST_F(Sai2ModelTest, UpdateKinematics) {
    // Set a new configuration
    Eigen::VectorXd q(3);
    q << 0.1, 0.2, 0.3;
    model_rrpbot->setQ(q);

    // Update the kinematics
    model_rrpbot->updateKinematics();

    // Perform assertions on the updated kinematics
    // ...

    // For example, check the position of a link
    Eigen::Vector3d pos;
    model_rrpbot->position(pos, "link2", Eigen::Vector3d::Zero());
    EXPECT_NEAR(pos(0), 0.25827013003357957, 1e-6);
    EXPECT_NEAR(pos(1), -0.12719641350943242, 1e-6);
    EXPECT_NEAR(pos(2), 1.2677214253623608, 1e-6);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

}  // namespace Sai2Model
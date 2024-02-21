// 02-update_model:
// example of how to use the different kinematic and dynamic functions to get
// Jacobians, the mass matrix, task space inertia and dynamically consistent
// inverse of the jacobian

#include <Sai2Model.h>

#include <iostream>
#include <random>

using namespace std;

const string robot_fname = "resources/rprbot.urdf";
const string robot_load_fname = "resources/rprbot_load.urdf";

// Function to generate a sample vector with each component within specified ranges
Eigen::VectorXd generatesample_vector(const Eigen::VectorXd& min_vals, const Eigen::VectorXd& max_vals) {
    // Check if the sizes of min_vals and max_vals match
    if (min_vals.size() != max_vals.size()) {
        throw std::invalid_argument("Size mismatch between min_vals and max_vals");
    }

    // Create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    // Create the sample vector
    Eigen::VectorXd sample_vector(min_vals.size());
    for (int i = 0; i < min_vals.size(); ++i) {
        // Create a uniform distribution for each component
        std::uniform_real_distribution<> dis(min_vals(i), max_vals(i));
        sample_vector(i) = dis(gen); // Assign a random value within the specified range for each component
    }
    return sample_vector;
}

int main(int argc, char** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	Sai2Model::Sai2Model* robot_default = new Sai2Model::Sai2Model(robot_fname);
	Sai2Model::Sai2Model* robot = new Sai2Model::Sai2Model(robot_fname);
	Sai2Model::Sai2Model* robot_with_load = new Sai2Model::Sai2Model(robot_load_fname);
	int dof = robot->dof();

	const string ee_link = "link2";
	const Eigen::Vector3d ee_pos_in_link = Eigen::Vector3d(0.0, 0.0, 1.0);

	// add load to robot
	double mass = 0.5;
	Vector3d com = Vector3d(1, 2, 3);
	Matrix3d inertia = Vector3d(0.2, 0.3, 0.4).asDiagonal();
	robot->addLoad(ee_link, mass, com, inertia, "load");

	// generate random configurations between joint limits 
	auto joint_limits = robot->jointLimits();
	VectorXd min_joint_limit(dof), max_joint_limit(dof);
	int cnt = 0;
	for (auto limit : joint_limits) {
		min_joint_limit(cnt) = limit.position_lower;
		max_joint_limit(cnt) = limit.position_upper;
		cnt++;
	}
	
	// test gravity vector, coriolis/centrifugal forces, and mass matrix 
	int n_samples = 1e5;
	for (int i = 0; i < n_samples; ++i) {
		VectorXd q = generatesample_vector(min_joint_limit, max_joint_limit);
		// std::cout << "Sampled q: " << q.transpose() << "\n";
		robot->setQ(q);
		robot->updateModel();
		robot_with_load->setQ(q);
		robot_with_load->updateModel();

		// tests
		if ((robot->jointGravityVector() - robot_with_load->jointGravityVector()).norm() > 1e-10) {
			throw runtime_error("Incorrect gravity vector");
		}
		if ((robot->coriolisForce() - robot_with_load->coriolisForce()).norm() > 1e-10) {
			throw runtime_error("Incorrect coriolis/centrifugal vector");
		}
		if ((robot->M() - robot_with_load->M()).norm() > 1e-10) {
			throw runtime_error("Incorrect mass matrix");
		}
	}

	std::cout << "Tests passed for robot with added load\n---\n";

	// remove load
	robot->removeLoad("load");
	for (int i = 0; i < n_samples; ++i) {
		VectorXd q = generatesample_vector(min_joint_limit, max_joint_limit);
		// std::cout << "Sampled q: " << q.transpose() << "\n";
		robot->setQ(q);
		robot->updateModel();
		robot_default->setQ(q);
		robot_default->updateModel();

		// tests
		if ((robot->jointGravityVector() - robot_default->jointGravityVector()).norm() > 1e-10) {
			throw runtime_error("Incorrect gravity vector");
		}
		if ((robot->coriolisForce() - robot_default->coriolisForce()).norm() > 1e-10) {
			throw runtime_error("Incorrect coriolis/centrifugal vector");
		}
		if ((robot->M() - robot_default->M()).norm() > 1e-10) {
			throw runtime_error("Incorrect mass matrix");
		}
	}

	std::cout << "Tests passed for robot with removed load\n---\n";

	// test adding the same load error 
	std::cout << "Testing adding the same load error\n---\n";
	robot->addLoad(ee_link, mass, com, inertia, "load");
	robot->addLoad(ee_link, mass, com, inertia, "load");

	// test removing a valid load twice
	robot->removeLoad("load");
	std::cout << "Test removing valid load multiple times\n---\n";
	robot->removeLoad("load");

	// test removing load with an invalid name
	std::cout << "Testing invalid name\n---\n";
	robot->removeLoad("invalid_name");

	std::cout << "All tests passed\n";

	return 0;
}
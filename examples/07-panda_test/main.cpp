// 02-update_model:
// example of how to use the different kinematic and dynamic functions to get
// Jacobians, the mass matrix, task space inertia and dynamically consistent
// inverse of the jacobian

#include <Sai2Model.h>

#include <iostream>
#include <random>

using namespace std;

const string robot_fname =
	string(EXAMPLES_FOLDER) + "/02-update_model/panda_arm.urdf";

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

	Sai2Model::Sai2Model* robot = new Sai2Model::Sai2Model(robot_fname);
	int dof = robot->dof();

	const string ee_link = "end-effector";
	const Eigen::Vector3d ee_pos_in_link = Eigen::Vector3d(0.0, 0.0, 1.0);

	auto joint_limits = robot->jointLimits();
	VectorXd min_joint_limit(7);
	VectorXd max_joint_limit(7);
	for (int i = 0; i < 7; ++i) {
		min_joint_limit(i) = joint_limits[i].position_lower;
		max_joint_limit(i) = joint_limits[i].position_upper;
	}

	// test that the sqrt(eigenvalues) and eigenvectors of Lambda_inv is the same as 
	// the singular values and singular vectors of J * L

	int n_test = 1e5;
	for (int i = 0; i < n_test; ++i) {
		VectorXd q = generatesample_vector(min_joint_limit, max_joint_limit);
		// std::cout << "Sampled q: " << q.transpose() << "\n";
		robot->setQ(q);
		robot->updateModel();

		MatrixXd J = robot->J(ee_link, ee_pos_in_link);
		MatrixXd Lambda_inv = (J * robot->MInv() * J.transpose());

		// cholesky decomp
		Eigen::LLT<Eigen::MatrixXd> lltOfA(robot->MInv()); // compute the Cholesky decomposition of A
	    Eigen::MatrixXd L = lltOfA.matrixL(); // retrieve the lower triangular matrix L
		MatrixXd JL = J * L;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(JL, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXd U = svd.matrixU();
    	Eigen::VectorXd singularValues = svd.singularValues();

		// eigendecomp
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(Lambda_inv);
		Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
    	Eigen::MatrixXd eigenvectors = eigensolver.eigenvectors();

		// std::cout << (eigenvalues.array().sqrt() - singularValues.array()) << "\n";
		// std::cout << (U - eigenvectors) << "\n";

		VectorXd eigenvalues_sqrt = eigenvalues.array().sqrt().reverse();

		if ((eigenvalues_sqrt - singularValues).norm() > 1e-10) {
			std::cout << eigenvalues_sqrt.transpose() << "\n";
			std::cout << singularValues.transpose() << "\n";
			throw runtime_error("Not matching");
		}

		// // comparison
		// for (int i = 0; i < 6; ++i) {
		// 	if (std::abs(sqrt(eigenvalues(5)) - singularValues(5)) > 1e-10) {
		// 		std::cout << sqrt(eigenvalues(i)) << ", " << singularValues(i);
		// 		throw runtime_error("Not matching minimum eigenvalues and singular values");
		// 	}
		// }

		if (((U - eigenvectors).norm()) < 1e-10) {
			throw runtime_error("Matching eigenvectors and singular vectors");
		}

	}

	return 0;
}
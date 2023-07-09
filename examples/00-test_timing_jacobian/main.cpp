// 01-create_model_from_file: example of how to create a robot model from a urdf
// file

#include <iostream>
#include <chrono>
#include <Sai2Model.h>

using namespace std;

const string robot_fname = "resources/panda_arm.urdf";
const string link_name = "link7";

int main(int argc, char** argv) {
	cout << "Loading robot file: " << robot_fname << endl;

	Sai2Model::Sai2Model robot(robot_fname, false);

	Eigen::VectorXd q = Eigen::VectorXd::Zero(robot.dof());
	Eigen::MatrixXd J1 = Eigen::MatrixXd::Zero(6, robot.dof());
	Eigen::MatrixXd Jv1 = Eigen::MatrixXd::Zero(3, robot.dof());
	robot.J(J1, link_name);
	robot.Jv(Jv1, link_name);
	Eigen::MatrixXd Lambda1 = Eigen::MatrixXd::Zero(6,6);
	robot.taskInertiaMatrix(Lambda1, J1);

	double t1 = 0;
	double t2 = 0;
	double t3 = 0;


    // Start the timer
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate the duration
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // Print the execution time
    // std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;


	int n_iterations = 1000000;
	for(int i=0 ; i<n_iterations ; ++i) {
		q.setRandom(robot.dof());
		robot.setQ(q);

		start = std::chrono::high_resolution_clock::now();
		Eigen::MatrixXd Lambda3 = robot.taskInertiaMatrix_ter(J1);
		// Eigen::MatrixXd J3 = robot.J_ter(link_name);
		// Eigen::MatrixXd Jv3 = robot.Jv_ter(link_name);
		end = std::chrono::high_resolution_clock::now();
		// cout << "t3: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << endl; 
		t3 += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()*1e-3;

		start = std::chrono::high_resolution_clock::now();
		Eigen::MatrixXd Lambda2 = robot.taskInertiaMatrix_bis(J1);
		// Eigen::MatrixXd J2 = robot.J_bis(link_name);
		// Eigen::MatrixXd Jv2 = robot.Jv_bis(link_name);
		end = std::chrono::high_resolution_clock::now();
		// cout << "t2: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << endl; 
		t2 += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()*1e-3;

		start = std::chrono::high_resolution_clock::now();
		robot.taskInertiaMatrix(Lambda1, J1);
		// robot.J(J1, link_name);
		// robot.Jv(Jv1, link_name);
		end = std::chrono::high_resolution_clock::now();
		// cout << "t1: " << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << endl; 
		t1 += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()*1e-3;



		// cout << "J1: " << J1 << endl;
		// cout << "J2: " << J2 << endl;
		// cout << "J3: " << J3 << endl;
		// cout << (J1-J2).norm() << endl;
		// cout << (J1-J3).norm() << endl;
		// cout << endl;

	}

	cout << "average durations over 1 million random configs (microseconds):"<< endl;
	cout << "method 1: " << t1/(double)n_iterations << endl;
	cout << "method 2: " << t2/(double)n_iterations << endl;
	cout << "method 3: " << t3/(double)n_iterations << endl;

	return 0;
}
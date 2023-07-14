// This tests the grasp matrix function of sai2 model interface.

#include <Sai2Model.h>

#include <iostream>
#include <memory>
#include <string>

using namespace std;
using namespace Eigen;

const string robot_file = "resources/linkage.urdf";

int main() {
	// load robot
	auto linkage = std::make_shared<Sai2Model::Sai2Model>(robot_file);

	// Make perfect tetrahedron
	linkage->setQ(Vector3d(54.7356 / 180.0 * M_PI, 54.7356 / 180.0 * M_PI,
							54.7356 / 180.0 * M_PI));
	linkage->updateModel();

	MatrixXd G, G_inverse;
	Matrix3d R;
	Vector3d center_point = Vector3d::Zero();

	//----------------------------------------
	// test dual contact case with surface contact at one side and point at the
	// other
	//----------------------------------------
	linkage->addEnvironmentalContact("link0", Vector3d(1, 0, 0),
									 Matrix3d::Identity(),
									 Sai2Model::ContactType::SurfaceContact);
	linkage->addEnvironmentalContact("link1", Vector3d(1, 0, 0),
									 Matrix3d::Identity(),
									 Sai2Model::ContactType::PointContact);

	linkage->environmentalGraspMatrixAtGeometricCenter(G, G_inverse, R,
													   center_point);

	cout << "--------------------------------------------" << endl;
	cout << "                  2 contacts                " << endl;
	cout << "--------------------------------------------" << endl;
	cout << "for 2 contact cases, the grasp matrix and its inverse are given "
			"in local frame where the X axis is the axis from the first to the "
			"second contact. The R matrix gives the local frame in robot base "
			"frame"
		 << endl;
	cout << "center point : " << center_point.transpose() << endl << endl;
	cout << "Grasp matrix : \n" << G << endl << endl;
	cout << "Grasp matrix inverse : \n"
		 << G_inverse << endl
		 << "R : \n"
		 << R << endl
		 << endl;
	cout << endl << endl;

	//----------------------------------------
	// test 3 contact case with one more point contact
	//----------------------------------------

	// add a third contact
	linkage->addEnvironmentalContact("link2", Vector3d(1, 0, 0),
									 Matrix3d::Identity(),
									 Sai2Model::ContactType::PointContact);

	linkage->environmentalGraspMatrixAtGeometricCenter(G, G_inverse, R,
													   center_point);

	cout << "--------------------------------------------" << endl;
	cout << "                  3 contacts                " << endl;
	cout << "--------------------------------------------" << endl;
	cout << "for 3 contact cases, the grasp matrix and its inverse are given "
			"in robot base frame. The tensions are in the order 1-2, 1-3, 2-3. "
			"The R is identity."
		 << endl;
	cout << "center point : " << center_point.transpose() << endl << endl;
	cout << "Grasp matrix : \n" << G << endl << endl;
	cout << "Grasp matrix : \n"
		 << G_inverse << endl
		 << "R : \n"
		 << R << endl
		 << endl;

	return 0;
}
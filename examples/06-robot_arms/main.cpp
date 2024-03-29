#include <Sai2Model.h>
#include <parser/Sai2Model_UrdfFolder.h>

#include <iostream>

using namespace std;

const string puma_fname = string(SAI2_MODEL_URDF_FOLDER) + "/puma/puma.urdf";
const string kuka_fname =
	string(SAI2_MODEL_URDF_FOLDER) + "/iiwa7/kuka_iiwa.urdf";

int main(int argc, char** argv) {
	Sai2Model::Sai2Model puma_robot(puma_fname);
	Sai2Model::Sai2Model kuka_robot(kuka_fname);

	cout << endl << endl;
	cout << "puma degrees of freedom : " << puma_robot.dof() << endl;
	cout << "puma joint angles coordinates : " << puma_robot.q().transpose()
		 << endl;
	cout << "puma jacobian at the end effector : \n"
		 << puma_robot.J("end-effector") << endl;

	cout << endl << endl;
	cout << "kuka degrees of freedom : " << kuka_robot.dof() << endl;
	cout << "kuka joint angles coordinates : " << kuka_robot.q().transpose()
		 << endl;
	cout << "kuka jacobian at the end effector : \n"
		 << kuka_robot.J("end-effector") << endl;
	cout << endl << endl;

	return 0;
}
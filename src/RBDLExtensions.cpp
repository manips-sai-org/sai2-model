#include "RBDLExtensions.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

namespace SaiModel {

RBDL_DLLAPI bool InverseKinematicsWithJointLimits(
	Model &model, const Math::VectorNd &Qinit,
	InverseKinematicsConstraintSet &CS, Math::VectorNd &Qmin,
	Math::VectorNd &Qmax, Math::VectorNd &Qres) {
	assert(Qinit.size() == model.q_size);
	assert(Qres.size() == Qinit.size());

	CS.J = MatrixNd::Zero(CS.num_constraints, model.qdot_size);
	CS.e = VectorNd::Zero(CS.num_constraints);
	double mass;

	Qres = Qinit;

	for (CS.num_steps = 0; CS.num_steps < CS.max_steps; CS.num_steps++) {
		UpdateKinematicsCustom(model, &Qres, NULL, NULL);

		for (unsigned int k = 0; k < CS.body_ids.size(); k++) {
			CS.G = MatrixNd::Zero(6, model.qdot_size);
			CalcPointJacobian6D(model, Qres, CS.body_ids[k], CS.body_points[k],
								CS.G, false);
			Vector3d point_base = CalcBodyToBaseCoordinates(
				model, Qres, CS.body_ids[k], CS.body_points[k], false);
			Matrix3d R =
				CalcBodyWorldOrientation(model, Qres, CS.body_ids[k], false);

			// changes rbdl computation of angular velocity as it seemed
			// incorrect
			Eigen::AngleAxisd angular_velocity_aa =
				Eigen::AngleAxisd(R * CS.target_orientations[k].transpose());
			Vector3d angular_velocity =
				angular_velocity_aa.angle() * angular_velocity_aa.axis();

			// assign offsets and Jacobians
			if (CS.constraint_type[k] ==
				InverseKinematicsConstraintSet::ConstraintTypeFull) {
				for (unsigned int i = 0; i < 3; i++) {
					unsigned int row = CS.constraint_row_index[k] + i;
					CS.e[row + 3] = CS.constraint_weight.at(k) *
									(CS.target_positions[k][i] - point_base[i]);
					CS.e[row] =
						CS.constraint_weight.at(k) * angular_velocity[i];
					for (unsigned int j = 0; j < model.qdot_size; j++) {
						CS.J(row + 3, j) =
							CS.constraint_weight.at(k) * CS.G(i + 3, j);
						CS.J(row, j) = CS.constraint_weight.at(k) * CS.G(i, j);
					}
				}
			} else if (CS.constraint_type[k] == InverseKinematicsConstraintSet::
													ConstraintTypeOrientation) {
				for (unsigned int i = 0; i < 3; i++) {
					unsigned int row = CS.constraint_row_index[k] + i;
					CS.e[row] =
						CS.constraint_weight.at(k) * angular_velocity[i];
					for (unsigned int j = 0; j < model.qdot_size; j++) {
						CS.J(row, j) = CS.constraint_weight.at(k) * CS.G(i, j);
					}
				}
			} else if (CS.constraint_type[k] ==
					   InverseKinematicsConstraintSet::ConstraintTypePosition) {
				for (unsigned int i = 0; i < 3; i++) {
					unsigned int row = CS.constraint_row_index[k] + i;
					CS.e[row] = CS.constraint_weight.at(k) *
								(CS.target_positions[k][i] - point_base[i]);
					for (unsigned int j = 0; j < model.qdot_size; j++) {
						CS.J(row, j) =
							CS.constraint_weight.at(k) * CS.G(i + 3, j);
					}
				}
			} else if (CS.constraint_type[k] == InverseKinematicsConstraintSet::
													ConstraintTypePositionXY) {
				for (unsigned int i = 0; i < 2; i++) {
					unsigned int row = CS.constraint_row_index[k] + i;
					CS.e[row] = CS.constraint_weight.at(k) *
								(CS.target_positions[k][i] - point_base[i]);
					for (unsigned int j = 0; j < model.qdot_size; j++) {
						CS.J(row, j) =
							CS.constraint_weight.at(k) * CS.G(i + 3, j);
					}
				}
			} else if (CS.constraint_type[k] == InverseKinematicsConstraintSet::
													ConstraintTypePositionZ) {
				unsigned int row = CS.constraint_row_index[k];
				CS.e[row] = CS.constraint_weight.at(k) *
							(CS.target_positions[k][2] - point_base[2]);
				for (unsigned int j = 0; j < model.qdot_size; j++) {
					CS.J(row, j) = CS.constraint_weight.at(k) * CS.G(2 + 3, j);
				}

			} else if (CS.constraint_type[k] ==
					   InverseKinematicsConstraintSet::
						   ConstraintTypePositionCoMXY) {
				Utils::CalcCenterOfMass(model, Qres, Qres, NULL, mass,
										point_base, NULL, NULL, NULL, NULL,
										false);
				CalcPointJacobian6D(model, Qres, CS.body_ids[k], point_base,
									CS.G, false);

				for (unsigned int i = 0; i < 2; i++) {
					unsigned int row = CS.constraint_row_index[k] + i;
					CS.e[row] = CS.constraint_weight.at(k) *
								(CS.target_positions[k][i] - point_base[i]);
					for (unsigned int j = 0; j < model.qdot_size; j++) {
						CS.J(row, j) =
							CS.constraint_weight.at(k) * CS.G(i + 3, j);
					}
				}
			} else {
				assert(false && !"Invalid inverse kinematics constraint");
			}
		}

		LOG << "J = " << CS.J << std::endl;
		LOG << "e = " << CS.e.transpose() << std::endl;
		CS.error_norm = CS.e.norm();

		// abort if we are getting "close"
		if (CS.error_norm < CS.constraint_tol) {
			LOG << "Reached target close enough after " << CS.num_steps
				<< " steps" << std::endl;
			return true;
		}

		double Ek = 0.;

		for (size_t ei = 0; ei < CS.e.size(); ei++) {
			Ek += CS.e[ei] * CS.e[ei] * 0.5;
		}

		VectorNd ek = CS.J.transpose() * CS.e;
		MatrixNd Wn = CS.lambda * MatrixNd::Identity(Qres.size(), Qres.size());

		assert(ek.size() == Qres.size());

		MatrixNd A = CS.J.transpose() * CS.J + Wn;
		VectorNd delta_theta =
			A.colPivHouseholderQr().solve(CS.J.transpose() * CS.e);

		VectorNd Qres_prev = Qres;
		Qres = Qres + CS.lambda * delta_theta;

		// check joint limits and saturate solution
		for (size_t i = 0; i < Qres.size(); i++) {
			if (Qres[i] < Qmin[i]) {
				Qres[i] = Qmin[i];
			} else if (Qres[i] > Qmax[i]) {
				Qres[i] = Qmax[i];
			}
		}

		// compute norm of change in joint angles taking into account joint
		// saturation
		CS.delta_q_norm = (Qres - Qres_prev).norm();
		if (CS.delta_q_norm < CS.step_tol) {
			LOG << "reached convergence after " << CS.num_steps << " steps"
				<< std::endl;
			return true;
		}
	}

	return false;
}

}  // namespace SaiModel
//
// Created by zhiyixu on 3/14/13.
//
//
//


#include "IKSolver.h"
#include </Users/zhiyixu/Downloads/armadillo-3.800.1/include/armadillo>

using namespace arma;

// use the original skeleton to get dof info
extern Skeleton *pSkeleton_NoDof;

// Calculate joint degrees
// from bone idx_start_bone to idx_end_bone, the desired position of idx_end_bone tip is goalPos
// return the solution to returnSolution, start the iteration from refPosture
void IKSolver::Solve(int idx_start_bone, int idx_end_bone, vector goalPos, Posture *returnSolution, Skeleton *skeleton, Posture *refPosture)
{
	// degree difference when evaluating derivative
	const double delta = 0.005;
	// record all freedom degree
	freedomValue input[100];
	int idx_input = 0;

	// Traverse from start bone to end to find all freedom degree
	Bone *ptr;
	ptr = pSkeleton_NoDof->getBone(pSkeleton_NoDof->getRoot(), idx_start_bone);
	do {
		if (ptr->dofrx == 1) {
			input[idx_input].boneId = ptr->idx;
			input[idx_input].x_y_z = 1;
			idx_input++;
		}
		if (ptr->dofry == 1) {
			input[idx_input].boneId = ptr->idx;
			input[idx_input].x_y_z = 2;
			idx_input++;
		}
		if (ptr->dofrz == 1) {
			input[idx_input].boneId = ptr->idx;
			input[idx_input].x_y_z = 3;
			idx_input++;
		}
		ptr = ptr->child;
	}
	while
			(ptr != NULL);
	input[idx_input].boneId = -1;

	// iteratively improve the solution
	// V = J * theta
	Posture iter = *refPosture;
	mat J = mat(3, idx_input);
	mat V = mat(3, 1);
	mat theta = mat(idx_input, 1);

	int times = 0;
	while (true) {
		// prevent iterating too many times. Mostly it is caused by unreachable position.
		if (times > 3000)
		{
			printf("Not Found\n");
			break;
		}
		times++;
		Posture postureCopy = iter;
		skeleton->setPosture(postureCopy);
		skeleton->computeBoneTipPos();
		vector originalPosition = skeleton->getBoneTipPosition(idx_end_bone);
		vector diff = goalPos - originalPosition;
		// Success
		if (diff.length() < 0.03)
			break;

		V(0, 0) = diff.p[0];
		V(1, 0) = diff.p[1];
		V(2, 0) = diff.p[2];

		// Calculate the Jacobie Matrix
		for (int i = 0; i < idx_input; i++) {
			postureCopy = iter;
			if (input[i].x_y_z == 1)
				postureCopy.bone_rotation[input[i].boneId].p[0] += delta;
			if (input[i].x_y_z == 2)
				postureCopy.bone_rotation[input[i].boneId].p[1] += delta;
			if (input[i].x_y_z == 3)
				postureCopy.bone_rotation[input[i].boneId].p[2] += delta;
			skeleton->setPosture(postureCopy);
			skeleton->computeBoneTipPos();
			vector newPosition = skeleton->getBoneTipPosition(idx_end_bone);
			vector diffV = newPosition - originalPosition;
			J(0, i) = diffV.p[0] / delta;
			J(1, i) = diffV.p[1] / delta;
			J(2, i) = diffV.p[2] / delta;
		}

		// pseudoinverse
		theta = pinv(J) * V;

		// use Euler Method to update theta
		double step = 0.003;
		for (int i = 0; i < idx_input; i++) {
			iter.bone_rotation[input[i].boneId].p[input[i].x_y_z-1] += theta(i, 0) * step;
		}
	}


	*returnSolution = iter;
}

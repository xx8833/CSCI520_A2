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

void IKSolver::Solve(int idx_start_bone, int idx_end_bone, vector goalPos, Posture *returnSolution, Skeleton *skeleton, Posture *refPosture)
{
	const double diff = 0.005;
	BoneValue input[100];
	int idx_input = 0;
	idx_start_bone = 18;
	idx_end_bone = 22;
	Bone *ptr;
	ptr = pSkeleton_NoDof->getBone(pSkeleton_NoDof->getRoot(), 18);
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

	Posture iter = *refPosture;
	mat A = mat(3, idx_input);
	mat V_ = mat(3, 1);
	mat theta_ = mat( idx_input, 1);

	//goalPos[0] = 0.246;
	//goalPos[1] = 1.0;
	//goalPos[2] = -1.955;

	int times = 0;
	while (true) {
		if(times > 10000)
		{
		 printf("Eroor\n");
			break;
		}
		times ++;
		Posture base = iter;
		skeleton->setPosture(base);
		skeleton->computeBoneTipPos();
		vector before = skeleton->getBoneTipPosition(idx_end_bone);
		vector V = goalPos - before;
		double diffDis = V.length();
		if (diffDis < 0.02)
		{
			printf("Bingo\n");
			break;
		}

		//printf("N %d %lf %lf %lf \n",times, before.p[0],before.p[1],before.p[2]);

		V_(0, 0) = V.p[0];
		V_(1, 0) = V.p[1];
		V_(2, 0) = V.p[2];

		for (int i = 0; i < idx_input; i++) {
			base = iter;
			if (input[i].x_y_z == 1)
				base.bone_rotation[input[i].boneId].p[0] += diff;
			if (input[i].x_y_z == 2)
				base.bone_rotation[input[i].boneId].p[1] += diff;
			if (input[i].x_y_z == 3)
				base.bone_rotation[input[i].boneId].p[2] += diff;
			skeleton->setPosture(base);
			skeleton->computeBoneTipPos();
			vector after = skeleton->getBoneTipPosition(idx_end_bone);
			vector diffV = after - before;
			A(0, i) = diffV.p[0] / diff;
			A(1, i) = diffV.p[1] / diff;
			A(2, i) = diffV.p[2] / diff;
			// printf("%lf %lf %lf\n", A(0, i), A(1, i), A(2, i));
		}
		//A.print();
		theta_ = inv(A.t() * A) * (A.t()) * V_;
		mat tempM = (inv(A.t() * A) * (A.t()));
		theta_ = pinv(A) * V_;
		//theta_.print();
		//V_.print();
		double step = 0.001;
		for (int i = 0; i < idx_input; i++) {
			if (input[i].x_y_z == 1)
				iter.bone_rotation[input[i].boneId].p[0] += theta_(i,0) * step;
			if (input[i].x_y_z == 2)
				iter.bone_rotation[input[i].boneId].p[1] += theta_(i,0) * step;
			if (input[i].x_y_z == 3)
				iter.bone_rotation[input[i].boneId].p[2] += theta_(i,0) * step;
		}
	}
	*returnSolution = iter;
	int rubb = 98;

}

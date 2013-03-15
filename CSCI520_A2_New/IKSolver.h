//
// Created by zhiyixu on 3/14/13.
//
//
//



#ifndef __IKSolver_H_
#define __IKSolver_H_

#include <iostream>
#include "vector.h"
#include "skeleton.h"
#include "posture.h"
struct BoneValue
{
	int boneId;
	int x_y_z;
	double value;
};

class IKSolver {
	public:
	// goalPosture is the solution, here used to get start position
	void Solve(int idx_start_bone,int idx_end_bone,vector goalPos,Posture * returnSolution,Skeleton * skeleton,Posture * refPosture);

};


#endif //__IKSolver_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "transform.h"
#include "types.h"
#include "IKSolver.h"

Interpolator::Interpolator()
{
	//Set default interpolation type
	m_InterpolationType = LINEAR;

	//set default angle representation to use for interpolation
	m_AngleRepresentation = EULER;

	num_keyFrames = 0;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion *pInputMotion, Motion **pOutputMotion,
		int N)
{
	//Allocate new motion
	*pOutputMotion = new Motion(pInputMotion->GetNumFrames(),
			pInputMotion->GetSkeleton());

	//Perform the interpolation
	if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
		LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == LINEAR)
			&& (m_AngleRepresentation == QUATERNION))
		LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == BEZIER)
			&& (m_AngleRepresentation == EULER))
		BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
	else if ((m_InterpolationType == BEZIER)
			&& (m_AngleRepresentation == QUATERNION))
		BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
	else {
		printf("Error: unknown interpolation / angle representation type.\n");
		exit(1);
	}

	// record for drawing graph
	// for(int frame = 1 ; frame <=  1000 ; frame ++)
	// {
	// 	 printf("%d %lf\n",frame,((*pOutputMotion)->GetPosture(frame))->bone_rotation[18][0]);
	// }
}

void Interpolator::LinearInterpolationEuler(Motion *pInputMotion,
		Motion *pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	// keyframe ID  starts from 1
	// frame number starts from 0
	// frame number 0 ..... 21 ..... 62 ..
	// keyframe ID  1       2        3  ..
	// in non time uniform situation, the interval is different
	// To get KeyFrame Position, use keyFramePos array
	for (int keyFrameID = 1; keyFrameID < num_keyFrames; keyFrameID++) {
		int startKeyframe = keyFramePos[keyFrameID];
		int endKeyframe = keyFramePos[keyFrameID + 1];

		Posture *startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture *endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= endKeyframe - startKeyframe - 1; frame++) {
			Posture interpolatedPosture;
			double t = 1.0 * frame / (endKeyframe - startKeyframe );

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t)
					+ endPosture->root_pos * t;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
				interpolatedPosture.bone_rotation[bone] =
						startPosture->bone_rotation[bone] * (1 - t)
								+ endPosture->bone_rotation[bone] * t;

			pOutputMotion->SetPosture(startKeyframe + frame,
					interpolatedPosture);
		}
	}

	for (int frame = keyFramePos[num_keyFrames] + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}


void Interpolator::BezierInterpolationEuler(Motion *pInputMotion,
		Motion *pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	// keyframe ID  starts from 1
	// frame number starts from 0
	// frame number 0 ..... 21 ..... 62 ..
	// keyframe ID  1       2        3  ..
	// in non time uniform situation, the interval is different
	// To get KeyFrame Position, use keyFramePos array
	if (num_keyFrames <= 3)
		throw "Too less key frames to do the Interpolation";
	for (int keyFrameID = 1; keyFrameID < num_keyFrames; keyFrameID++) {
		int startKeyframe = keyFramePos[keyFrameID];
		int endKeyframe = keyFramePos[keyFrameID + 1];
		// p_n,p_(n+1)
		Posture *startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture *endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame < endKeyframe - startKeyframe - 1; frame++) {
			Posture interpolatedPosture;
			double t = 1.0 * frame / (endKeyframe - startKeyframe);

			// interpolate root position
			// a_n,b_(n+1)
			vector a, b;
			// p_(n-1),p_n,p_(n+1),p_(n+2)
			vector p0, p1, p2, p3;
			p1 = startPosture->root_pos;
			p2 = endPosture->root_pos;

			// a_n
			// special case for a1
			if (keyFrameID == 1) {
				p3 = pInputMotion->GetPosture(keyFramePos[keyFrameID + 2])->root_pos;
				a = Lerp(p1, Lerp(p3, p2, 2), 1.0 / 3);
			}
			else {
				p0 = pInputMotion->GetPosture(keyFramePos[keyFrameID - 1])->root_pos;
				// (a_n)_
				vector a_ = Lerp(Lerp(p0, p1, 2), p2, 0.5);
				a = Lerp(p1, a_, 1.0 / 3);
			}

			// b_(n+1)
			// special case for bn
			if (keyFrameID == num_keyFrames - 1)
				b = Lerp(p2, Lerp(p0, p1, 2), 1.0 / 3);
			else {
				p3 = pInputMotion->GetPosture(keyFramePos[keyFrameID + 2])->root_pos;
				// (a_n+1)_
				vector a_1 = Lerp(Lerp(p1, p2, 2), p3, 0.5);
				b = Lerp(p2, a_1, -1.0 / 3);
			}
			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a, b, p2);

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
				// interpolate bone rotation
				// a_n,b_(n+1)
				vector a, b;
				// p_(n-1),p_n,p_(n+1),p_(n+2)
				vector p0, p1, p2, p3;
				p1 = startPosture->bone_rotation[bone];
				p2 = endPosture->bone_rotation[bone];

				// a_n
				// special case for a1
				if (keyFrameID == 1) {
					p3 = pInputMotion->GetPosture(keyFramePos[keyFrameID + 2])->bone_rotation[bone];
					a = Lerp(p1, Lerp(p3, p2, 2), 1.0 / 3);
				}
				else {
					p0 = pInputMotion->GetPosture(keyFramePos[keyFrameID - 1])->bone_rotation[bone];
					// (a_n)_
					vector a_ = Lerp(Lerp(p0, p1, 2), p2, 0.5);
					a = Lerp(p1, a_, 1.0 / 3);
				}

				// b_(n+1)
				// special case for bn
				if (keyFrameID == num_keyFrames - 1)
					b = Lerp(p2, Lerp(p0, p1, 2), 1.0 / 3);
				else {
					p3 = pInputMotion->GetPosture(keyFramePos[keyFrameID + 2])->bone_rotation[bone];
					// (a_n+1)_
					vector a_1 = Lerp(Lerp(p1, p2, 2), p3, 0.5);
					b = Lerp(p2, a_1, -1.0 / 3);
				}
				interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, p1, a, b, p2);

			}
			pOutputMotion->SetPosture(startKeyframe + frame,
					interpolatedPosture);
		}

	}

	for (int frame = keyFramePos[num_keyFrames] + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion *pInputMotion,
		Motion *pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	// keyframe ID  starts from 1
	// frame number starts from 0
	// frame number 0 ..... 21 ..... 62 ..
	// keyframe ID  1       2        3  ..
	// in non time uniform situation, the interval is different
	// To get KeyFrame Position, use keyFramePos array
	for (int keyFrameID = 1; keyFrameID < num_keyFrames; keyFrameID++) {
		int startKeyframe = keyFramePos[keyFrameID];
		int endKeyframe = keyFramePos[keyFrameID + 1];

		Posture *startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture *endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= endKeyframe - startKeyframe - 1; frame++) {
			Posture interpolatedPosture;
			double t = 1.0 * frame / (endKeyframe - startKeyframe);

			// interpolate root position
			interpolatedPosture.root_pos = startPosture->root_pos * (1 - t)
					+ endPosture->root_pos * t;

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
				Quaternion<double> start, end, result;
				double startEuler[3], EndEuler[3], ResultEuler[3];
				startPosture->bone_rotation[bone].getValue(startEuler);
				endPosture->bone_rotation[bone].getValue(EndEuler);
				Euler2Quaternion(startEuler, start);
				Euler2Quaternion(EndEuler, end);
				result = Slerp(start, end, t);
				Quaternion2Euler(result, ResultEuler);
				interpolatedPosture.bone_rotation[bone] = ResultEuler;
			}

			if (m_EnableIKSolver) {
				// Get the actual hands and feet position and root position
				// 5 left toes, 10 right toes, 22 left finger, 29 right finger
				interpolatedPosture.root_pos = pInputMotion->GetPosture(startKeyframe + frame)->root_pos;
				Skeleton *skeleton = pInputMotion->GetSkeleton();
				skeleton->setPosture(*(pInputMotion->GetPosture(startKeyframe + frame)));
				skeleton->computeBoneTipPos();
				vector v22 = skeleton->getBoneTipPosition(22);
				vector v5 = skeleton->getBoneTipPosition(5);
				vector v10 = skeleton->getBoneTipPosition(10);
				vector v29 = skeleton->getBoneTipPosition(29);
				// Adjust current angle to reach these position
				IKSolver::Solve(18, 22, v22, &interpolatedPosture, skeleton, &interpolatedPosture);
				IKSolver::Solve(2, 5, v5, &interpolatedPosture, skeleton, &interpolatedPosture);
				IKSolver::Solve(7, 10, v10, &interpolatedPosture, skeleton, &interpolatedPosture);
				IKSolver::Solve(25, 29, v29, &interpolatedPosture, skeleton, &interpolatedPosture);
			}

			pOutputMotion->SetPosture(startKeyframe + frame,
					interpolatedPosture);
		}

		startKeyframe = endKeyframe;
	}

	for (int frame = keyFramePos[num_keyFrames] + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion *pInputMotion,
		Motion *pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	// keyframe ID  starts from 1
	// frame number starts from 0
	// frame number 0 ..... 21 ..... 62 ..
	// keyframe ID  1       2        3  ..
	// in non time uniform situation, the interval is different
	// To get KeyFrame Position, use keyFramePos array
	if (num_keyFrames <= 3)
		throw "Too less key frames to do Interpolate";
	for (int keyFrameID = 1; keyFrameID < num_keyFrames; keyFrameID++) {

		int startKeyframe = keyFramePos[keyFrameID];
		int endKeyframe = keyFramePos[keyFrameID + 1];
		// q_n
		Posture *startPosture = pInputMotion->GetPosture(startKeyframe);
		// q_(n+1)
		Posture *endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= endKeyframe - startKeyframe - 1; frame++) {
			Posture interpolatedPosture;
			double t = 1.0 * frame / (endKeyframe - startKeyframe);

			// interpolate root position
			// a_n,b_(n+1)
			vector a, b;
			// p_(n-1),p_n,p_(n+1),p_(n+2)
			vector p0, p1, p2, p3;
			p1 = startPosture->root_pos;
			p2 = endPosture->root_pos;

			// a_n
			// special case for a1
			if (keyFrameID == 1) {
				p3 = pInputMotion->GetPosture(keyFramePos[keyFrameID + 2])->root_pos;
				a = Lerp(p1, Lerp(p3, p2, 2), 1.0 / 3);
			}
			else {
				p0 = pInputMotion->GetPosture(keyFramePos[keyFrameID - 1])->root_pos;
				// (a_n)_
				vector a_ = Lerp(Lerp(p0, p1, 2), p2, 0.5);
				a = Lerp(p1, a_, 1.0 / 3);
			}

			// b_(n+1)
			// special case for bn
			if (keyFrameID == num_keyFrames - 1)
				b = Lerp(p2, Lerp(p0, p1, 2), 1.0 / 3);
			else {
				p3 = pInputMotion->GetPosture(keyFramePos[keyFrameID + 2])->root_pos;
				// (a_n+1)_
				vector a_1 = Lerp(Lerp(p1, p2, 2), p3, 0.5);
				b = Lerp(p2, a_1, -1.0 / 3);
			}
			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a, b, p2);

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
				// interpolate bone rotation
				// a_n,b_(n+1)
				Quaternion<double> a, b;
				// p_(n-1),p_n,p_(n+1),p_(n+2)
				Quaternion<double> q0, q1, q2, q3, resultQ;
				double e0[3], e1[3], e2[3], e3[3], resultEuler[3];

				startPosture->bone_rotation[bone].getValue(e1);
				endPosture->bone_rotation[bone].getValue(e2);
				Euler2Quaternion(e1, q1);
				Euler2Quaternion(e2, q2);

				// a_n
				// special case for a1
				if (keyFrameID == 1) {
					pInputMotion->GetPosture(keyFramePos[keyFrameID + 2])->bone_rotation[bone].getValue(e3);
					Euler2Quaternion(e3, q3);
					Quaternion<double> temp = Double(q3, q2);
					a = Slerp(q1, temp, 1.0 / 3);
				}
				else {
					pInputMotion->GetPosture(keyFramePos[keyFrameID - 1])->bone_rotation[bone].getValue(e0);
					Euler2Quaternion(e0, q0);
					// (a_n)_
					Quaternion<double> temp = Double(q0, q1);
					Quaternion<double> a_ = Slerp(temp, q2, 0.5);
					a = Slerp(q1, a_, 1.0 / 3);
				}

				// b_(n+1)
				// special case for bn
				if (keyFrameID == num_keyFrames - 1) {
					Quaternion<double> temp = Slerp(q0, q1, 2);
					b = Slerp(q2, temp, 1.0 / 3);
				}
				else {
					pInputMotion->GetPosture(keyFramePos[keyFrameID + 2])->bone_rotation[bone].getValue(e3);
					Euler2Quaternion(e3, q3);
					// (a_n+1)_
					Quaternion<double> temp = Double(q1, q2);
					Quaternion<double> a_1 = Slerp(temp, q3, 0.5);
					b = Slerp(q2, a_1, -1.0 / 3);
				}

				resultQ = DeCasteljauQuaternion(t, q1, a, b, q2);
				Quaternion2Euler(resultQ, resultEuler);
				interpolatedPosture.bone_rotation[bone] = resultEuler;
			}

			if (m_EnableIKSolver) {
				// Get the actual hands and feet position and root position
				// 5 left toes, 10 right toes, 22 left finger, 29 right finger
				interpolatedPosture.root_pos = pInputMotion->GetPosture(startKeyframe + frame)->root_pos;
				Skeleton *skeleton = pInputMotion->GetSkeleton();
				skeleton->setPosture(*(pInputMotion->GetPosture(startKeyframe + frame)));
				skeleton->computeBoneTipPos();
				vector v22 = skeleton->getBoneTipPosition(22);
				vector v5 = skeleton->getBoneTipPosition(5);
				vector v10 = skeleton->getBoneTipPosition(10);
				vector v29 = skeleton->getBoneTipPosition(29);

				// Debug 
				// tt->setPosture(interpolatedPosture);
				// tt->computeBoneTipPos();
				// vector v2 = tt->getBoneTipPosition(22);
				// IKSolver k;
				///if((v1-v2).length() > 0.1)
				///{
				///	printf("%d %lf \n",startKeyframe + frame, (v1-v2).length());
				///}

				// Adjust current angle to reach these position
				IKSolver::Solve(18, 22, v22, &interpolatedPosture, skeleton, &interpolatedPosture);
				IKSolver::Solve(2, 5, v5, &interpolatedPosture, skeleton, &interpolatedPosture);
				IKSolver::Solve(7, 10, v10, &interpolatedPosture, skeleton, &interpolatedPosture);
				IKSolver::Solve(25, 29, v29, &interpolatedPosture, skeleton, &interpolatedPosture);
			}

			pOutputMotion->SetPosture(startKeyframe + frame,
					interpolatedPosture);
		}

	}

	for (int frame = keyFramePos[num_keyFrames]; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q)
{
	double Rotation[9];
	Euler2Rotation(angles, Rotation);
	q = q.Matrix2Quaternion(Rotation);
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3])
{
	double Rotation[9];
	q.Quaternion2Matrix(Rotation);
	Rotation2Euler(Rotation, angles);
}

// Reference: Physically based Rendering from theory to implementation 2nd
// Reference: Computer Animation Algorithm & Techniques 2nd
Quaternion<double> Interpolator::Slerp(Quaternion<double>  & qStart, Quaternion<double>  & qEnd, double t)
{
	Quaternion<double> result;
	double cosTheta = qStart.Gets() * qEnd.Gets() + qStart.Getx() * qEnd.Getx()
			+ qStart.Gety() * qEnd.Gety() + qStart.Getz() * qEnd.Getz();

	// choose the short path
	if (cosTheta < 0.0) {
		cosTheta = -1 * cosTheta;
		qEnd = -1. * qEnd;
	}
	// avoid dividing by zero
	if (cosTheta > 0.9995) {
		result = (1 - t) * qStart + t * qEnd;
	}
	else {
		double theta = acos(cosTheta);
		result = sin((1 - t) * theta) / sin(theta) * qStart + sin(t * theta) / sin(theta) * qEnd;
	}
	result.Normalize();
	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p,
		Quaternion<double> q)
{
	return Slerp(p, q, 2.0);
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2,
		vector p3)
{
	vector temp1 = Lerp(p0, p1, t);
	vector temp2 = Lerp(p1, p2, t);
	vector temp3 = Lerp(p2, p3, t);
	temp1 = Lerp(temp1, temp2, t);
	temp2 = Lerp(temp2, temp3, t);
	return Lerp(temp1, temp2, t);
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t,
		Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2,
		Quaternion<double> p3)
{
	Quaternion<double> temp1 = Slerp(p0, p1, t);
	Quaternion<double> temp2 = Slerp(p1, p2, t);
	Quaternion<double> temp3 = Slerp(p2, p3, t);
	temp1 = Slerp(temp1, temp2, t);
	temp2 = Slerp(temp2, temp3, t);
	return Slerp(temp1, temp2, t);
}

void Interpolator::SetTimeUniformKeyframe(int interval, int length)
{
	int keyFrameID, currentKeyFramePos = 0;
	for (keyFrameID = 1; ; keyFrameID++) {
		keyFramePos[keyFrameID] = currentKeyFramePos;
		currentKeyFramePos += interval + 1;
		if (currentKeyFramePos >= length)
			break;
	}
	num_keyFrames = keyFrameID;
}

void Interpolator::AddNextKeyframePos(int keyFramePos)
{
	num_keyFrames++;
	this->keyFramePos[num_keyFrames] = keyFramePos;
}


void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
	double cy = sqrt(R[0] * R[0] + R[3] * R[3]);

	if (cy > 16 * DBL_EPSILON) {
		angles[0] = atan2(R[7], R[8]);
		angles[1] = atan2(-R[6], cy);
		angles[2] = atan2(R[3], R[0]);
	} else {
		angles[0] = atan2(-R[5], R[4]);
		angles[1] = atan2(-R[6], cy);
		angles[2] = 0;
	}

	for (int i = 0; i < 3; i++)
		angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
	// this implementation is straightforward but slow
	double Rx[4][4], Ry[4][4], Rz[4][4], Rtemp[4][4], Rresult[4][4];
	rotationZ(Rz, angles[2]);
	rotationY(Ry, angles[1]);
	rotationX(Rx, angles[0]);
	matrix_mult(Rz, Ry, Rtemp);
	matrix_mult(Rtemp, Rx, Rresult);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i * 3 + j] = Rresult[i][j];
}

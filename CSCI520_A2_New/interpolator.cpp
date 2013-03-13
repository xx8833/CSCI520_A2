#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "transform.h"
#include "types.h"

Interpolator::Interpolator()
{
	//Set default interpolation type
	m_InterpolationType = LINEAR;

	//set default angle representation to use for interpolation
	m_AngleRepresentation = EULER;
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
}

void Interpolator::LinearInterpolationEuler(Motion *pInputMotion,
		Motion *pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	int startKeyframe = 0;
	while (startKeyframe + N + 1 < inputLength) {
		int endKeyframe = startKeyframe + N + 1;

		Posture *startPosture = pInputMotion->GetPosture(startKeyframe);
		Posture *endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++) {
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

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

		startKeyframe = endKeyframe;
	}

	for (int frame = startKeyframe + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
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

void Interpolator::BezierInterpolationEuler(Motion *pInputMotion,
		Motion *pOutputMotion, int N)
{
	int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

	// keyframe ID (n) starts from 1
	// frame number starts from 0
	// frame number 0 ..N.. 31 ..N.. 62 ..
	// keyframe ID  1       2        3  ..
	int maxKeyFrameID = inputLength / (N+1) + 1;
	if(maxKeyFrameID <= 3)
		throw "Too less key frames to do Interpolate" ;
	for(int keyFrameID = 1 ; keyFrameID < maxKeyFrameID ; keyFrameID ++ )
	{
		int startKeyframe = (keyFrameID - 1) * (N + 1);
		int endKeyframe = startKeyframe + N + 1;
		// q_n
		Posture *startPosture = pInputMotion->GetPosture(startKeyframe);
		// q_(n+1)
		Posture *endPosture = pInputMotion->GetPosture(endKeyframe);

		// copy start and end keyframe
		pOutputMotion->SetPosture(startKeyframe, *startPosture);
		pOutputMotion->SetPosture(endKeyframe, *endPosture);

		// interpolate in between
		for (int frame = 1; frame <= N; frame++) {
			Posture interpolatedPosture;
			double t = 1.0 * frame / (N + 1);

			// interpolate root position
			// a_n,b_(n+1)
			vector a,b;
			// p_(n-1),p_n,p_(n+1),p_(n+2)
			vector p0,p1,p2,p3;
			p1 = startPosture->root_pos;
			p2 = endPosture->root_pos;

			if(keyFrameID == 1)
			{
				p3 = pInputMotion->GetPosture(endKeyframe + N + 1)->root_pos;
				a = Lerp(p1,Lerp(p3, p2, 2),1.0/3);
			}
			else
			{
				p0 = pInputMotion->GetPosture(startKeyframe - N - 1)->root_pos;
				// (a_n)_
				vector a_ = Lerp(Lerp(p0, p1, 2), p2, 0.5);
				a = Lerp(p1, a_, 1.0/3);
			}
			if(keyFrameID == maxKeyFrameID - 1)
				b = Lerp(p2, Lerp(p0, p1, 2),1.0/3);
			else
			{
				p3 = pInputMotion->GetPosture(endKeyframe + N + 1)->root_pos;
				// (a_n+1)_
				vector a_1 = Lerp(Lerp(p1, p2, 2), p3, 0.5);
				b = Lerp(p2, a_1, -1.0/3);
			}
			interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a, b, p2);

			// interpolate bone rotations
			for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
			{
				// interpolate bone rotation
				// a_n,b_(n+1)
				vector a,b;
				// p_(n-1),p_n,p_(n+1),p_(n+2)
				vector p0,p1,p2,p3;
				p1 = startPosture->bone_rotation[bone];
				p2 = endPosture->bone_rotation[bone];

				if(keyFrameID == 1)
				{
					p3 = pInputMotion->GetPosture(endKeyframe + N + 1)->bone_rotation[bone];
					a = Lerp(p1,Lerp(p3, p2, 2),1.0/3);
				}
				else
				{
					p0 = pInputMotion->GetPosture(startKeyframe - N - 1)->bone_rotation[bone];
					// (a_n)_
					vector a_ = Lerp(Lerp(p0, p1, 2), p2, 0.5);
					a = Lerp(p1, a_, 1.0/3);
				}
				if(keyFrameID == maxKeyFrameID - 1)
					b = Lerp(p2, Lerp(p0, p1, 2),1.0/3);
				else
				{
					p3 = pInputMotion->GetPosture(endKeyframe + N + 1)->bone_rotation[bone];
					// (a_n+1)_
					vector a_1 = Lerp(Lerp(p1, p2, 2), p3, 0.5);
					b = Lerp(p2, a_1, -1.0/3);
				}
				interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, p1, a, b, p2);


			}

			pOutputMotion->SetPosture(startKeyframe + frame,
					interpolatedPosture);
		}

	}

	for (int frame = (maxKeyFrameID - 1)  * (N + 1) + 1; frame < inputLength; frame++)
		pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion *pInputMotion,
		Motion *pOutputMotion, int N)
{
	// students should implement this
}

void Interpolator::BezierInterpolationQuaternion(Motion *pInputMotion,
		Motion *pOutputMotion, int N)
{
	// students should implement this
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q)
{
	// students should implement this
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3])
{
	// students should implement this
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart,
		Quaternion<double> & qEnd_)
{
	// students should implement this
	Quaternion<double> result;
	return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p,
		Quaternion<double> q)
{
	// students should implement this
	Quaternion<double> result;
	return result;
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
	// students should implement this
	Quaternion<double> result;
	return result;
}


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
	// students should implement this
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
	vector temp1,temp2,temp3;
	temp1 = p0 * (1-t) + p1 * t;
	temp2 = p1 * (1-t) + p2 * t;
	temp3 = p2 * (1-t) + p3 * t;
	temp1 = temp1 * (1-t) + temp2 * t;
	temp2 = temp2 * (1-t) + temp3 * t;
	temp1 = temp1 * (1-t) + temp2 * t;
	return temp1;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t,
		Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2,
		Quaternion<double> p3)
{
	// students should implement this
	Quaternion<double> result;
	return result;
}


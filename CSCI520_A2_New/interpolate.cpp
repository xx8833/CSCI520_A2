#ifdef WIN32
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>

#include "interpolator.h"
#include "motion.h"

int main(int argc, char **argv) {

	double R[9],R2[3];
	R2[0] = 12.1;
	R2[1] = 19.0;
	R2[2] = -351.7;
	Interpolator::Euler2Rotation(R2, R);
	Interpolator::Rotation2Euler(R, R2);

	vector v1(0,0,0);
	vector v2(0,1,0);
	vector v3(1,1,0);
	vector v4(1,0,0);
	vector iv;
	iv = Interpolator::DeCasteljauEuler(0.5, v1, v2, v3, v4);
	iv = Interpolator::DeCasteljauEuler(0.0, v1, v2, v3, v4);
	iv = Interpolator::DeCasteljauEuler(1.0, v1, v2, v3, v4);
	iv = Interpolator::DeCasteljauEuler(0.1, v1, v2, v3, v4);



	if (argc != 7) {
		printf("Interpolates motion capture data.");
		printf(
				"Usage: %s <input skeleton file> <input motion capture file> <interpolation type> <angle representation for interpolation> <N> <output motion capture file>\n",
				argv[0]);
		printf("  interpolation method:\n");
		printf("    l: linear\n");
		printf("    b: Bezier\n");
		printf("  angle representation for interpolation:\n");
		printf("    e: Euler angles\n");
		printf("    q: quaternions\n");
		printf("  N: number of skipped frames\n");
		printf("Example: %s skeleton.asf motion.amc l e 5 outputMotion.amc\n",
				argv[0]);
		return -1;
	}

	char * inputSkeletonFile = argv[1];
	char * inputMotionCaptureFile = argv[2];
	char * interpolationTypeString = argv[3];
	char * angleRepresentationString = argv[4];
	char * NString = argv[5];
	char * outputMotionCaptureFile = argv[6];

	int N = strtol(NString, NULL, 10);
	if (N < 0) {
		printf("Error: invalid N value (%d).\n", N);
		exit(1);
	}
	printf("N=%d\n", N);

	Skeleton * pSkeleton = NULL;	// skeleton as read from an ASF file (input)
	Motion * pInputMotion = NULL; // motion as read from an AMC file (input)

	printf("Loading skeleton from %s...\n", inputSkeletonFile);
	try {
		pSkeleton = new Skeleton(inputSkeletonFile, MOCAP_SCALE);
	} catch (int exceptionCode) {
		printf("Error: failed to load skeleton from %s. Code: %d\n",
				inputSkeletonFile, exceptionCode);
		exit(1);
	}

	printf("Loading input motion from %s...\n", inputMotionCaptureFile);
	try {
		pInputMotion = new Motion(inputMotionCaptureFile, MOCAP_SCALE,
				pSkeleton);
	} catch (int exceptionCode) {
		printf("Error: failed to load motion from %s. Code: %d\n",
				inputMotionCaptureFile, exceptionCode);
		exit(1);
	}

	pSkeleton->enableAllRotationalDOFs();

	InterpolationType interpolationType;
	if (interpolationTypeString[0] == 'l')
		interpolationType = LINEAR;
	else if (interpolationTypeString[0] == 'b')
		interpolationType = BEZIER;
	else {
		printf("Error: unknown interpolation type: %s\n",
				interpolationTypeString);
		exit(1);
	}
	printf("Interpolation type is: %s\n",
			(interpolationType == LINEAR) ? "LINEAR" : "BEZIER");

	AngleRepresentation angleRepresentation;
	if (angleRepresentationString[0] == 'e')
		angleRepresentation = EULER;
	else if (angleRepresentationString[0] == 'q')
		angleRepresentation = QUATERNION;
	else {
		printf("Error: unknown angle representation: %s\n",
				angleRepresentationString);
		exit(1);
	}
	printf("Angle representation for interpolation is: %s\n",
			(angleRepresentation == EULER) ? "EULER" : "QUATERNION");

	Interpolator interpolator;
	interpolator.SetInterpolationType(interpolationType);
	interpolator.SetAngleRepresentation(angleRepresentation);

	printf("Interpolating...\n");
	Motion * pOutputMotion; // interpolated motion (output)
	interpolator.Interpolate(pInputMotion, &pOutputMotion, N);
	if (pOutputMotion == NULL) {
		printf("Error: interpolation failed. No output generated.\n");
		exit(1);
	}
	printf("Interpolation completed.\n");

	printf("Writing output motion capture file to %s...\n",
			outputMotionCaptureFile);
	int forceAllJointsBe3DOF = 1;
	pOutputMotion->writeAMCfile(outputMotionCaptureFile, 0.06,
			forceAllJointsBe3DOF);

	return 0;
}


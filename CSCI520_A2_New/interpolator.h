/*
 interpolator.h
 
 Create interpolated motion.

 Revision 1 - Alla and Kiran, Jan 18, 2002
 Revision 2 - Jernej Barbic, Yili Zhao, Feb 2012
 */

#ifndef _INTERPOLATOR_H
#define _INTERPOLATOR_H

#include "motion.h"
#include "quaternion.h"
#include <iostream>

enum InterpolationType {
	LINEAR = 0, BEZIER = 1
};

enum AngleRepresentation {
	EULER = 0, QUATERNION = 1
};

class Interpolator {
public:
	//constructor, destructor
	Interpolator();
	~Interpolator();

	//Set interpolation type
	void SetInterpolationType(InterpolationType interpolationType) {
		m_InterpolationType = interpolationType;
	}
	;
	//Set angle representation for interpolation
	void SetAngleRepresentation(AngleRepresentation angleRepresentation) {
		m_AngleRepresentation = angleRepresentation;
	}
	;

	//Set whether to use IK solver or not
	void SetIKSolverOnOFF(bool EnableIkSolver) {
		m_EnableIKSolver = EnableIkSolver;
	}
	;
	//Create interpolated motion and store it into pOutputMotion (which will also be allocated)
	void Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N);

	// set time uniform keyframe
	void SetTimeUniformKeyframe(int interval,int length);

	// set keyframe position
	void AddNextKeyframePos(int keyFramePos);
private:
	InterpolationType m_InterpolationType; //Interpolation type (Linear, Bezier)
	AngleRepresentation m_AngleRepresentation; //Angle representation (Euler, Quaternion)
	bool m_EnableIKSolver;

	int keyFramePos[10000];
	int num_keyFrames;
	// conversion routines
	// angles are given in degrees; assume XYZ Euler angle order
	void Rotation2Euler(double R[9], double angles[3]);
	void Euler2Rotation(double angles[3], double R[9]);
	void Euler2Quaternion(double angles[3], Quaternion<double> & q);
	void Quaternion2Euler(Quaternion<double> & q, double angles[3]);

	// quaternion interpolation
	Quaternion<double> Slerp(Quaternion<double>  & qStart, Quaternion<double>  & qEnd, double t);
	Quaternion<double> Double(Quaternion<double> p, Quaternion<double> q);

	// interpolation routines
	void LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion,
			int N);
	void BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion,
			int N);
	void LinearInterpolationQuaternion(Motion * pInputMotion,
			Motion * pOutputMotion, int N);
	void BezierInterpolationQuaternion(Motion * pInputMotion,
			Motion * pOutputMotion, int N);

	// Bezier spline evaluation
	vector DeCasteljauEuler(double t, vector p0, vector p1, vector p2,
			vector p3); // evaluate Bezier spline at t, using DeCasteljau construction, vector version
	Quaternion<double> DeCasteljauQuaternion(double t, Quaternion<double> p0,
			Quaternion<double> p1, Quaternion<double> p2,
			Quaternion<double> p3); // evaluate Bezier spline at t, using DeCasteljau construction, Quaternion version

};

#endif


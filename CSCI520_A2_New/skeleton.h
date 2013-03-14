/*
 skeleton.h

 Definition of the skeleton. 

 Revision 1 - Steve Lin, Jan. 14, 2002
 Revision 2 - Alla and Kiran, Jan 18, 2002
 Revision 3 - Jernej Barbic and Yili Zhao, Feb, 2012

 */

#ifndef _SKELETON_H
#define _SKELETON_H

#include "posture.h"
#include "transform.h"
// this structure defines the property of each bone segment, including its connection to other bones,
// DOF (degrees of freedom), relative orientation and distance to the outboard bone 
struct Bone {
	struct Bone *sibling; // Pointer to the sibling (branch bone) in the hierarchy tree 
	struct Bone *child; // Pointer to the child (outboard bone) in the hierarchy tree 

	int idx; // Bone index

	double dir[3]; // Unit vector describes the direction from local origin to 
	// the origin of the child bone 
	// Notice: stored in local coordinate system of the bone

	double length; // Bone length  

	double axis_x, axis_y, axis_z; // orientation of each bone's local coordinate 
	//system as specified in ASF file (axis field)

	double aspx, aspy; // aspect ratio of bone shape

	int dof; // number of bone's degrees of freedom 
	int dofrx, dofry, dofrz; // rotational degree of freedom mask in x, y, z axis 
	int doftx, dofty, doftz; // translational degree of freedom mask in x, y, z axis
	int doftl;
	// dofrx=1 if this bone has x rotational degree of freedom, otherwise dofrx=0.

	// bone names
	char name[256];
	// rotation matrix from the local coordinate of this bone to the local coordinate system of it's parent
	double rot_parent_current[4][4];

	//Rotation angles for this bone at a particular time frame (as read from AMC file) in local coordinate system, 
	//they are set in the setPosture function before display function is called
	double rx, ry, rz;
	double tx, ty, tz;
	double tl;
	int dofo[8];
};

class Skeleton {
public:
	// The scale parameter adjusts the size of the skeleton. The default value is 0.06 (MOCAP_SCALE).
	// This creates a human skeleton of 1.7 m in height (approximately)
	Skeleton(char *asf_filename, double scale);
	~Skeleton();

	//Get root node's address; for accessing bone data
	Bone* getRoot();
	static int getRootIndex() {
		return 0;
	}

	//Set the skeleton's pose based on the given posture    
	void setPosture(Posture posture);

	//Initial posture Root at (0,0,0)
	//All bone rotations are set to 0
	void setBasePosture();

	// marks previously unavailable rotational DOFs as available, and sets them to 0
	void enableAllRotationalDOFs();

	int name2idx(char *);
	char * idx2name(int);
	void GetRootPosGlobal(double rootPosGlobal[3]);
	void GetTranslation(double translation[3]);
	void GetRotationAngle(double rotationAngle[3]);
	void SetTranslationX(double tx_) {
		tx = tx_;
	}
	void SetTranslationY(double ty_) {
		ty = ty_;
	}
	void SetTranslationZ(double tz_) {
		tz = tz_;
	}
	void SetRotationAngleX(double rx_) {
		rx = rx_;
	}
	void SetRotationAngleY(double ry_) {
		ry = ry_;
	}
	void SetRotationAngleZ(double rz_) {
		rz = rz_;
	}

	int numBonesInSkel(Bone bone);
	int movBonesInSkel(Bone bone);
	void computeBoneEndPos()
	{
		ComputeRotationToParentCoordSystem(m_pBoneList);
		double identityMat[4][4];
		memset(identityMat,0,sizeof(double)*16);
		for(int i = 0; i < 4; i++)
			identityMat[i][i] = 1;
		Traverse(m_pRootBone, identityMat);
	}

protected:

	//parse the skeleton (.ASF) file	
	int readASFfile(char* asf_filename, double scale);

	//This recursive function traverses skeleton hierarchy 
	//and returns a pointer to the bone with index - bIndex
	//ptr should be a pointer to the root node 
	//when this function first called
	Bone* getBone(Bone *ptr, int bIndex);

	//This function sets sibling or child for parent bone
	//If parent bone does not have a child, 
	//then pChild is set as parent's child
	//else pChild is set as a sibling of parents already existing child
	int setChildrenAndSibling(int parent, Bone *pChild);

	//Rotate all bone's direction vector (dir) from global to local coordinate system
	void RotateBoneDirToLocalCoordSystem();

	void set_bone_shape(Bone *bone);
	void compute_rotation_parent_child(Bone *parent, Bone *child);
	void ComputeRotationToParentCoordSystem(Bone *bone);

	void ProcessBone(Bone *ptr,double transToWorld[4][4],double TransferMatForChild[4][4])
	{
		//Transform (rotate) from the local coordinate system of this bone to it's parent
		//This step corresponds to doing: ModelviewMatrix = M_k * (rot_parent_current)
		matrix_mult( transToWorld ,  ptr->rot_parent_current , TransferMatForChild);

		double temp[4][4];

		//translate AMC
		//if(ptr->doftz)
		//{
		//	translate(temp,0,0,double(ptr->tz));
		//	matrix_multS(TransferMatForChild, temp);
		//}
		//if(ptr->dofty)
		//{
		//	translate(temp,0,double(ptr->ty),0);
		//	matrix_multS(TransferMatForChild, temp);
		//}
		//if(ptr->doftx)
		//{
		//	translate(temp,double(ptr->tx),0,0);
		//	matrix_multS(TransferMatForChild, temp);
		//}

		//rotate AMC (rarely used)
		//if(ptr->dofrz)
		//{
		//	rotationZ(temp, double(ptr->rz));
		//	matrix_multS(TransferMatForChild, temp);
		//}
		//if(ptr->dofry)
		//{
		//	rotationY(temp, double(ptr->ry));
		//	matrix_multS(TransferMatForChild, temp);
		//}
		//if(ptr->dofrx)
		//{
		//	rotationX(temp, double(ptr->rx));
		//	matrix_multS(TransferMatForChild, temp);
		//}

		//Compute tx, ty, tz : translation from pBone to its child (in local coordinate system of pBone)
		double tx = ptr->dir[0] * ptr->length;
		double ty = ptr->dir[1] * ptr->length;
		double tz = ptr->dir[2] * ptr->length;
		translate(temp,tx,ty,tz);
		matrix_multS(TransferMatForChild, temp);

		double rr[3];
		matrix_transform_affine(TransferMatForChild, 0, 0, 0, rr);
		printf("id: %d  %lf %lf %lf\n",ptr->idx,rr[0],rr[1],rr[2]);
	}

	void Traverse(Bone *ptr,double transToWorld[4][4])
	{
		if(ptr != 0)
		{
			double transToWorldForChild[4][4];
			ProcessBone(ptr, transToWorld, transToWorldForChild);
			Traverse(ptr->child, transToWorldForChild);
			Traverse(ptr->sibling, transToWorld);
		}
	}
	// root position in world coordinate system
	double m_RootPos[3];
	double tx, ty, tz;
	double rx, ry, rz;

	int NUM_BONES_IN_ASF_FILE;
	int MOV_BONES_IN_ASF_FILE;

	Bone *m_pRootBone;  // Pointer to the root bone, m_RootBone = &bone[0]
	Bone m_pBoneList[MAX_BONES_IN_ASF_FILE];   // Array with all skeleton bones

	void removeCR(char * str); // removes CR at the end of line
};

#endif


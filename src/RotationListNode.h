#ifndef _RotationListNode
#define _RotationListNode
//
// File: RotationListNode.h
//
// Dependency Graph Node: rotationList
//
// Author: Benjamin H. Singleton
//

#include <utility>
#include <map>
#include <vector>

#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MMatrix.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MEulerRotation.h>
#include <maya/MQuaternion.h>
#include <maya/MAngle.h>
#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include <maya/MMatrixArray.h>
#include <maya/MString.h>
#include <maya/MFloatArray.h>
#include <maya/MFnData.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MTypeId.h> 
#include <maya/MGlobal.h>
#include <math.h>


enum class AxisOrder
{
	
	xyz = 1,
	xzy = 2,
	yzx = 3,
	yxz = 4,
	zxy = 5,
	zyx = 6,
	xyx = 7,
	yzy = 8,
	zxz = 9
	
};


struct RotationListItem
{

	MString	name = "";
	float weight = 1.0;
	bool absolute = false;
	AxisOrder axisOrder = AxisOrder::xyz;
	MVector radians = MVector::zero;

};


class RotationList : public MPxNode
{

public:

						RotationList();
	virtual				~RotationList();

	virtual MStatus		compute(const MPlug& plug, MDataBlock& data);
	static  void*		creator();
	static  MStatus		initialize();
	
	static	MMatrix		average(const std::vector<RotationListItem>& items);
	static	void		normalize(std::vector<RotationListItem>& items);

	static	double		dot(const MQuaternion& quat, const MQuaternion& otherQuat);
	static	MQuaternion	slerp(const MQuaternion& startQuat, const MQuaternion& endQuat, const float weight);
	static	MMatrix		slerp(const MMatrix& startMatrix, const MMatrix& endMatrix, const float weight);

	static	MMatrix		createRotationMatrix(const double x, const double y, const double z, const AxisOrder axisOrder);
	static	MMatrix		createRotationMatrix(const MVector& radians, AxisOrder axisOrder);
	
public:

	static	MObject		active;
	static	MObject		normalizeWeights;
	static	MObject		rotateOrder;
	static	MObject		list;
	static	MObject		name;
	static	MObject		weight;
	static	MObject		absolute;
	static	MObject		axisOrder;
	static	MObject		rotationX;
	static	MObject		rotationY;
	static	MObject		rotationZ;
	static	MObject		rotation;
	
	static	MObject		value;
	static	MObject		valueX;
	static	MObject		valueY;
	static	MObject		valueZ;
	static	MObject		matrix;
	
	static	MTypeId		id;
	static	MString		listCategory;
	static	MString		rotationCategory;
	static	MString		outputCategory;
	
};

#endif
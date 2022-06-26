//
// File: RotationListNode.cpp
//
// Dependency Graph Node: rotationList
//
// Author: Benjamin H. Singleton
//

#include "RotationListNode.h"

MObject		RotationList::active;
MObject		RotationList::normalizeWeights;
MObject		RotationList::rotateOrder;
MObject		RotationList::list;
MObject		RotationList::name;
MObject		RotationList::weight;
MObject		RotationList::absolute;
MObject		RotationList::axisOrder;
MObject		RotationList::rotation;
MObject		RotationList::rotationX;
MObject		RotationList::rotationY;
MObject		RotationList::rotationZ;

MObject		RotationList::value;
MObject		RotationList::valueX;
MObject		RotationList::valueY;
MObject		RotationList::valueZ;
MObject		RotationList::matrix;

MTypeId		RotationList::id(0x0013b1c6);
MString		RotationList::listCategory("List");
MString		RotationList::rotationCategory("Rotation");
MString		RotationList::outputCategory("Output");


RotationList::RotationList() {};
RotationList::~RotationList() {};


MStatus RotationList::compute(const MPlug& plug, MDataBlock& data) 
/**
This method should be overridden in user defined nodes.
Recompute the given output based on the nodes inputs.
The plug represents the data value that needs to be recomputed, and the data block holds the storage for all of the node's attributes.
The MDataBlock will provide smart handles for reading and writing this node's attribute values.
Only these values should be used when performing computations!

@param plug: Plug representing the attribute that needs to be recomputed.
@param data: Data block containing storage for the node's attributes.
@return: Return status.
*/
{
	
	MStatus status;

	// Check requested attribute
	//
	MObject attribute = plug.attribute(&status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnAttribute fnAttribute(attribute, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	if (fnAttribute.hasCategory(RotationList::outputCategory))
	{
		
		// Get input data handles
		//
		MDataHandle activeHandle = data.inputValue(RotationList::active, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle normalizeWeightsHandle = data.inputValue(RotationList::normalizeWeights, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle rotateOrderHandle = data.inputValue(RotationList::rotateOrder, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MArrayDataHandle listHandle = data.inputArrayValue(RotationList::list, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		// Get values from handles
		//
		short active = activeHandle.asShort();
		bool normalizeWeights = normalizeWeightsHandle.asBool();
		short rotateOrder = rotateOrderHandle.asShort();
		
		// Collect position entries
		//
		unsigned int listCount = listHandle.elementCount();
		std::vector<RotationListItem> items = std::vector<RotationListItem>(listCount);

		MDataHandle elementHandle, nameHandle, weightHandle, absoluteHandle, axisOrderHandle, rotationHandle, rotationXHandle, rotationYHandle, rotationZHandle;
		MString name;
		float weight;
		bool absolute;
		AxisOrder axisOrder;
		double rotationX, rotationY, rotationZ;
		
		for (unsigned int i = 0; i < listCount; i++)
		{
			
			// Jump to array element
			//
			status = listHandle.jumpToElement(i);
			CHECK_MSTATUS_AND_RETURN_IT(status)

			elementHandle = listHandle.inputValue(&status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			// Get element data handles
			//
			nameHandle = elementHandle.child(RotationList::name);
			weightHandle = elementHandle.child(RotationList::weight);
			absoluteHandle = elementHandle.child(RotationList::absolute);
			axisOrderHandle = elementHandle.child(RotationList::axisOrder);
			rotationHandle = elementHandle.child(RotationList::rotation);
			rotationXHandle = rotationHandle.child(RotationList::rotationX);
			rotationYHandle = rotationHandle.child(RotationList::rotationY);
			rotationZHandle = rotationHandle.child(RotationList::rotationZ);
			

			// Get values from handles
			//
			name = nameHandle.asString();
			weight = weightHandle.asFloat();
			absolute = absoluteHandle.asBool();
			axisOrder = AxisOrder(axisOrderHandle.asInt());
			rotationX = rotationXHandle.asAngle().asRadians();
			rotationY = rotationYHandle.asAngle().asRadians();
			rotationZ = rotationZHandle.asAngle().asRadians();
			
			// Assign item to array
			//
			items[i] = RotationListItem{ name, weight, absolute, axisOrder, MVector(rotationX, rotationY, rotationZ) };
			
		}
		
		// Check if weights should be normalized
		//
		if (normalizeWeights)
		{

			RotationList::normalize(items);

		}

		// Calculate weighted average
		//
		MMatrix matrix = RotationList::average(items);
		MQuaternion quat = MTransformationMatrix(matrix).rotation();
		
		MEulerRotation eulerRotation = quat.asEulerRotation();
		eulerRotation.reorderIt(MEulerRotation::RotationOrder(rotateOrder));

		// Get output data handles
		//
		MDataHandle valueXHandle = data.outputValue(RotationList::valueX, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle valueYHandle = data.outputValue(RotationList::valueY, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle valueZHandle = data.outputValue(RotationList::valueZ, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle matrixHandle = data.outputValue(RotationList::matrix, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);


		// Set output handle values
		//
		valueXHandle.setMAngle(MAngle(eulerRotation.x, MAngle::kRadians));
		valueXHandle.setClean();
		
		valueYHandle.setMAngle(MAngle(eulerRotation.y, MAngle::kRadians));
		valueYHandle.setClean();
		
		valueZHandle.setMAngle(MAngle(eulerRotation.z, MAngle::kRadians));
		valueZHandle.setClean();

		matrixHandle.setMMatrix(matrix);
		matrixHandle.setClean();

		// Mark plug as clean
		//
		status = data.setClean(plug);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		return MS::kSuccess;

	}
	else
	{
		
		return MS::kUnknownParameter;

	}

};


MMatrix RotationList::average(const std::vector<RotationListItem>& items)
/**
Returns the weighted average of the supplied rotation items.

@param items: The rotation items to average.
@return: Weighted average matrix.
*/
{

	// Evaluate item count
	//
	unsigned long itemCount = items.size();
	MMatrix average = MMatrix(MMatrix::identity);

	if (itemCount == 0)
	{

		return average;

	}

	// Calculate weighted average
	//
	MMatrix rotationMatrix;

	for (RotationListItem item : items)
	{

		// Evaluate which method to use
		//
		rotationMatrix = RotationList::createRotationMatrix(item.radians, item.axisOrder);

		if (item.absolute)
		{

			average = RotationList::slerp(average, rotationMatrix, item.weight);

		}
		else
		{
			
			average = RotationList::slerp(MMatrix::identity, rotationMatrix, item.weight) * average;

		}

	}

	return average;
	
};


double RotationList::dot(const MQuaternion& quat, const MQuaternion& otherQuat)
/**
Returns the dot product of two quaternions.

@param quat: Quaternion.
@param: otherQuat: Other quaternion.
@return: Dot length.
*/
{

	return (quat.x * otherQuat.x) + (quat.y * otherQuat.y) + (quat.z * otherQuat.z) + (quat.w * otherQuat.w);

};


MQuaternion RotationList::slerp(const MQuaternion& startQuat, const MQuaternion& endQuat, const float weight)
/**
Spherical interpolates two quaternions.
See the following for details: https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm

@param startQuat: Start Quaternion.
@param endQuat: End Quaternion.
@param weight: The amount to interpolate.
@return: The interpolated quaternion.
*/
{

	MQuaternion q1 = MQuaternion(startQuat);
	MQuaternion q2 = MQuaternion(endQuat);

	double dot = RotationList::dot(q1, q2);

	if (dot < 0.0)
	{

		dot = RotationList::dot(q1, q2.negateIt());

	}

	double theta = acos(dot);
	double sinTheta = sin(theta);

	double w1, w2;

	if (sinTheta > 1e-3)
	{

		w1 = sin((1.0 - weight) * theta) / sinTheta;
		w2 = sin(weight * theta) / sinTheta;

	}
	else
	{

		w1 = 1.0 - weight;
		w2 = weight;

	}

	q1.scaleIt(w1);
	q2.scaleIt(w2);

	return q1 + q2;

};


MMatrix RotationList::slerp(const MMatrix& startMatrix, const MMatrix& endMatrix, const float weight)
/**
Spherical interpolates two rotation matrices.

@param startQuat: Start matrix.
@param endQuat: End matrix.
@param weight: The amount to interpolate.
@return: The interpolated matrix.
*/
{

	MQuaternion startQuat;
	startQuat = startMatrix;

	MQuaternion endQuat;
	endQuat = endMatrix;

	return RotationList::slerp(startQuat, endQuat, weight).asMatrix();

};


void RotationList::normalize(std::vector<RotationListItem>& items)
/**
Normalizes the passed weights so that the total sum equals 1.0.

@param items: The items to normalize.
@return: void
*/
{

	// Get weight sum
	//
	unsigned long itemCount = items.size();
	float sum = 0.0;

	for (unsigned long i = 0; i < itemCount; i++)
	{

		sum += std::fabs(items[i].weight);

	}

	// Check for divide by zero errors!
	//
	if (sum == 0.0 || sum == 1.0)
	{

		return;

	}

	// Multiply weights by scale factor
	//
	float factor = 1.0 / sum;

	for (unsigned long i = 0; i < itemCount; i++)
	{

		items[i].weight *= factor;

	}

};


MMatrix RotationList::createRotationMatrix(const double x, const double y, const double z, const AxisOrder axisOrder)
/**
Creates a rotation matrix from the supplied angles and axis order.

@param x: The X angle in radians.
@param y: The Y angle in radians.
@param z: The Z angle in radians.
@param axisOrder: The axis order.
@return: The new rotation matrix.
*/
{
	
	// Compose rotation axis matrices
	//
	double rotateX[4][4] = {
		{1.0, 0.0, 0.0, 0.0},
		{0.0, cos(x), sin(x), 0.0},
		{0.0, -sin(x), cos(x), 0.0},
		{0.0, 0.0, 0.0, 1.0}
	};
	
	double rotateY[4][4] = {
		{cos(y), 0.0, -sin(y), 0.0},
		{0.0, 1.0, 0.0, 0.0},
		{sin(y), 0.0, cos(y), 0.0},
		{0.0, 0.0, 0.0, 1.0}
	};
	
	double rotateZ[4][4] = {
		{cos(z), sin(z), 0.0, 0.0},
		{-sin(z), cos(z), 0.0, 0.0},
		{0.0, 0.0, 1.0, 0.0},
		{0.0, 0.0, 0.0, 1.0}
	};
	
	MMatrix rotateXMatrix = MMatrix(rotateX);
	MMatrix rotateYMatrix = MMatrix(rotateY);
	MMatrix rotateZMatrix = MMatrix(rotateZ);
	
	// Compose rotation matrix using axis order
	//
	switch (axisOrder)
	{
		
		case AxisOrder::xyz:
			return rotateXMatrix * rotateYMatrix * rotateZMatrix;
		
		case AxisOrder::xzy:
			return rotateXMatrix * rotateZMatrix * rotateYMatrix;
		
		case AxisOrder::yzx:
			return rotateYMatrix * rotateZMatrix * rotateXMatrix;
		
		case AxisOrder::yxz:
			return rotateYMatrix * rotateXMatrix * rotateZMatrix;
		
		case AxisOrder::zxy:
			return rotateZMatrix * rotateXMatrix * rotateYMatrix;
		
		case AxisOrder::zyx:
			return rotateZMatrix * rotateYMatrix * rotateXMatrix;
		
		case AxisOrder::xyx:
			return rotateXMatrix * rotateYMatrix * rotateXMatrix;
		
		case AxisOrder::yzy:
			return rotateYMatrix * rotateZMatrix * rotateYMatrix;
			
		case AxisOrder::zxz:
			return rotateZMatrix * rotateXMatrix * rotateZMatrix;
		
		default:
			return MMatrix(MMatrix::identity);
		
	}

};


MMatrix RotationList::createRotationMatrix(const MVector& radians, const AxisOrder axisOrder)
/**
Creates a rotation matrix from the supplied angles and axis order.

@param radians: The XYZ values in radians.
@param axisOrder: The axis order.
@return: The new rotation matrix.
*/
{

	return RotationList::createRotationMatrix(radians.x, radians.y, radians.z, axisOrder);

};


void* RotationList::creator() 
/**
This function is called by Maya when a new instance is requested.
See pluginMain.cpp for details.

@return: RotationList
*/
{

	return new RotationList();

};


MStatus RotationList::initialize()
/**
This function is called by Maya after a plugin has been loaded.
Use this function to define any static attributes.

@return: MStatus
*/
{
	
	MStatus status;

	// Initialize function sets
	//
	MFnNumericAttribute fnNumericAttr;
	MFnTypedAttribute fnTypedAttr;
	MFnUnitAttribute fnUnitAttr;
	MFnEnumAttribute fnEnumAttr;
	MFnMatrixAttribute fnMatrixAttr;
	MFnCompoundAttribute fnCompoundAttr;

	// Input attributes:
	// ".active" attribute
	//
	RotationList::active = fnNumericAttr.create("active", "a", MFnNumericData::kInt, 0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// ".normalizeWeights" attribute
	//
	RotationList::normalizeWeights = fnNumericAttr.create("normalizeWeights", "nw", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	// ".rotateOrder" attribute
	//
	RotationList::rotateOrder = fnEnumAttr.create("rotateOrder", "ro", short(0), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnEnumAttr.addField("xyz", 0));
	CHECK_MSTATUS(fnEnumAttr.addField("yzx", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("zxy", 2));
	CHECK_MSTATUS(fnEnumAttr.addField("xzy", 3));
	CHECK_MSTATUS(fnEnumAttr.addField("yxz", 4));
	CHECK_MSTATUS(fnEnumAttr.addField("zyx", 5));
	
	// ".name" attribute
	//
	RotationList::name = fnTypedAttr.create("name", "n", MFnData::kString, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnTypedAttr.addToCategory(RotationList::listCategory));

	// ".weight" attribute
	//
	RotationList::weight = fnNumericAttr.create("weight", "w", MFnNumericData::kFloat, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	CHECK_MSTATUS(fnNumericAttr.setMin(-1.0));
	CHECK_MSTATUS(fnNumericAttr.setMax(1.0));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(RotationList::listCategory));

	// ".absolute" attribute
	//
	RotationList::absolute = fnNumericAttr.create("absolute", "abs", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(RotationList::listCategory));

	// ".axisOrder" attribute
	//
	RotationList::axisOrder = fnEnumAttr.create("axisOrder", "ao", short(1), &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	CHECK_MSTATUS(fnEnumAttr.addField("xyz", 1));
	CHECK_MSTATUS(fnEnumAttr.addField("xzy", 2));
	CHECK_MSTATUS(fnEnumAttr.addField("yzx", 3));
	CHECK_MSTATUS(fnEnumAttr.addField("yxz", 4));
	CHECK_MSTATUS(fnEnumAttr.addField("zxy", 5));
	CHECK_MSTATUS(fnEnumAttr.addField("zyx", 6));
	CHECK_MSTATUS(fnEnumAttr.addField("xyx", 7));
	CHECK_MSTATUS(fnEnumAttr.addField("yzy", 8));
	CHECK_MSTATUS(fnEnumAttr.addField("zxz", 9));
	CHECK_MSTATUS(fnEnumAttr.addToCategory(RotationList::listCategory));

	// ".rotationX" attribute
	//
	RotationList::rotationX = fnUnitAttr.create("rotationX", "rx", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(RotationList::rotationCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(RotationList::listCategory));

	// ".rotationY" attribute
	//
	RotationList::rotationY = fnUnitAttr.create("rotationY", "ry",  MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(RotationList::rotationCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(RotationList::listCategory));

	// ".rotationZ" attribute
	//
	RotationList::rotationZ = fnUnitAttr.create("rotationZ", "rz",  MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.addToCategory(RotationList::rotationCategory));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(RotationList::listCategory));

	// ".rotation" attribute
	//
	RotationList::rotation = fnNumericAttr.create("rotation", "r", RotationList::rotationX, RotationList::rotationY, RotationList::rotationZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.addToCategory(RotationList::rotationCategory));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(RotationList::listCategory));

	// ".list" attribute
	//
	RotationList::list = fnCompoundAttr.create("list", "l", &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	
	CHECK_MSTATUS(fnCompoundAttr.addChild(RotationList::name));
	CHECK_MSTATUS(fnCompoundAttr.addChild(RotationList::weight));
	CHECK_MSTATUS(fnCompoundAttr.addChild(RotationList::absolute));
	CHECK_MSTATUS(fnCompoundAttr.addChild(RotationList::axisOrder));
	CHECK_MSTATUS(fnCompoundAttr.addChild(RotationList::rotation));
	CHECK_MSTATUS(fnCompoundAttr.setArray(true));
	CHECK_MSTATUS(fnCompoundAttr.addToCategory(RotationList::listCategory));

	// Output attributes:
	// ".valueX" attribute
	//
	RotationList::valueX = fnUnitAttr.create("valueX", "vx", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(RotationList::outputCategory));

	// ".valueY" attribute
	//
	RotationList::valueY = fnUnitAttr.create("valueY", "vy", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(RotationList::outputCategory));

	// ".valueZ" attribute
	//
	RotationList::valueZ = fnUnitAttr.create("valueZ", "vz", MFnUnitAttribute::kAngle, 0.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnUnitAttr.setWritable(false));
	CHECK_MSTATUS(fnUnitAttr.setStorable(false));
	CHECK_MSTATUS(fnUnitAttr.addToCategory(RotationList::outputCategory));

	// ".value" attribute
	//
	RotationList::value = fnNumericAttr.create("value", "v", RotationList::valueX, RotationList::valueY, RotationList::valueZ, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnNumericAttr.setWritable(false));
	CHECK_MSTATUS(fnNumericAttr.setStorable(false));
	CHECK_MSTATUS(fnNumericAttr.addToCategory(RotationList::outputCategory));

	// ".matrix" attribute
	//
	RotationList::matrix = fnMatrixAttr.create("matrix", "m", MFnMatrixAttribute::kDouble, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	CHECK_MSTATUS(fnMatrixAttr.setWritable(false));
	CHECK_MSTATUS(fnMatrixAttr.setStorable(false));
	CHECK_MSTATUS(fnMatrixAttr.addToCategory(RotationList::outputCategory));

	// Add attributes to node
	//
	CHECK_MSTATUS(RotationList::addAttribute(RotationList::active));
	CHECK_MSTATUS(RotationList::addAttribute(RotationList::normalizeWeights));
	CHECK_MSTATUS(RotationList::addAttribute(RotationList::rotateOrder));
	CHECK_MSTATUS(RotationList::addAttribute(RotationList::list));

	CHECK_MSTATUS(RotationList::addAttribute(RotationList::value));
	CHECK_MSTATUS(RotationList::addAttribute(RotationList::matrix));

	// Define attribute relationships
	//
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::active, RotationList::valueX));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::normalizeWeights, RotationList::valueX));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotateOrder, RotationList::valueX));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::weight, RotationList::valueX));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::absolute, RotationList::valueX));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::axisOrder, RotationList::valueX));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationX, RotationList::valueX));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationY, RotationList::valueX));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationZ, RotationList::valueX));

	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::active, RotationList::valueY));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::normalizeWeights, RotationList::valueY));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotateOrder, RotationList::valueY));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::weight, RotationList::valueY));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::absolute, RotationList::valueY));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::axisOrder, RotationList::valueY));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationX, RotationList::valueY));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationY, RotationList::valueY));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationZ, RotationList::valueY));

	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::active, RotationList::valueZ));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::normalizeWeights, RotationList::valueZ));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotateOrder, RotationList::valueZ));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::weight, RotationList::valueZ));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::absolute, RotationList::valueZ));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::axisOrder, RotationList::valueZ));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationX, RotationList::valueZ));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationY, RotationList::valueZ));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationZ, RotationList::valueZ));

	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::active, RotationList::matrix));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::normalizeWeights, RotationList::matrix));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotateOrder, RotationList::matrix));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::weight, RotationList::matrix));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::absolute, RotationList::matrix));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::axisOrder, RotationList::matrix));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationX, RotationList::matrix));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationY, RotationList::matrix));
	CHECK_MSTATUS(RotationList::attributeAffects(RotationList::rotationZ, RotationList::matrix));

	return status;

};
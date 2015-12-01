#include "Real2DStateSpace.h"
#include "v_repLib.h"

#define Real2D(x) reinterpret_cast<Real2DStateSpace*>(x)

Real2DStateSpace::Real2DStateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension) :
	StateSpace(rotAxisRot, rotAxisRotInv, dimension) {
}

StateSpace* Real2DStateSpace::copyMyself() {
	Real2DStateSpace* newNode = new Real2DStateSpace(_rotAxisRotInv, _rotAxisRotInv, getSize());
	newNode->_nodeType = _nodeType;
	newNode->setAllValues(_values);

	return newNode;
}

int Real2DStateSpace::getVector(StateSpace* from, StateSpace* to, float vect[7], float e, float& artificialLength) {
	int retVal = -1; 

	vect[0] = Real2D(to)->_values[0] - Real2D(from)->_values[0];
	vect[1] = Real2D(to)->_values[1] - Real2D(from)->_values[1];

	artificialLength = sqrtf(vect[0] * vect[0] + vect[1] * vect[1]);
	retVal = (int)(artificialLength / e) + 1;
	float l = (float)retVal;
	vect[0] /= l;
	vect[1] /= l;

	return retVal;
}

void Real2DStateSpace::reSample(int theType, float searchMin[4], float searchRange[4]) {
	for (int i = 0; i < _dimension; i++) {
		_values[i] = searchMin[i] + searchRange[i] * SIM_RAND_FLOAT;
	}
}

int Real2DStateSpace::getSize() {
	return 2;
}

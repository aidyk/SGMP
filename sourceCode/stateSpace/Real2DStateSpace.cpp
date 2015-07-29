#include "Real2DStateSpace.h"
#include "v_repLib.h"

Real2DStateSpace::Real2DStateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension) :
	StateSpace(rotAxisRot, rotAxisRotInv, dimension) {
}

StateSpace* Real2DStateSpace::copyMyself() {
	Real2DStateSpace* newNode = new Real2DStateSpace(_rotAxisRotInv, _rotAxisRotInv, getSize());
	newNode->_nodeType = _nodeType;
	newNode->setAllValues(_values);

	return newNode;
}

int Real2DStateSpace::getSize() {
	return 2;
}

void Real2DStateSpace::reSample(int theType, float searchMin[4], float searchRange[4]) {
	for (int i = 0; i < _dimension; i++) {
		_values[i] = searchMin[i] + searchRange[i] * SIM_RAND_FLOAT;
	}
}

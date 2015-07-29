#include "Real3DStateSpace.h"

#include "v_repLib.h"

Real3DStateSpace::Real3DStateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension) :
	StateSpace(rotAxisRot, rotAxisRotInv, dimension) {
}

Real3DStateSpace* Real3DStateSpace::copyMyself() {
	Real3DStateSpace* newNode = new Real3DStateSpace(_rotAxisRotInv, _rotAxisRotInv, getSize());
	newNode->_nodeType = _nodeType;

	newNode->setAllValues(_values);

	return newNode;
}

int Real3DStateSpace::getSize() {
	return 3;
}

void Real3DStateSpace::reSample(int theType, float searchMin[4], float searchRange[4]) {
	for (int i = 0; i < _dimension; i++) {
		_values[i] = searchMin[i] + searchRange[i] * SIM_RAND_FLOAT;
	}
}

#include "SE3StateSpace.h"

#include "v_repLib.h"

SE3StateSpace::SE3StateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension) :
	StateSpace(rotAxisRot, rotAxisRotInv, dimension) {
}

SE3StateSpace* SE3StateSpace::copyMyself() {
	SE3StateSpace* newNode = new SE3StateSpace(_rotAxisRotInv, _rotAxisRotInv, getSize());
	newNode->_nodeType = _nodeType;

	newNode->setAllValues(_values);

	return newNode;
}

int SE3StateSpace::getSize() {
	return 6;
}

void SE3StateSpace::reSample(int theType, float searchMin[4], float searchRange[4]) {
	_values[0] = searchMin[0] + searchRange[0] * SIM_RAND_FLOAT;
	_values[1] = searchMin[1] + searchRange[1] * SIM_RAND_FLOAT;
	_values[2] = searchMin[2] + searchRange[2] * SIM_RAND_FLOAT;

	C4Vector q;
	q.buildRandomOrientation();
	for (int i = 0; i < 4; i++) {
		_values[i + 3] = q(i);
	}
}

#include "SE2StateSpace.h"

#include "v_repLib.h"
#include "pathPlanningInterface.h"

SE2StateSpace::SE2StateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension) :
	StateSpace(rotAxisRot, rotAxisRotInv, dimension) {
}

SE2StateSpace* SE2StateSpace::copyMyself() {
	SE2StateSpace* newNode = new SE2StateSpace(_rotAxisRotInv, _rotAxisRotInv, getSize());
	newNode->_nodeType = _nodeType;

	newNode->setAllValues(_values);

	return newNode;
}

int SE2StateSpace::getSize() {
	return 3;
}

void SE2StateSpace::reSample(int theType, float searchMin[4], float searchRange[4]) {
	_values[0] = searchMin[0] + searchRange[0] * SIM_RAND_FLOAT;
	_values[1] = searchMin[1] + searchRange[1] * SIM_RAND_FLOAT;
    _values[2] = CPathPlanningInterface::getNormalizedAngle(searchMin[3] + searchRange[3] * SIM_RAND_FLOAT);
}

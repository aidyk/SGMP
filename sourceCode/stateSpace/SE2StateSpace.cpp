#include "SE2StateSpace.h"

#include "v_repLib.h"
#include "pathPlanningInterface.h"

#define SE2(x) reinterpret_cast<SE2StateSpace*>(x)

SE2StateSpace::SE2StateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension) :
  StateSpace(rotAxisRot, rotAxisRotInv, dimension) {
}

SE2StateSpace* SE2StateSpace::copyMyself() {
  SE2StateSpace* newNode = new SE2StateSpace(_rotAxisRotInv, _rotAxisRotInv, getSize());
  newNode->_nodeType = _nodeType;

  newNode->setAllValues(_values);

  return newNode;
}

int SE2StateSpace::getVector(StateSpace* from, StateSpace* to, float vect[7], float e, float& artificialLength) {
  int retVal = -1;

  vect[0] = SE2(to)->_values[0] - SE2(from)->_values[0];
  vect[1] = SE2(to)->_values[1] - SE2(from)->_values[1];
  vect[2] = CPathPlanningInterface::getNormalizedAngle(SE2(to)->_values[2] - SE2(from)->_values[2]);

  artificialLength = vect[0] * vect[0] + vect[1] * vect[1];
  artificialLength = sqrtf(artificialLength + vect[2] * _angularCoeff * vect[2] * _angularCoeff);
  retVal = (int)(artificialLength / e) + 1;
  float l = (float)retVal;
  vect[0] /= l;
  vect[1] /= l;
  vect[2] /= l;
}

void SE2StateSpace::reSample(int theType, float searchMin[4], float searchRange[4]) {
  _values[0] = searchMin[0] + searchRange[0] * SIM_RAND_FLOAT;
  _values[1] = searchMin[1] + searchRange[1] * SIM_RAND_FLOAT;
  _values[2] = CPathPlanningInterface::getNormalizedAngle(searchMin[3] + searchRange[3] * SIM_RAND_FLOAT);
}

int SE2StateSpace::getSize() {
  return 3;
}

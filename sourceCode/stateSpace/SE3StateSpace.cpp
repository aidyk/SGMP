#include "SE3StateSpace.h"

#include "v_repLib.h"

#define SE3(x) reinterpret_cast<SE3StateSpace*>(x)

SE3StateSpace::SE3StateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension) :
	StateSpace(rotAxisRot, rotAxisRotInv, dimension) {
}

SE3StateSpace* SE3StateSpace::copyMyself() {
	SE3StateSpace* newNode = new SE3StateSpace(_rotAxisRotInv, _rotAxisRotInv, getSize());
	newNode->_nodeType = _nodeType;

	newNode->setAllValues(_values);

	return newNode;
}

int SE3StateSpace::getVector(StateSpace* from, StateSpace* to, float vect[7], float e, float& artificialLength) {
  int retVal = -1;

  vect[0] = SE3(to)->_values[0] - SE3(from)->_values[0];
  vect[1] = SE3(to)->_values[1] - SE3(from)->_values[1];
  vect[2] = SE3(to)->_values[2] - SE3(from)->_values[2];

  C4Vector toP, fromP;

  from->getOrientation(fromP);
  to->getOrientation(toP);
  C4Vector diff(fromP.getInverse() * toP);
  for (int i = 0; i < 4; i++) {
    vect[i + 3] = diff(i);
  }

  float ap = _angularCoeff * fromP.getAngleBetweenQuaternions(toP);
  artificialLength = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2];
  artificialLength = sqrtf(artificialLength + ap * ap);
  retVal = (int)(artificialLength / e) + 1;
  float l = (float)retVal;
  for (int i = 0; i < 3; i++)
    vect[i] /= l;

  C4Vector q;
  q.setIdentity();
  fromP.buildInterpolation(q, diff, 1.0f / l);
  for (int i = 0; i < 4; i++)
    vect[i + 3] = fromP(i);
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

int SE3StateSpace::getSize() {
    return 6;
}

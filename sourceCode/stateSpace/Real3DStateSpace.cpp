#include "Real3DStateSpace.h"

#include "v_repLib.h"

#define Real3D(x) reinterpret_cast<Real3DStateSpace*>(x)

Real3DStateSpace::Real3DStateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension) :
  StateSpace(rotAxisRot, rotAxisRotInv, dimension) {
}

Real3DStateSpace* Real3DStateSpace::copyMyself() {
  Real3DStateSpace* newNode = new Real3DStateSpace(_rotAxisRotInv, _rotAxisRotInv, getSize());
  newNode->_nodeType = _nodeType;

  newNode->setAllValues(_values);

  return newNode;
}

void Real3DStateSpace::reSample(int theType, float searchMin[4], float searchRange[4]) {
  for (int i = 0; i < _dimension; i++) {
    _values[i] = searchMin[i] + searchRange[i] * SIM_RAND_FLOAT;
  }
}

int Real3DStateSpace::getVector(StateSpace* from, StateSpace* to, float vect[7], float e, float& artificialLength) {
  int retVal = -1;

  vect[0] = Real3D(to)->_values[0] - Real3D(from)->_values[0];
  vect[1] = Real3D(to)->_values[1] - Real3D(from)->_values[1];
  vect[2] = Real3D(to)->_values[2] - Real3D(from)->_values[2];
  artificialLength = sqrtf(vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2]);
  retVal = (int)(artificialLength / e) + 1;
  float l = (float)retVal;
  vect[0] /= l;
  vect[1] /= l;
  vect[2] /= l;

  return retVal;
}

int Real3DStateSpace::getSize() {
  return 3;
}

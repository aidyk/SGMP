#include "stateSpace.h"

StateSpace::StateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension) {
	_rotAxisRot = rotAxisRot;
	_rotAxisRotInv = rotAxisRotInv;
	_nodeType = -1; 
	_values = new float[dimension];
}

StateSpace::~StateSpace() {
	delete[] _values;
}

void StateSpace::getAllValues(float *v) {
	int size = getSize();
	for (int i = 0; i < size; i++) {
		v[i] = _values[i];
	}
}

void StateSpace::setAllValues(float *v) {
	int size = getSize();
	for (int i = 0; i < size; i++) {
		_values[i] = v[i];
	}
}

void StateSpace::getOrientation(C4Vector& q) {
  assert(getSize() >= 7);

  for (int i = 0; i < 4; i++) {
    q(i) = _values[i + getSize() - 4];
  }
}

void StateSpace::setOrientation(C4Vector* q) {
  assert(getSize() >= 7);

  for (int i = 0; i < 4; i++) {
    _values[i + getSize() - 4] = (*q)(i);
  }
}

#ifndef PATH_PLANNING_Real2D_STATE_SPACE_H_
#define PATH_PLANNING_Real2D_STATE_SPACE_H_

#include "3Vector.h"
#include "4Vector.h"
#include "7Vector.h"
#include "stateSpace.h"

class Real2DStateSpace : public StateSpace {
	public:
		Real2DStateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension);
		virtual StateSpace* copyMyself();
		virtual int getSize();

		virtual void reSample(int theType, float searchMin[4], float searchRange[4]);
};

#endif

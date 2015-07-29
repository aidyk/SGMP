#ifndef PATH_PLANNING_Real3D_STATE_SPACE_H_
#define PATH_PLANNING_Real3D_STATE_SPACE_H_

#include "3Vector.h"
#include "4Vector.h"
#include "7Vector.h"
#include "stateSpace.h"

class Real3DStateSpace : public StateSpace {
	public:
		Real3DStateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension);
		virtual Real3DStateSpace* copyMyself();
		virtual int getSize();

		virtual void reSample(int theType, float searchMin[4], float searchRange[4]);
};

#endif

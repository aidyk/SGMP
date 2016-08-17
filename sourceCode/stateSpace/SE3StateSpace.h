#ifndef PATH_PLANNING_SE3_STATE_SPACE_H_
#define PATH_PLANNING_SE3_STATE_SPACE_H_

#include "3Vector.h"
#include "4Vector.h"
#include "7Vector.h"
#include "stateSpace.h"

class SE3StateSpace : public StateSpace {
	public:
        SE3StateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension);
        virtual SE3StateSpace* copyMyself();
		virtual int getSize();
        virtual int getVector(StateSpace* from, StateSpace* to, float vect[7], float e, float& artificialLength);

        virtual void reSample(int theType, float searchMin[4], float searchRange[4]);
};

#endif

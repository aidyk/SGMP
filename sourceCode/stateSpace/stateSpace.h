#ifndef PATH_PLANNING_STATE_SPACE_H_
#define PATH_PLANNING_STATE_SPACE_H_

#include "3Vector.h"
#include "4Vector.h"
#include "7Vector.h"

class StateSpace {
	public:
		StateSpace(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv, int dimension);
		virtual ~StateSpace();
	
		virtual StateSpace* copyMyself() = 0;
		virtual void reSample(int theType, float searchMin[4], float searchRange[4]) = 0;
		virtual int getVector(StateSpace* from, StateSpace* to, float vect[7], float e, float& artificialLength) = 0;
		virtual int getSize() = 0;

		virtual void getAllValues(float* v);
		virtual void setAllValues(float* v);

        virtual void getOrientation(C4Vector& q); // Quaternion
        virtual void setOrientation(C4Vector* q);

	protected:  
		C4Vector _rotAxisRot; 
		C4Vector _rotAxisRotInv;
        float _angularCoeff;

		int _nodeType;
		int _dimension;
		float* _values;
};

#endif

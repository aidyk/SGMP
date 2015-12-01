// This file is part of the PATH PLANNING PLUGIN for V-REP
// 
// Copyright 2006-2014 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The PATH PLANNING PLUGIN is licensed under the terms of EITHER:
//   1. PATH PLANNING PLUGIN commercial license (contact us for details)
//   2. PATH PLANNING PLUGIN educational license (see below)
// 
// PATH PLANNING PLUGIN educational license:
// -------------------------------------------------------------------
// The PATH PLANNING PLUGIN educational license applies only to EDUCATIONAL
// ENTITIES composed by following people and institutions:
// 
// 1. Hobbyists, students, teachers and professors
// 2. Schools and universities
// 
// EDUCATIONAL ENTITIES do NOT include companies, research institutions,
// non-profit organisations, foundations, etc.
// 
// An EDUCATIONAL ENTITY may use, modify, compile and distribute the
// modified/unmodified PATH PLANNING PLUGIN under following conditions:
//  
// 1. Distribution should be free of charge.
// 2. Distribution should be to EDUCATIONAL ENTITIES only.
// 3. Usage should be non-commercial.
// 4. Altered source versions must be plainly marked as such and distributed
//    along with any compiled code.
// 5. When using the PATH PLANNING PLUGIN in conjunction with V-REP, the "EDU"
//    watermark in the V-REP scene view should not be removed.
// 6. The origin of the PATH PLANNING PLUGIN must not be misrepresented. you must
//    not claim that you wrote the original software.
// 
// THE PATH PLANNING PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR
// IMPLIED WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.1.3 on Sept. 30th 2014

#pragma once

#include "PathPlanning.h"
#include "HolonomicPathNode.h"
#include "dummyClasses.h"
#include <vector>
#include "4Vector.h"
#include "7Vector.h"

class CHolonomicPathPlanning : public CPathPlanning  
{
public:
	CHolonomicPathPlanning() {};
	CHolonomicPathPlanning(int theStartDummyID,int theGoalDummyID,
							int theRobotCollectionID,int theObstacleCollectionID,int ikGroupID,
							int thePlanningType,float theAngularCoeff,
							float theStepSize,
							const float theSearchMinVal[4],const float theSearchRange[4],
							const int theDirectionConstraints[4],const float clearanceAndMaxDistance[2],const C3Vector& gammaAxis);
	virtual ~CHolonomicPathPlanning();

	// Following functions are inherited from CPathPlanning:
	virtual int searchPath(int maxTimePerPass);
    virtual void setGoalBias(float value);
    virtual void setMaxDistance(float value);
	virtual bool setPartialPath();
	virtual int smoothFoundPath(int steps,int maxTimePerPass);
	virtual void getPathData(std::vector<float>& data);

	virtual void getSearchTreeData(std::vector<float>& data,bool fromStart);

	virtual void setAngularCoefficient(float coeff);
	virtual void setStepSize(float size);

	std::vector<CHolonomicPathNode*> fromStart;
	std::vector<CHolonomicPathNode*> fromGoal;
	std::vector<CHolonomicPathNode*> foundPath;

private:
	virtual bool doCollide(float* dist);

	virtual bool addVector(C3Vector& pos,C4Vector& orient,float vect[7]);
	virtual bool areDirectionConstraintsRespected(float vect[7]);
	virtual bool areSomeValuesForbidden(float values[7]);

	C4Vector _gammaAxisRotation;
	C4Vector _gammaAxisRotationInv;

	C7Vector _startDummyCTM;
	C7Vector _startDummyLTM;

	std::vector<int> foundPathSameStraightLineID_forSteppedSmoothing;

    // Adjustable parameters -
    // Randomized approach can be applied, for the details refer to the following
    // paper; Completely randomized RRT-connect proposed by D Schneider, ICRA2015
    float _goalBias;
    float _maxDistance;

    CHolonomicPathPlanning* ptrPlanner;
    CHolonomicPathPlanning* (*constructor_table[10]) (int theStartDummyID, int theGoalDummyID,
                                                      int theRobotCollectionID, int theObstacleCollectionID, int ikGroupID,
                                                      int thePlanningType, float theAngularCoeff,
                                                      float theStepSize,
                                                      const float theSearchMinVal[4], const float theSearchRange[4],
    const int theDirectionConstraints[4], const float clearanceAndMaxDistance[2], const C3Vector& gammaAxis);
};

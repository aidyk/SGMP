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
#include "NonHolonomicPathNode.h"
#include "dummyClasses.h"
#include <vector>
#include "7Vector.h"

class CNonHolonomicPathPlanning : public CPathPlanning  
{
public:
	CNonHolonomicPathPlanning(int theStartDummyID,int theGoalDummyID,
							int theRobotCollectionID,int theObstacleCollectionID,int ikGroupID,float theAngularCoeff,
							float theSteeringAngleCoeff,float theMaxSteeringAngleVariation,float theMaxSteeringAngle,
							float theStepSize,const float theSearchMinVal[2],const float theSearchRange[2],
							const int theDirectionConstraints[2],const float clearanceAndMaxDistance[2]);
	virtual ~CNonHolonomicPathPlanning();

	// Following functions are inherited from CPathPlanning:
	int searchPath(int maxTimePerPass);
	bool setPartialPath();
	int smoothFoundPath(int steps,int maxTimePerPass);
	void getPathData(std::vector<float>& data);
	void getSearchTreeData(std::vector<float>& data,bool fromStart);

	void setStepSize(float size);

	std::vector<CNonHolonomicPathNode*> fromStart;
	std::vector<CNonHolonomicPathNode*> fromGoal;
	std::vector<CNonHolonomicPathNode*> foundPath;

private:
	bool doCollide(float* dist);

	CNonHolonomicPathNode* getClosestNode(std::vector<CNonHolonomicPathNode*>& nodes,CNonHolonomicPathNode* sample,bool forward,bool forConnection);
	CNonHolonomicPathNode* extend(std::vector<CNonHolonomicPathNode*>* currentList,CNonHolonomicPathNode* toBeExtended,CNonHolonomicPathNode* extention,bool forward,CDummyDummy* startDummy);
	CNonHolonomicPathNode* connect(std::vector<CNonHolonomicPathNode*>* currentList,std::vector<CNonHolonomicPathNode*>* nextList,CNonHolonomicPathNode* toBeExtended,CNonHolonomicPathNode* extention,bool forward,bool connect,bool test,CDummyDummy* startDummy);

	int _startDummyID;
	float angularCoeff;
	float steeringAngleCoeff;
	float maxSteeringAngleVariation;
	float maxSteeringAngle;
	float minTurningRadius;
	float stepSize;
	int DoF;
	float searchMinVal[2];
	float searchRange[2];
	C7Vector _startDummyCTM;
	C7Vector _startDummyLTM;

	int numberOfRandomConnectionTries_forSteppedSmoothing;
	int numberOfRandomConnectionTriesLeft_forSteppedSmoothing;
	std::vector<int> foundPathSameStraightLineID_forSteppedSmoothing;
	int sameStraightLineNextID_forSteppedSmoothing;
	int nextIteration_forSteppedSmoothing;




	float _startConfInterferenceState;

	int directionConstraints[2]; // WRONG!!! NOT USED NOW!!! 0 is for vehicle direction, 1 is for steering direction
};

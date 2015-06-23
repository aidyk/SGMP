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

#include <vector>
#include "mpPhase1Node.h"
#include "mpPhase2Node.h"
#include "4X4Matrix.h"
#include "dummyClasses.h"

class CmpObject
{
public:
	CmpObject();
	virtual ~CmpObject();

	void setData(int jointCnt,const int* jointHandles,const int* jointStepCount,int ikGroupId,int baseFrameId,int tipFrameId,int robotSelfCollEntity1,int robotSelfCollEntity2,int robotEntity,int obstacleEntity,float distanceThreshold,const float* tipMetricAndIkElConstraints,const float* robotMetric);

	CmpObject* copyYourself(int jointCount);
	char calculateNodes(const char* serializationData,int serializationDataSize); // make sure setTempData was caller beforehand!
	int getPhase1NodesRenderData(int index,float** pos);
// KEEP!!	int getPhase1ConnectionData(int index,int cIndex,float** pos);
	int getPhase2NodesRenderData(unsigned char what,int index,float** pos1,float** pos2);

	char* getSerializationData(int cnt[1]);
	int getPhase1NodeCnt(char collidingNodesOnly);
//	void setSerializationData(const char* dat,int cnt);
//	void setSerializationData(const char* dat,int cnt,int jointCount);

	int getRobotConfigFromTipPose(const float* tipPos,const float* tipQuat,int options,float* robotJoints,const float* constraints,const float* configConstraints,int trialCount,float tipCloseDistance,const float* referenceConfigs,int configCount,const int* jointBehaviour,int correctionPasses,int maxTimeInMs);

	float* findPath(const float* startConfig,const float* goalConfig,int options,float stepSize,int* outputConfigsCnt,int maxTimeInMs,const int* auxIntParams,const float* auxFloatParams);
	float* findIkPath(const float* startConfig,const float* goalPos,const float* goalQuat,int options,float stepSize,int* outputConfigsCnt,const int* auxIntParams,const float* auxFloatParams);
	float* simplifyPath(const float* pathBuffer,int configCnt,int options,float stepSize,int increment,int* outputConfigsCnt,int maxTimeInMs,const int* auxIntParams,const float* auxFloatParams);
	float* getConfigTransition(const float* startConfig,const float* goalConfig,int options,const int* select,float calcStepSize,float maxOutStepSize,int wayPointCnt,const float* wayPoints,int* outputConfigsCnt,const int* auxIntParams,const float* auxFloatParams);


protected:
	float _getWorkSpaceDistance(const C7Vector& tr1_relToBase,C7Vector tr2_relToBase,bool ignoreOrientationComponents);
	void _prepareNodes(int loopIndex,int* loopsProgress,int* loopNeighbourStepSize,float* jointPositions,const float* jointRangesSpecial,const C7Vector& baseInverse,const CDummy3DObject* tipObject);
	float _getErrorSquared(const C4X4Matrix& m1,const C4X4Matrix& m2,const float constraints[5]);
	float _getConfigErrorSquared(const float* config1,const float* config2,const float* configConstraints);
	bool _performIkThenCheckForCollision(const float* initialJointPositions,float* finalJointPositions,const C7Vector& targetNewLocal,const float* desiredJointPositions,const int* jointBehaviour,int correctionPasses,int collisionCheckMask);
	bool _performIkThenCheckForCollision__(const float* initialJointPositions,float* finalJointPositions,const C7Vector& targetNewLocal,int collisionCheckMask);
	void _clearAllPhase1Nodes();

	C7Vector _getInterpolatedTransformationOnCurve(const std::vector<C7Vector>& wayPts,const std::vector<float>& wayPtsRelDists,float t);

	CmpPhase2Node* _extendNode(std::vector<CmpPhase2Node*>& nodeList,CmpPhase2Node* toBeExtended,CmpPhase2Node* goalNode,float stepSize,bool connect,int collisionCheckMask);
	bool _applyJointPosToRobotAndCheckForCollisions_phase2(const float* jointValues,C7Vector& tipTransf,int collisionCheckMask);
	int _getPhase2Vector(const CmpPhase2Node* startNode,const CmpPhase2Node* goalNode,float stepSize,float* returnVector);
	int _getPhase2Vector(const float* startConfig,const float* goalConfig,float stepSize,float* returnVector);
	void _clearAllPhase2Nodes();
	CmpPhase2Node* _getPhase2ClosestNode(const std::vector<CmpPhase2Node*>& nodeList,const CmpPhase2Node* aNode);
	bool _simplifyFoundPath(const int* auxIntParams,const float* auxFloatParams,int incrementStep,float stepSize,bool activityToConsole,int collisionCheckMask,int maxTimeInMs);
	bool _simplifyFoundPath_pass(int incrementStep,float stepSize,bool activityToConsole,int collisionCheckMask,int maxTimeInMs);
	C7Vector _getTipTranformationFromPhase2Node(const CmpPhase2Node& node);


	std::vector<CmpPhase1Node*> _allNodes;

	const char* _serializationData;
	int _serializationDataSize;

	// following is not serialized:
	std::vector<int> _jointHandles;
	std::vector<CDummyJoint*> _jointObjects;
	CDummy3DObject* _tipObject;
	CDummyDummy* _tipDummy;
	CDummyDummy* _targetDummy;
	C7Vector _baseFrameTransformation;
	C7Vector _baseFrameTransformationInverse;
	std::vector<int> _jointStepCount;
	std::vector<char> _cyclicJointFlags;
	std::vector<float> _jointMinVals;
	std::vector<float> _jointRanges;
	int _ikGroupId;
	CDummyIkGroup* _ikGroup;
	int _baseFrameId;
	int _tipFrameId;
	int _robotSelfCollEntity1;
	int _robotSelfCollEntity2;
	int _robotEntity;
	int _obstacleEntity;
	float _distanceThreshold;
	int _ikElementConstraints;
	float _tipMetric[4];
	std::vector<float> _robotMetric;
	std::vector<int> _closeNodesIndices;

	std::vector<CmpPhase2Node*> _nodeListFromStart;
	std::vector<CmpPhase2Node*> _nodeListFromGoal;
	std::vector<CmpPhase2Node*> _nodeListFoundPath;
};

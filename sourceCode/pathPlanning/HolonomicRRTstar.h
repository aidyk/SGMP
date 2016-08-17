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

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsLinear.h>

#include "HolonomicPathPlanning.h"
#include "HolonomicRRTstarNode.h"
#include "dummyClasses.h"
#include <vector>
#include "4Vector.h"
#include "7Vector.h"

class HolonomicRRTstar : public CHolonomicPathPlanning
{
public:
  HolonomicRRTstar(int theStartDummyID,int theGoalDummyID,
                   int theRobotCollectionID,int theObstacleCollectionID,int ikGroupID,
                   int thePlanningType,float theAngularCoeff,
                   float theStepSize,
                   const float theSearchMinVal[4],const float theSearchRange[4],
  const int theDirectionConstraints[4],const float clearanceAndMaxDistance[2],const C3Vector& gammaAxis);
  virtual ~HolonomicRRTstar();

  // Following functions are inherited from CPathPlanning:
  int searchPath(int maxTimePerPass);
  bool setPartialPath();
  int smoothFoundPath(int steps,int maxTimePerPass);
  void getPathData(std::vector<float>& data);
  void getSearchTreeData(std::vector<float>& data,bool fromStart);

  void setMaxDistance(float distance) { _maxDistance = distance; }
  void setAngularCoefficient(float coeff) { angularCoeff = coeff; }
  void setStepSize(float size) { stepSize = size; }

  float getNearNeighborRadius(void);

  // <Set/Getters
  void setBallRadiusConst(float ballRadiusConst) { _ballRadiusConst = ballRadiusConst; }
  float getBallRadiusConst() { return _ballRadiusConst; }
  // >

private:
  bool doCollide(float* dist);

  bool addVector(C3Vector& pos,C4Vector& orient,float vect[7]);
  bool areDirectionConstraintsRespected(float vect[7]);
  bool areSomeValuesForbidden(float values[7]);
  bool areSomeValuesForbidden(C7Vector configuration);

  float distance(HolonomicRRTstarNode* a, HolonomicRRTstarNode* b);
  int getVector(HolonomicRRTstarNode* fromPoint,HolonomicRRTstarNode* toPoint,float vect[7],float e,float& artificialLength,bool dontDivide);
  std::vector<HolonomicRRTstarNode*> getNearNeighborNodes(std::vector<HolonomicRRTstarNode*>& nodes, HolonomicRRTstarNode* node, float radius);
  HolonomicRRTstarNode* extend(HolonomicRRTstarNode* from, HolonomicRRTstarNode* to,
                               bool shouldBeConnected, CDummyDummy* dummy, float &artificialCost);
  HolonomicRRTstarNode* slerp(HolonomicRRTstarNode*, HolonomicRRTstarNode*, float t);

  int startDummyID;
  int goalDummyID;
  int planningType;
  float angularCoeff;
  float stepSize;
  float _searchMinVal[4];
  float _searchRange[4];
  int _directionConstraints[4];
  bool _directionConstraintsOn;
  float _kConstant;

  C4Vector _gammaAxisRotation;
  C4Vector _gammaAxisRotationInv;

  C7Vector _startDummyCTM;
  C7Vector _startDummyLTM;

  std::vector<int> foundPathSameStraightLineID_forSteppedSmoothing;
  std::vector<double> _collisionCache;
  // boost::shared_ptr<ompl::NearestNeighborsLinear<HolonomicRRTstarNode*> > _nn;
  boost::shared_ptr<ompl::NearestNeighborsGNAT<HolonomicRRTstarNode*> > _nn;

  // <For evaluation
  int _collision_detection_count;
  int _collision_detection_time;
  int _near_neighbor_search_time;
  // >

  // <RRTstar variables
  float _ballRadiusConst;
  float _ballRadiusMax;
  float _goalBias;
  float _maxDistance;
  // >

  int numberOfRandomConnectionTries_forSteppedSmoothing;
  int numberOfRandomConnectionTriesLeft_forSteppedSmoothing;
  int sameStraightLineNextID_forSteppedSmoothing;
  int nextIteration_forSteppedSmoothing;
};

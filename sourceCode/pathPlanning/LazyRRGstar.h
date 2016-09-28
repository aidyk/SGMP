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

#ifndef LAZY_RRGSTAR_H_
#define LAZY_RRGSTAR_H_

#include <vector>

#include "HolonomicPathPlanning.h"
#include "LazyRRGstarNode.h"
#include "dummyClasses.h"
#include "4Vector.h"
#include "7Vector.h"

// using namespace std;

class LazyRRGstar : public CHolonomicPathPlanning
{
public:
  LazyRRGstar(int theStartDummyID,int theGoalDummyID,
                   int theRobotCollectionID,int theObstacleCollectionID,int ikGroupID,
                   int thePlanningType,float theAngularCoeff,
                   float theStepSize,
                   const float theSearchMinVal[4],const float theSearchRange[4],
  const int theDirectionConstraints[4],const float clearanceAndMaxDistance[2],const C3Vector& gammaAxis);
  virtual ~LazyRRGstar();

  // Following functions are inherited from CPathPlanning:
  int searchPath(int maxTimePerPass);
  void getPathData(std::vector<float> &data);
  bool setPartialPath();
  void getSearchTreeData(std::vector<float>& data, bool fromStart);

	float getNearNeighborK(void);
	float getNearNeighborRadius(void);

  // <Set/Getters
  void setGoalBias(float goalBias) { _goalBias = goalBias; }
  float getGoalBias(void) { return _goalBias; }
  void setMaxDistance(float maxDistance) { _maxDistance = maxDistance; }
  float getMaxDistance(void) { return _maxDistance; }
  // >

  // Return the length of best solution path and its actual sequence of configurations
  // as vector form.
  float getBestSolutionPath(LazyRRGstarNode* goal_node);

private:
  float distance(LazyRRGstarNode* a, LazyRRGstarNode* b);
  // int getVector(LazyRRGstarNode* fromPoint,LazyRRGstarNode* toPoint,float vect[7],float e,float& artificialLength,bool dontDivide);
  std::vector<LazyRRGstarNode*> getNearNeighborNodes(std::vector<LazyRRGstarNode*>& nodes, LazyRRGstarNode* node, float radius);
  LazyRRGstarNode* extend(LazyRRGstarNode* from, LazyRRGstarNode* to,
                      bool shouldBeConnected, CDummyDummy* dummy, float &artificialCost);
  LazyRRGstarNode* lazyExtend(LazyRRGstarNode* from, LazyRRGstarNode* to,
                      bool shouldBeConnected, CDummyDummy* dummy, float &artificialCost);
  LazyRRGstarNode* slerp(LazyRRGstarNode*, LazyRRGstarNode*, float t);
  bool isFree(LazyRRGstarNode*, CDummyDummy *dummy);
	bool shouldBeLazy(LazyRRGstarNode* from, LazyRRGstarNode* to);

  void DynamicShortestPathUpdate(CDummyDummy* startDummy);
  void DynamicDecrease(LazyRRGstarNode *node);
  void DynamicIncrease(LazyRRGstarNode *from, LazyRRGstarNode *to);
  void DynamicDelete(LazyRRGstarNode *from, LazyRRGstarNode *to);

  bool gotPotential(LazyRRGstarNode* it);

	std::shared_ptr<ompl::NearestNeighbors<LazyRRGstarNode*> > _nn;

	// <RRG controllable parameters
	float _kConstant;
	float _rConstant;
	float _ballRadiusMax;
  float _goalBias;
  float _maxDistance;
  // >
  LazyRRGstarNode* _start_node;
  LazyRRGstarNode* _goal_node;
  float _best_cost;
	float _dimension;
	std::vector<int> _nn_cache;
	std::vector<int> _depth_table;
	std::vector<LazyRRGstarNode*> _witnesses;

  // <For evaluation
  int _skipped_collision_detection_count;
  int _dynamic_increase_count;
  int _dynamic_decrease_count;

  int _remove_time;
  int _collision_detection_time; // in ms
  int _dynamic_increase_time;
  int _dynamic_decrease_time;
  int _near_neighbor_search_time;
  // >
};
#endif //LAZY_RRGSTAR_H_

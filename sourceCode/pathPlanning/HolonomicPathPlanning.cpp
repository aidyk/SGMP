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

#include "HolonomicRRT.h"
#include "HolonomicBiRRT.h"
#include "HolonomicRRTstar.h"
#include "LazyRRGstar.h"
#include "RRGstar.h"
#include "HolonomicPathPlanning.h"
#include "pathPlanningInterface.h"
#include "v_repLib.h"

#define SIM_MIN(a,b) (((a)<(b)) ? (a) : (b))
#define SIM_MAX(a,b) (((a)>(b)) ? (a) : (b))

CHolonomicPathPlanning::CHolonomicPathPlanning(int theStartDummyID, int theGoalDummyID,
    int theRobotCollectionID, int theObstacleCollectionID, int ikGroupID,
    int thePlanningType, float theAngularCoeff,
    float theStepSize,
    const float theSearchMinVal[4], const float theSearchRange[4],
    const int theDirectionConstraints[4], const float clearanceAndMaxDistance[2], const C3Vector& gammaAxis) {
  FILE *aux_fp = NULL;

  aux_fp = fopen("setting.cfg", "r");
  if (aux_fp == NULL) {
    fprintf(stderr, "Failed to load a configuration file, check the 'setting.cfg' file and its location.");
    return;
  } else {
    fprintf(stderr, "setting.cfg successfully loaded.");
  }

  char planner_type[16], option_type[32];
  float option_value;
  // constructor_table[0] =
  // constructor_table[1]

  fscanf(aux_fp, "%s", planner_type);
  if (!strcmp(planner_type, "RRT")) { // Naive RRT
    HolonomicRRT* rrt = new HolonomicRRT(theStartDummyID, theGoalDummyID,
                                         theRobotCollectionID, theObstacleCollectionID, ikGroupID,
                                         thePlanningType, theAngularCoeff,
                                         theStepSize,
                                         theSearchMinVal, theSearchRange,
                                         theDirectionConstraints, clearanceAndMaxDistance, gammaAxis);
    ptrPlanner = rrt;
    printf("\n%s\n", "HolonomicRRT loaded");
  } else if (!strcmp(planner_type, "BiRRT")) { // BiRRT(Bi-directional RRT or RRT-connect)
    HolonomicBiRRT* birrt = new HolonomicBiRRT(theStartDummyID, theGoalDummyID,
                                               theRobotCollectionID, theObstacleCollectionID, ikGroupID,
                                               thePlanningType, theAngularCoeff,
                                               theStepSize,
                                               theSearchMinVal, theSearchRange,
                                               theDirectionConstraints, clearanceAndMaxDistance, gammaAxis);
/*
    while (fscanf(aux_fp, "%s %f", option_type, &option_value) != -1) {
      fprintf(stderr, "Following options are ignored, %s : %f\n", option_type, option_value);
    }
*/
    ptrPlanner = birrt;
    printf("\n%s\n", "HolonomicBiRRT loaded");
  } else if (!strcmp(planner_type, "RRT*")) { // RRT*
    HolonomicRRTstar* rrt_star = new HolonomicRRTstar(theStartDummyID, theGoalDummyID,
                                                      theRobotCollectionID, theObstacleCollectionID, ikGroupID,
                                                      thePlanningType, theAngularCoeff,
                                                      theStepSize,
                                                      theSearchMinVal, theSearchRange,
                                                      theDirectionConstraints, clearanceAndMaxDistance, gammaAxis);
    ptrPlanner = rrt_star;
    printf("\n%s\n", "HolonomicRRT* loaded");
  } else if (!strcmp(planner_type, "RRG*")) { // RRG*
    RRGstar* rrg_star = new RRGstar(theStartDummyID, theGoalDummyID,
                                    theRobotCollectionID, theObstacleCollectionID, ikGroupID,
                                    thePlanningType, theAngularCoeff,
                                    theStepSize,
                                    theSearchMinVal, theSearchRange,
                                    theDirectionConstraints, clearanceAndMaxDistance, gammaAxis);

    ptrPlanner = rrg_star;
    printf("\n%s\n", "HolonomicRRG* loaded");
  } else if (!strcmp(planner_type, "DynamicRRG*")) { // RRG*
    LazyRRGstar* lazy_rrg_star = new LazyRRGstar(theStartDummyID, theGoalDummyID,
                                                 theRobotCollectionID, theObstacleCollectionID, ikGroupID,
                                                 thePlanningType, theAngularCoeff,
                                                 theStepSize,
                                                 theSearchMinVal, theSearchRange,
                                                 theDirectionConstraints, clearanceAndMaxDistance, gammaAxis);

    ptrPlanner = lazy_rrg_star;
    printf("\n%s\n", "Dynamic RRG* loaded");
  } else {
    fprintf(stderr, "%s\n", "Undefined planner type");
    return;
  }

  while (fscanf(aux_fp, "%s %f", option_type, &option_value) != -1) {
    if (!strcmp(option_type, "goalBias")) {
      if (option_value >= 0.0 && option_value <= 1.0) {
        ptrPlanner->setGoalBias(option_value);
        printf("%s : %f\n", option_type, option_value);
      } else {
        fprintf(stderr, "$goalBias should be betewen 0.0 ~ 1.0\n");
      }
    } else if (!strcmp(option_type, "maxDistance")) {
      if (option_value >= 0.0 && option_value <= 1.0) {
        ptrPlanner->setMaxDistance(option_value);
        printf("%s : %f\n", option_type, option_value);
      } else {
        fprintf(stderr, "$maxDistance should be betewen 0.0 ~ 1.0\n");
      }
    } else {
      fprintf(stderr, "Invalid option, %s : %f\n", option_type, option_value);
    }
  }
}

CHolonomicPathPlanning::~CHolonomicPathPlanning() {
  // TODO
  // ptrPlanner->~CHolonomicPathPlanning();
}

void CHolonomicPathPlanning::setAngularCoefficient(float coeff) {
  ptrPlanner->setAngularCoefficient(coeff);
}

void CHolonomicPathPlanning::setStepSize(float size) {
  ptrPlanner->setStepSize(size);
}

void CHolonomicPathPlanning::getSearchTreeData(std::vector<float>& data, bool fromTheStart) {
  ptrPlanner->getSearchTreeData(data, fromTheStart);
}

int CHolonomicPathPlanning::searchPath(int maxTimePerPass) {
  // maxTimePerPass is in milliseconds
  return ptrPlanner->searchPath(maxTimePerPass);
}

void CHolonomicPathPlanning::setGoalBias(float value) {
  _goalBias = value;
}

void CHolonomicPathPlanning::setMaxDistance(float value) {
  _maxDistance = value;
}

bool CHolonomicPathPlanning::setPartialPath() {
  return ptrPlanner->setPartialPath();
}

bool CHolonomicPathPlanning::addVector(C3Vector& pos, C4Vector& orient, float vect[7]) {
  // return value true means values are not forbidden!
  return ptrPlanner->addVector(pos, orient, vect);
}

int CHolonomicPathPlanning::smoothFoundPath(int steps, int maxTimePerPass) {
  // step specifies the number of passes (each pass is composed by a calculated sub-pass, and some random sub-pass)
  // We first copy foundPath:
  return ptrPlanner->smoothFoundPath(steps, maxTimePerPass);
}

void CHolonomicPathPlanning::getPathData(std::vector<float>& data) {
  ptrPlanner->getPathData(data);
}

bool CHolonomicPathPlanning::areDirectionConstraintsRespected(float vect[7]) {
  return ptrPlanner->areDirectionConstraintsRespected(vect);
}

bool CHolonomicPathPlanning::areSomeValuesForbidden(float values[7]) {
  return ptrPlanner->areSomeValuesForbidden(values);
}

bool CHolonomicPathPlanning::doCollide(float* dist) {
  // dist can be NULL. Dist returns the actual distance only when return value is true!! otherwise it is SIM_MAX_FLOAT!!
  return ptrPlanner->doCollide(dist);
}


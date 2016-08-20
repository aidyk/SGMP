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

	char planner_type[32], option_type[32];
  float option_value;

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
		} else if (!strcmp(option_type, "maxTimebudget")) {
			ptrPlanner->setMaxTimebudget(option_value);
			printf("%s : %f\n", option_type, option_value);
		} else if (!strcmp(option_type, "rewireFactor")) {
			ptrPlanner->setRewireFactor(option_value);
			printf("%s : %f\n", option_type, option_value);
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
  angularCoeff = coeff;
}

void CHolonomicPathPlanning::setStepSize(float size) {
  stepSize = size;
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

void CHolonomicPathPlanning::setMaxTimebudget(float value) {
	_maxTimebudget = value;
}

void CHolonomicPathPlanning::setRewireFactor(float value) {
	_rewireFactor = value;
}

bool CHolonomicPathPlanning::setPartialPath() {
  return ptrPlanner->setPartialPath();
}

// Return value true means values are not forbidden!
bool CHolonomicPathPlanning::addVector(C3Vector& pos, C4Vector& orient, float vect[7]) {
  // return value true means values are not forbidden!
  float auxVect[7];
  if (planningType == sim_holonomicpathplanning_xy) {
    pos(0) += vect[0];
    pos(1) += vect[1];
    auxVect[0] = pos(0);
    auxVect[1] = pos(1);
  } else if (planningType == sim_holonomicpathplanning_xg) {
    pos(0) += vect[0];
    orient = orient * _gammaAxisRotation * (C4Vector(C3Vector(0.0f, 0.0f, vect[1])) * _gammaAxisRotationInv);
    auxVect[0] = pos(0);
    auxVect[1] = (_gammaAxisRotationInv * orient * _gammaAxisRotation).getEulerAngles()(2);
  } else if (planningType == sim_holonomicpathplanning_xyz) {
    pos(0) += vect[0];
    pos(1) += vect[1];
    pos(2) += vect[2];
    auxVect[0] = pos(0);
    auxVect[1] = pos(1);
    auxVect[2] = pos(2);
  } else if (planningType == sim_holonomicpathplanning_xyg) {
    pos(0) += vect[0];
    pos(1) += vect[1];
    orient = orient * _gammaAxisRotation * (C4Vector(C3Vector(0.0f, 0.0f, vect[2])) * _gammaAxisRotationInv);
    auxVect[0] = pos(0);
    auxVect[1] = pos(1);
    auxVect[2] = (_gammaAxisRotationInv * orient * _gammaAxisRotation).getEulerAngles()(2);
  } else if (planningType == sim_holonomicpathplanning_abg) {
    orient = orient * C4Vector(vect);
    auxVect[0] = orient(0);
    auxVect[1] = orient(1);
    auxVect[2] = orient(2);
    auxVect[3] = orient(3);
  } else if (planningType == sim_holonomicpathplanning_xyzg) {
    pos(0) += vect[0];
    pos(1) += vect[1];
    pos(2) += vect[2];
    orient = orient * _gammaAxisRotation * (C4Vector(C3Vector(0.0f, 0.0f, vect[3])) * _gammaAxisRotationInv);
    auxVect[0] = pos(0);
    auxVect[1] = pos(1);
    auxVect[2] = pos(2);
    auxVect[3] = (_gammaAxisRotationInv * orient * _gammaAxisRotation).getEulerAngles()(2);
  } else if (planningType == sim_holonomicpathplanning_xabg) {
    pos(0) += vect[0];
    orient = orient * C4Vector(vect + 1);
    auxVect[0] = pos(0);
    auxVect[1] = orient(0);
    auxVect[2] = orient(1);
    auxVect[3] = orient(2);
    auxVect[4] = orient(3);
  } else if (planningType == sim_holonomicpathplanning_xyabg) {
    pos(0) += vect[0];
    pos(1) += vect[1];
    orient = orient * C4Vector(vect + 2);
    auxVect[0] = pos(0);
    auxVect[1] = pos(1);
    auxVect[2] = orient(0);
    auxVect[3] = orient(1);
    auxVect[4] = orient(2);
    auxVect[5] = orient(3);
  } else { // (planningType==sim_holonomicpathplanning_xyzabg)
    pos(0) += vect[0];
    pos(1) += vect[1];
    pos(2) += vect[2];
    orient = orient * C4Vector(vect + 3);
    auxVect[0] = pos(0);
    auxVect[1] = pos(1);
    auxVect[2] = pos(2);
    auxVect[3] = orient(0);
    auxVect[4] = orient(1);
    auxVect[5] = orient(2);
    auxVect[6] = orient(3);
  }
  return(!areSomeValuesForbidden(auxVect));
}

int CHolonomicPathPlanning::getVector(CHolonomicPathNode *fromPoint, CHolonomicPathNode *toPoint, float vect[], float e, float &artificialLength, bool dontDivide) {
  // if direction constraints are not respected, return value is -1 and vect does not contain anything
  // Otherwise return value is the number of times we have to add 'vect' to 'fromPoint' to reach 'toPoint'
  int retVal = -1;

  if (planningType == sim_holonomicpathplanning_xy) {
    vect[0] = toPoint->values[0] - fromPoint->values[0];
    vect[1] = toPoint->values[1] - fromPoint->values[1];
    if (areDirectionConstraintsRespected(vect)) {
      artificialLength = sqrtf(vect[0] * vect[0] + vect[1] * vect[1]);
      if (dontDivide)
        return(1);
      retVal = (int)(artificialLength / e) + 1;
      float l = (float)retVal;
      vect[0] /= l;
      vect[1] /= l;
    }

  } else if (planningType == sim_holonomicpathplanning_xg) {
    vect[0] = toPoint->values[0] - fromPoint->values[0];
    vect[1] = CPathPlanningInterface::getNormalizedAngle(toPoint->values[1] - fromPoint->values[1]);
    if (areDirectionConstraintsRespected(vect)) {
      artificialLength = vect[0] * vect[0];
      artificialLength = sqrtf(artificialLength + vect[1] * angularCoeff * vect[1] * angularCoeff);
      if (dontDivide)
        return(1);
      retVal = (int)(artificialLength / e) + 1;
      float l = (float)retVal;
      vect[0] /= l;
      vect[1] /= l;
    }

  } else if (planningType == sim_holonomicpathplanning_xyz) {
    vect[0] = toPoint->values[0] - fromPoint->values[0];
    vect[1] = toPoint->values[1] - fromPoint->values[1];
    vect[2] = toPoint->values[2] - fromPoint->values[2];
    if (areDirectionConstraintsRespected(vect)) {
      artificialLength = sqrtf(vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2]);
      if (dontDivide)
        return(1);
      retVal = (int)(artificialLength / e) + 1;
      float l = (float)retVal;
      vect[0] /= l;
      vect[1] /= l;
      vect[2] /= l;
    }

  } else if (planningType == sim_holonomicpathplanning_xyg) {
    vect[0] = toPoint->values[0] - fromPoint->values[0];
    vect[1] = toPoint->values[1] - fromPoint->values[1];
    vect[2] = CPathPlanningInterface::getNormalizedAngle(toPoint->values[2] - fromPoint->values[2]);
    if (areDirectionConstraintsRespected(vect)) {
      artificialLength = vect[0] * vect[0] + vect[1] * vect[1];
      artificialLength = sqrtf(artificialLength + vect[2] * angularCoeff * vect[2] * angularCoeff);
      if (dontDivide)
        return(1);
      retVal = (int)(artificialLength / e) + 1;
      float l = (float)retVal;
      vect[0] /= l;
      vect[1] /= l;
      vect[2] /= l;
    }

  } else if (planningType == sim_holonomicpathplanning_abg) {
    C4Vector toP, fromP;
    C3Vector dum;
    toPoint->getAllValues(dum, toP);
    fromPoint->getAllValues(dum, fromP);
    C4Vector diff(fromP.getInverse()*toP);
    vect[0] = diff(0);
    vect[1] = diff(1);
    vect[2] = diff(2);
    vect[3] = diff(3);
    if (areDirectionConstraintsRespected(vect)) {
      artificialLength = angularCoeff * fromP.getAngleBetweenQuaternions(toP);
      if (dontDivide)
        return(1);
      retVal = (int)(artificialLength / e) + 1;
      float l = (float)retVal;
      C4Vector q;
      q.setIdentity();
      fromP.buildInterpolation(q, diff, 1.0f / l);
      vect[0] = fromP(0);
      vect[1] = fromP(1);
      vect[2] = fromP(2);
      vect[3] = fromP(3);
    }

  } else if (planningType == sim_holonomicpathplanning_xyzg) {
    vect[0] = toPoint->values[0] - fromPoint->values[0];
    vect[1] = toPoint->values[1] - fromPoint->values[1];
    vect[2] = toPoint->values[2] - fromPoint->values[2];
    vect[3] = CPathPlanningInterface::getNormalizedAngle(toPoint->values[3] - fromPoint->values[3]);
    if (areDirectionConstraintsRespected(vect)) {
      artificialLength = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2];
      artificialLength = sqrtf(artificialLength + vect[3] * angularCoeff * vect[3] * angularCoeff);
      if (dontDivide)
        return(1);
      retVal = (int)(artificialLength / e) + 1;
      float l = (float)retVal;
      vect[0] /= l;
      vect[1] /= l;
      vect[2] /= l;
      vect[3] /= l;
    }
  } else if (planningType == sim_holonomicpathplanning_xabg) {
    vect[0] = toPoint->values[0] - fromPoint->values[0];
    C4Vector toP, fromP;
    C3Vector dum;
    toPoint->getAllValues(dum, toP);
    fromPoint->getAllValues(dum, fromP);
    C4Vector diff(fromP.getInverse()*toP);
    vect[1] = diff(0);
    vect[2] = diff(1);
    vect[3] = diff(2);
    vect[4] = diff(3);
    if (areDirectionConstraintsRespected(vect)) {
      float ap = angularCoeff * fromP.getAngleBetweenQuaternions(toP);
      artificialLength = vect[0] * vect[0];
      artificialLength = sqrtf(artificialLength + ap * ap);

      if (dontDivide)
        return(1);
      retVal = (int)(artificialLength / e) + 1;
      float l = (float)retVal;
      vect[0] /= l;
      C4Vector q;
      q.setIdentity();
      fromP.buildInterpolation(q, diff, 1.0f / l);
      vect[1] = fromP(0);
      vect[2] = fromP(1);
      vect[3] = fromP(2);
      vect[4] = fromP(3);
    }

  } else if (planningType == sim_holonomicpathplanning_xyabg) {
    vect[0] = toPoint->values[0] - fromPoint->values[0];
    vect[1] = toPoint->values[1] - fromPoint->values[1];
    C4Vector toP, fromP;
    C3Vector dum;
    toPoint->getAllValues(dum, toP);
    fromPoint->getAllValues(dum, fromP);
    C4Vector diff(fromP.getInverse()*toP);
    vect[2] = diff(0);
    vect[3] = diff(1);
    vect[4] = diff(2);
    vect[5] = diff(3);
    if (areDirectionConstraintsRespected(vect)) {
      float ap = angularCoeff * fromP.getAngleBetweenQuaternions(toP);
      artificialLength = vect[0] * vect[0] + vect[1] * vect[1];
      artificialLength = sqrtf(artificialLength + ap * ap);

      if (dontDivide)
        return(1);
      retVal = (int)(artificialLength / e) + 1;
      float l = (float)retVal;
      vect[0] /= l;
      vect[1] /= l;
      C4Vector q;
      q.setIdentity();
      fromP.buildInterpolation(q, diff, 1.0f / l);
      vect[2] = fromP(0);
      vect[3] = fromP(1);
      vect[4] = fromP(2);
      vect[5] = fromP(3);
    }

  } else if (planningType == sim_holonomicpathplanning_xyzabg) {
    vect[0] = toPoint->values[0] - fromPoint->values[0];
    vect[1] = toPoint->values[1] - fromPoint->values[1];
    vect[2] = toPoint->values[2] - fromPoint->values[2];
    C4Vector toP, fromP;
    C3Vector dum;
    toPoint->getAllValues(dum, toP);
    fromPoint->getAllValues(dum, fromP);
    C4Vector diff(fromP.getInverse()*toP);
    vect[3] = diff(0);
    vect[4] = diff(1);
    vect[5] = diff(2);
    vect[6] = diff(3);
    if (areDirectionConstraintsRespected(vect)) {
      float ap = angularCoeff * fromP.getAngleBetweenQuaternions(toP);
      artificialLength = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2];
      artificialLength = sqrtf(artificialLength + ap * ap);

      if (dontDivide)
        return(1);
      retVal = (int)(artificialLength / e) + 1;
      float l = (float)retVal;
      vect[0] /= l;
      vect[1] /= l;
      vect[2] /= l;
      C4Vector q;
      q.setIdentity();
      fromP.buildInterpolation(q, diff, 1.0f / l);
      vect[3] = fromP(0);
      vect[4] = fromP(1);
      vect[5] = fromP(2);
      vect[6] = fromP(3);
    }
  }
  return(retVal);
}

int CHolonomicPathPlanning::smoothFoundPath(int steps, int maxTimePerPass) {
  // step specifies the number of passes (each pass is composed by a calculated sub-pass, and some random sub-pass)
  // We first copy foundPath:
  if (steps < 2)
    return(1);
  if (invalidData)
    return(0);
  CDummyDummy* startDummy = (CDummyDummy*)_simGetObject(startDummyID);
  if (startDummy == NULL)
    return(0);

  if (foundPath.size() < 3)
    return(0);

  if (foundPathSameStraightLineID_forSteppedSmoothing.size() == 0) {
    // the first time we call this routine!
    sameStraightLineNextID_forSteppedSmoothing = 0;
    for (int i = 0; i < int(foundPath.size()); i++)
      foundPathSameStraightLineID_forSteppedSmoothing.push_back(sameStraightLineNextID_forSteppedSmoothing++);
    numberOfRandomConnectionTries_forSteppedSmoothing = steps;
    numberOfRandomConnectionTriesLeft_forSteppedSmoothing = steps;
  }
  int startTime = simGetSystemTimeInMs(-1);
  while (true) {
    if (numberOfRandomConnectionTriesLeft_forSteppedSmoothing <= 0) {
      // we finished smoothing!
      return(1);
    }
    if (_simGetTimeDiffInMs(startTime) > maxTimePerPass)
      return(-1); // we are not yet finished, but we did enough for the time we had
    numberOfRandomConnectionTriesLeft_forSteppedSmoothing--;
    int lowIndex, highIndex;
    CHolonomicPathNode* startP;
    CHolonomicPathNode* endP;
    for (int randomPass = 0; randomPass < 5; randomPass++) {
      // If randomPass==0, the pass is not random, i.e. the low and high indices are calculated
      startP = NULL; // added on 2010/09/09
      if (randomPass == 0) {
        // We calculate lowIndex and highIndex!
        float span = float(foundPath.size()) / float(numberOfRandomConnectionTries_forSteppedSmoothing);
        while ( (span < 5) && (numberOfRandomConnectionTries_forSteppedSmoothing > 1) ) {
          numberOfRandomConnectionTries_forSteppedSmoothing--;
          if (numberOfRandomConnectionTriesLeft_forSteppedSmoothing >= numberOfRandomConnectionTries_forSteppedSmoothing)
            numberOfRandomConnectionTriesLeft_forSteppedSmoothing = numberOfRandomConnectionTries_forSteppedSmoothing - 1;
          span = float(foundPath.size()) / float(numberOfRandomConnectionTries_forSteppedSmoothing);
        }
        if (numberOfRandomConnectionTries_forSteppedSmoothing <= 1)
          return(1); // finished!
        lowIndex = int(span * float(numberOfRandomConnectionTriesLeft_forSteppedSmoothing + 0));
        highIndex = int(span * float(numberOfRandomConnectionTriesLeft_forSteppedSmoothing + 1));
        while (highIndex >= int(foundPath.size())) // probably not needed
          highIndex--;
        if (foundPathSameStraightLineID_forSteppedSmoothing[lowIndex] != foundPathSameStraightLineID_forSteppedSmoothing[highIndex]) {
          // otherwise this pass is skipped!
          startP = static_cast<CHolonomicPathNode*>(foundPath[lowIndex]);
          endP = static_cast<CHolonomicPathNode*>(foundPath[highIndex]);
        }
      } else {
        // We randomly chose lowIndex and highIndex!
        for (int i = 0; i < 5; i++) {
          // so that if we have only very few ids, we don't stay stuck here
          int ra = int((SIM_RAND_FLOAT * float(foundPath.size())) - 0.5f);
          int rb = int((SIM_RAND_FLOAT * float(foundPath.size())) - 0.5f);
          if ( (ra != rb) && (abs(ra - rb) > 1) && (foundPathSameStraightLineID_forSteppedSmoothing[ra] != foundPathSameStraightLineID_forSteppedSmoothing[rb]) ) {
            lowIndex = SIM_MIN(ra, rb);
            highIndex = SIM_MAX(ra, rb);
            startP = static_cast<CHolonomicPathNode*>(foundPath[lowIndex]);
            endP = static_cast<CHolonomicPathNode*>(foundPath[highIndex]);
            break;
          }
        }
      }
      if (startP != NULL) {
        // Now let's try to link highIndex from lowIndex with a "straight" line:
        float vect[7];
        float artificialVectorLength;
        int passes = getVector(startP, endP, vect, stepSize, artificialVectorLength, false);
        if ( (passes != -1) && (highIndex - (lowIndex + 1) > passes - 1) ) {
          // no forbidden direction, and the number of nodes is reduced!
          C3Vector pos;
          C4Vector orient;
          startP->getAllValues(pos, orient);

          C3Vector posCop(pos);
          C4Vector orientCop(orient);
          bool impossible = false;
          C7Vector originalLocal;
          _simGetObjectLocalTransformation(startDummy, originalLocal.X.data, originalLocal.Q.data, true);
          for (int currentPass = 0; currentPass < passes - 1; currentPass++) {
            if (!addVector(pos, orient, vect)) {
              impossible = true;
              break; // We are in forbidden values!
            }
            C7Vector transf(orient, pos);
            C7Vector tmpTr(_startDummyLTM * transf);
            _simSetObjectLocalTransformation(startDummy, tmpTr.X.data, tmpTr.Q.data);
            if (doCollide(NULL)) {
              impossible = true;
              break;
            }
          }
          _simSetObjectLocalTransformation(startDummy, originalLocal.X.data, originalLocal.Q.data);
          pos = posCop;
          orient = orientCop;
          if (!impossible) {
            // Path was collision free:
            // We delete the nodes between low and high, then remove some, to have to correct desired length:
            for (int i = lowIndex + 1; i < highIndex; i++)
              delete foundPath[i];
            int a = lowIndex + 1;
            int b = lowIndex + 1 + highIndex - (lowIndex + 1) - (passes - 1) - 0;
            foundPath.erase(foundPath.begin() + a, foundPath.begin() + b);
            foundPathSameStraightLineID_forSteppedSmoothing.erase(foundPathSameStraightLineID_forSteppedSmoothing.begin() + a, foundPathSameStraightLineID_forSteppedSmoothing.begin() + b);

            for (int currentPass = 0; currentPass < passes - 1; currentPass++) {
              addVector(pos, orient, vect);
              CHolonomicPathNode* it = endP->copyYourself(); // just to have the right size!
              it->setAllValues(pos, orient);
              foundPath[lowIndex + 1 + currentPass] = it;
              foundPathSameStraightLineID_forSteppedSmoothing[lowIndex + 1 + currentPass] = sameStraightLineNextID_forSteppedSmoothing;
            }
            sameStraightLineNextID_forSteppedSmoothing++;
          }
        }
      }
    }
  }
  return(0); // will never pass here!
}

void CHolonomicPathPlanning::getPathData(std::vector<float>& data) {
  ptrPlanner->getPathData(data);
    /*
  data.clear();
  printf("inv : %d %d\n", invalidData, foundPath.size());
  if (invalidData)
    return;
  for (int i = 0; i < int(foundPath.size()); i++) {
    CHolonomicPathNode* theNode = foundPath[i];
    C3Vector p;
    C4Vector o;
    theNode->getAllValues(p, o);

    C7Vector conf(o, p);
    conf = _startDummyCTM * conf;
    data.push_back(conf(0));
    data.push_back(conf(1));
    data.push_back(conf(2));
    data.push_back(conf(3));
    data.push_back(conf(4));
    data.push_back(conf(5));
    data.push_back(conf(6));
  }
  */
}

bool CHolonomicPathPlanning::areDirectionConstraintsRespected(float vect[7]) {
  if (planningType == sim_holonomicpathplanning_xy) {
    if (!_directionConstraintsOn)
      return(true);
    for (int i = 0; i < 2; i++) {
      if ( (_directionConstraints[i] == -1) && (vect[i] > 0.0f) )
        return(false);
      if ( (_directionConstraints[i] == +1) && (vect[i] < 0.0f) )
        return(false);
    }
    return(true);
  }
  if (planningType == sim_holonomicpathplanning_xg) {
    CPathPlanningInterface::getNormalizedAngle(vect[1]);
    if (!_directionConstraintsOn)
      return(true);
    if ( (_directionConstraints[0] == -1) && (vect[0] > 0.0f) )
      return(false);
    if ( (_directionConstraints[0] == +1) && (vect[0] < 0.0f) )
      return(false);
    if ( (_directionConstraints[3] == -1) && (vect[1] > 0.0f) )
      return(false);
    if ( (_directionConstraints[3] == +1) && (vect[1] < 0.0f) )
      return(false);
    return(true);
  }
  if (planningType == sim_holonomicpathplanning_xyz) {
    if (!_directionConstraintsOn)
      return(true);
    for (int i = 0; i < 3; i++) {
      if ( (_directionConstraints[i] == -1) && (vect[i] > 0.0f) )
        return(false);
      if ( (_directionConstraints[i] == +1) && (vect[i] < 0.0f) )
        return(false);
    }
    return(true);
  }
  if (planningType == sim_holonomicpathplanning_xyg) {
    CPathPlanningInterface::getNormalizedAngle(vect[2]);
    if (!_directionConstraintsOn)
      return(true);
    for (int i = 0; i < 2; i++) {
      if ( (_directionConstraints[i] == -1) && (vect[i] > 0.0f) )
        return(false);
      if ( (_directionConstraints[i] == +1) && (vect[i] < 0.0f) )
        return(false);
    }
    if ( (_directionConstraints[3] == -1) && (vect[2] > 0.0f) )
      return(false);
    if ( (_directionConstraints[3] == +1) && (vect[2] < 0.0f) )
      return(false);
    return(true);
  }
  if (planningType == sim_holonomicpathplanning_abg) {
    // No direction constraints for A,B,G here!
    return(true);
  }
  if (planningType == sim_holonomicpathplanning_xyzg) {
    CPathPlanningInterface::getNormalizedAngle(vect[3]);
    if (!_directionConstraintsOn)
      return(true);
    for (int i = 0; i < 3; i++) {
      if ( (_directionConstraints[i] == -1) && (vect[i] > 0.0f) )
        return(false);
      if ( (_directionConstraints[i] == +1) && (vect[i] < 0.0f) )
        return(false);
    }
    if ( (_directionConstraints[3] == -1) && (vect[3] > 0.0f) )
      return(false);
    if ( (_directionConstraints[3] == +1) && (vect[3] < 0.0f) )
      return(false);
    return(true);
  }
  if (planningType == sim_holonomicpathplanning_xabg) {
    // No direction constraints for A,B,G here!
    if (!_directionConstraintsOn)
      return(true);
    if ( (_directionConstraints[0] == -1) && (vect[0] > 0.0f) )
      return(false);
    if ( (_directionConstraints[0] == +1) && (vect[0] < 0.0f) )
      return(false);
    return(true);
  }
  if (planningType == sim_holonomicpathplanning_xyabg) {
    // No direction constraints for A,B,G here!
    if (!_directionConstraintsOn)
      return(true);
    for (int i = 0; i < 2; i++) {
      if ( (_directionConstraints[i] == -1) && (vect[i] > 0.0f) )
        return(false);
      if ( (_directionConstraints[i] == +1) && (vect[i] < 0.0f) )
        return(false);
    }
    return(true);
  }
  if (planningType == sim_holonomicpathplanning_xyzabg) {
    // No direction constraints for A,B,G here!
    if (!_directionConstraintsOn)
      return(true);
    for (int i = 0; i < 3; i++) {
      if ( (_directionConstraints[i] == -1) && (vect[i] > 0.0f) )
        return(false);
      if ( (_directionConstraints[i] == +1) && (vect[i] < 0.0f) )
        return(false);
    }
    return(true);
  }
  return(true);
}

bool CHolonomicPathPlanning::areSomeValuesForbidden(C7Vector configuration) {
  float values[7];
  configuration.getInternalData(values);
  return areSomeValuesForbidden(values);
}

bool CHolonomicPathPlanning::areSomeValuesForbidden(float values[7]) {
  float gamma = 0.0f;
  if (planningType == sim_holonomicpathplanning_xy) {
    if ((values[0] < _searchMinVal[0]) || (values[0] > _searchMinVal[0] + _searchRange[0]))
      return(true);
    if ((values[1] < _searchMinVal[1]) || (values[1] > _searchMinVal[1] + _searchRange[1]))
      return(true);
    return(false);
  }
  if (planningType == sim_holonomicpathplanning_xg) {
    if ((values[0] < _searchMinVal[0]) || (values[0] > _searchMinVal[0] + _searchRange[0]))
      return(true);
    gamma = values[1];
  }
  if (planningType == sim_holonomicpathplanning_xyz) {
    if ((values[0] < _searchMinVal[0]) || (values[0] > _searchMinVal[0] + _searchRange[0]))
      return(true);
    if ((values[1] < _searchMinVal[1]) || (values[1] > _searchMinVal[1] + _searchRange[1]))
      return(true);
    if ((values[2] < _searchMinVal[2]) || (values[2] > _searchMinVal[2] + _searchRange[2]))
      return(true);
    return(false);
  }
  if (planningType == sim_holonomicpathplanning_xyg) {
    if ((values[0] < _searchMinVal[0]) || (values[0] > _searchMinVal[0] + _searchRange[0]))
      return(true);
    if ((values[1] < _searchMinVal[1]) || (values[1] > _searchMinVal[1] + _searchRange[1]))
      return(true);
    gamma = values[2];
  }
  if (planningType == sim_holonomicpathplanning_abg) {
    return(false);
  }
  if (planningType == sim_holonomicpathplanning_xyzg) {
    if ((values[0] < _searchMinVal[0]) || (values[0] > _searchMinVal[0] + _searchRange[0]))
      return(true);
    if ((values[1] < _searchMinVal[1]) || (values[1] > _searchMinVal[1] + _searchRange[1]))
      return(true);
    if ((values[2] < _searchMinVal[2]) || (values[2] > _searchMinVal[2] + _searchRange[2]))
      return(true);
    gamma = values[3];
  }
  if (planningType == sim_holonomicpathplanning_xabg) {
    if ((values[0] < _searchMinVal[0]) || (values[0] > _searchMinVal[0] + _searchRange[0]))
      return(true);
    return(false);
  }
  if (planningType == sim_holonomicpathplanning_xyabg) {
    if ((values[0] < _searchMinVal[0]) || (values[0] > _searchMinVal[0] + _searchRange[0]))
      return(true);
    if ((values[1] < _searchMinVal[1]) || (values[1] > _searchMinVal[1] + _searchRange[1]))
      return(true);
    return(false);
  }
  if (planningType == sim_holonomicpathplanning_xyzabg) {
    if ((values[0] < _searchMinVal[0]) || (values[0] > _searchMinVal[0] + _searchRange[0]))
      return(true);
    if ((values[1] < _searchMinVal[1]) || (values[1] > _searchMinVal[1] + _searchRange[1]))
      return(true);
    if ((values[2] < _searchMinVal[2]) || (values[2] > _searchMinVal[2] + _searchRange[2]))
      return(true);
    return(false);
  }
  // We check the gamma value here:
  if (_searchRange[3] > (359.0f * degToRad))
    return(false);
  // Search range is smaller than 360 degrees:
  while (gamma > _searchMinVal[3])
    gamma -= piValTimes2;
  while (gamma < _searchMinVal[3])
    gamma += piValTimes2;
  return(gamma > (_searchMinVal[3] + _searchRange[3]));
}

bool CHolonomicPathPlanning::doCollide(float* dist) {
  _collision_detection_count += 1;
  // dist can be NULL. Dist returns the actual distance only when return value is true!! otherwise it is SIM_MAX_FLOAT!!
  if (dist != NULL)
    dist[0] = SIM_MAX_FLOAT;
  if (obstacleClearanceAndMaxDistance[0] <= 0.0f) {
    if ( (obstacleCollectionID == -1) && (!_allIsObstacle) )
      return(false);
    if (_simDoEntitiesCollide(robotCollectionID, obstacleCollectionID, buffer, false, false, true) != 0) {
      if (dist != NULL)
        dist[0] = 0.0f;
      return(true);
    }
    return(false);
  } else {
    if ( (obstacleCollectionID == -1) && (!_allIsObstacle) )
      return(false);
    if (obstacleClearanceAndMaxDistance[1] <= 0.0f) {
      // no max distance, only min. distance:
      float ray[7];
      float theDist = obstacleClearanceAndMaxDistance[0];
      if (_simGetDistanceBetweenEntitiesIfSmaller(robotCollectionID, obstacleCollectionID, &theDist, ray, buffer, false, false, true) != 0) {
        if (dist != NULL)
          dist[0] = theDist;
        return(true);
      }
      return(false);
    } else {
      // min. distance and max. distance:
      float ray[7];
      float theDist = obstacleClearanceAndMaxDistance[1];
      if (_simGetDistanceBetweenEntitiesIfSmaller(robotCollectionID, obstacleCollectionID, &theDist, ray, buffer, false, false, true) != 0) {
        if (theDist >= obstacleClearanceAndMaxDistance[0])
          return(false);
        if (dist != NULL)
          dist[0] = theDist;
        return(true);
      }
      if (dist != NULL)
        dist[0] = theDist;
      return(true);
    }
  }
}

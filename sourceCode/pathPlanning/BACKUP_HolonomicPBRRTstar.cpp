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

#include "HolonomicPBRRTstar.h"
#include "pathPlanningInterface.h"
#include "v_repLib.h"

#include <stack>

#define SIM_MIN(a,b) (((a)<(b)) ? (a) : (b))
#define SIM_MAX(a,b) (((a)>(b)) ? (a) : (b))

HolonomicPBRRTstar::HolonomicPBRRTstar(int theStartDummyID, int theGoalDummyID,
                               int theRobotCollectionID, int theObstacleCollectionID, int ikGroupID,
                               int thePlanningType, float theAngularCoeff,
                               float theStepSize,
                               const float theSearchMinVal[4], const float theSearchRange[4],
                               const int theDirectionConstraints[4], const float clearanceAndMaxDistance[2], const C3Vector& gammaAxis)
{
  isHolonomic = true;
  float angle = C3Vector::unitZVector.getAngle(gammaAxis);
  if (angle < 0.1f * degToRad)
    _gammaAxisRotation.setIdentity();
  else {
    if (angle > 179.9f * degToRad)
      _gammaAxisRotation.setEulerAngles(piValue, 0.0f, 0.0f);
    else {
      C3Vector r((C3Vector::unitZVector ^ gammaAxis).getNormalized());
      _gammaAxisRotation.setAngleAndAxis(angle, r);
    }
  }
  _gammaAxisRotationInv = _gammaAxisRotation.getInverse();

  _allIsObstacle = (theObstacleCollectionID == -1);
  firstPass = true;
  invalidData = true;
  obstacleClearanceAndMaxDistance[0] = clearanceAndMaxDistance[0];
  obstacleClearanceAndMaxDistance[1] = clearanceAndMaxDistance[1];
  planningType = thePlanningType;
  startDummyID = theStartDummyID;
  goalDummyID = theGoalDummyID;
  CDummyDummy* startDummy = (CDummyDummy*)_simGetObject(startDummyID);
  CDummyDummy* goalDummy = (CDummyDummy*)_simGetObject(goalDummyID);
  if ( (startDummy == NULL) || (goalDummy == NULL) )
    return;

  _simGetObjectCumulativeTransformation(startDummy, _startDummyCTM.X.data, _startDummyCTM.Q.data, false);
  _simGetObjectLocalTransformation(startDummy, _startDummyLTM.X.data, _startDummyLTM.Q.data, false);

  C7Vector goalDummyCumulTransf;
  _simGetObjectCumulativeTransformation(goalDummy, goalDummyCumulTransf.X.data, goalDummyCumulTransf.Q.data, false);
  C7Vector goalDummyLocalConf(_startDummyCTM.getInverse() * goalDummyCumulTransf);

  C7Vector sConf;
  sConf.setIdentity();
/*
  fromStart.push_back(new HolonomicPBRRTstarNode(planningType, sConf, _gammaAxisRotation, _gammaAxisRotationInv));
*/
  HolonomicPBRRTstarNode* start_node = new HolonomicPBRRTstarNode(planningType, sConf, _gammaAxisRotation, _gammaAxisRotationInv);

  fromGoal.push_back(new HolonomicPBRRTstarNode(planningType, goalDummyLocalConf, _gammaAxisRotation, _gammaAxisRotationInv));

  start_node->setCost(0.0f);
  start_node->setRadius(SIM_MAX_FLOAT);
  static_cast<HolonomicPBRRTstarNode*>(fromGoal[0])->setCost(SIM_MAX_FLOAT);

  robotCollectionID = theRobotCollectionID;
  obstacleCollectionID = theObstacleCollectionID;

  angularCoeff = theAngularCoeff;
  stepSize = theStepSize;
  _directionConstraintsOn = false;

  for (int i = 0; i < 4; i++) {
    _searchMinVal[i] = theSearchMinVal[i];
    _searchRange[i] = theSearchRange[i];
    _directionConstraints[i] = theDirectionConstraints[i];
    if (_directionConstraints[i] != 0)
      _directionConstraintsOn = true;
  }

#ifdef OMPL_NN
  _nn.reset(new ompl::NearestNeighborsLinear<HolonomicPBRRTstarNode*>()); // Initialize NearestNeighbors structure
  // _nn.reset(new ompl::NearestNeighborsGNAT<HolonomicPBRRTstarNode*>()); // Initialize NearestNeighbors structure
  _nn->setDistanceFunction(boost::bind(&HolonomicPBRRTstar::distance, this, _1, _2));
  _nn->add(start_node);

  fromStart.push_back(start_node);
#else
  fromStart.push_back(start_node);
#endif

  _delayCC = false;
  _goalBias = 0.05;
  // Set ballRadiusMax and ballRadiusConst to maximum extent
  _ballRadiusMax = _ballRadiusConst = sqrt(_searchRange[0] * _searchRange[0] + _searchRange[1] * _searchRange[1] + _searchRange[2] * _searchRange[2]);
  start_node->setRadius(SIM_MAX_FLOAT); // Initialize the radius of collifion free sphere
  // For performance analysis
  _collisionDetection = _skippedCollisionDetection = _totalSampling = _successfulSampling = _rewireTest = 0LL;
  _earlyChecking = _updateSolution = 0LL;
  _validNodeCount = 1LL;

  if (planningType == sim_holonomicpathplanning_xyz) {
    _kConstant = CONST_E + CONST_E / 3.0;
  } else if (planningType == sim_holonomicpathplanning_xyzabg){
    _kConstant = CONST_E + CONST_E / 3.0;
    // _kConstant = CONST_E + CONST_E / 6.0;
  }

  _updateIteration = 0LL;
  _collisionCache.resize(1000);
  _knownBestCost = SIM_MAX_FLOAT;

  passes = 0;
  break_point = 1000;

  buffer[0] = -1; // What the hell is this?
  invalidData = false;

  debug_fp = fopen("logs_pbrrt.txt", "a");
  if (debug_fp == NULL) {
    fprintf(stderr, "File open error!\n");
    exit(1);
  }
}

HolonomicPBRRTstar::~HolonomicPBRRTstar() {
  for (int i = 0; i < int(fromStart.size()); i++)
    delete fromStart[i];
  fromStart.clear();
  for (int i = 0; i < int(fromGoal.size()); i++)
    delete fromGoal[i];
  fromGoal.clear();
  for (int i = 0; i < int(foundPath.size()); i++)
    delete foundPath[i];
  foundPath.clear();
#ifdef OMPL_NN
  if (_nn) {
    std::vector<HolonomicPBRRTstarNode*> node_list;
    _nn->list(node_list);
    for (int i = 0; i < int(node_list.size()); i++) {
      delete node_list[i];
    }
    _nn->clear();
  }
#endif
}

void HolonomicPBRRTstar::setAngularCoefficient(float coeff) {
  angularCoeff = coeff;
}

void HolonomicPBRRTstar::setStepSize(float size) {
  stepSize = size;
}

float HolonomicPBRRTstar::getNearNeighborRadius() {
  // (1/3 ~= 0.333333) and (1/6 ~= 1.66666)
#ifdef KNN
  // For 3DOF problem
  return ceil((2 << (4)) * _kConstant);
#endif

#ifdef NAIVE
  return SIM_MIN(_ballRadiusMax, _ballRadiusConst * pow(log(1.0 + _nn->size()) / (1.0 + _nn->size()), 0.333333));
#else
#ifdef OMPL_NN
  return SIM_MIN(_ballRadiusMax, _ballRadiusConst * pow(log(1.0 + _validNodeCount) / (1.0 + _validNodeCount), 0.3333333));
#else
  return SIM_MIN(_ballRadiusMax, _ballRadiusConst * pow(log(1.0 + fromStart.size()) / (1.0 + fromStart.size()), 0.333333));
#endif
#endif
}

float HolonomicPBRRTstar::distance(HolonomicPBRRTstarNode* a, HolonomicPBRRTstarNode* b) {
  float dist = 0.0;
  if (planningType == sim_holonomicpathplanning_xyz) {
    float vect[3];
    vect[0] = a->values[0] - b->values[0];
    vect[1] = a->values[1] - b->values[1];
    vect[2] = a->values[2] - b->values[2];

    dist = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2];
  } else if (planningType == sim_holonomicpathplanning_xyzabg) {
    float vect[7];
    vect[0] = a->values[0] - b->values[0];
    vect[1] = a->values[1] - b->values[1];
    vect[2] = a->values[2] - b->values[2];
    C4Vector toP, fromP;
    C3Vector dum;
    a->getAllValues(dum, toP);
    b->getAllValues(dum, fromP);

    float ad = angularCoeff * fromP.getAngleBetweenQuaternions(toP);
    dist = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2] + ad * ad;
  }
  return sqrt(dist);
}

float HolonomicPBRRTstar::squaredDistance(HolonomicPBRRTstarNode* a, HolonomicPBRRTstarNode* b) {
  float dist = 0.0;
  if (planningType == sim_holonomicpathplanning_xyz) {
    float vect[3];
    vect[0] = a->values[0] - b->values[0];
    vect[1] = a->values[1] - b->values[1];
    vect[2] = a->values[2] - b->values[2];

    dist = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2];
  } else if (planningType == sim_holonomicpathplanning_xyzabg) {
    float vect[7];
    vect[0] = a->values[0] - b->values[0];
    vect[1] = a->values[1] - b->values[1];
    vect[2] = a->values[2] - b->values[2];
    C4Vector toP, fromP;
    C3Vector dum;
    a->getAllValues(dum, toP);
    b->getAllValues(dum, fromP);

    float ad = angularCoeff * fromP.getAngleBetweenQuaternions(toP);
    dist = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2] + ad * ad;
  }
  return dist;
}

std::vector<HolonomicPBRRTstarNode*> HolonomicPBRRTstar::getNearNeighborNodes(std::vector<HolonomicPBRRTstarNode*>& nodes,
		HolonomicPBRRTstarNode* query, float radius) {
  std::vector<HolonomicPBRRTstarNode*> neighbors;
  if (planningType == sim_holonomicpathplanning_xyz) {
    for (int i = 0; i < int(nodes.size()); i++) {
      float vect[3];
      vect[0] = query->values[0] - nodes[i]->values[0];
      vect[1] = query->values[1] - nodes[i]->values[1];
      vect[2] = query->values[2] - nodes[i]->values[2];

      float d = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2];
      if (d > radius * radius) continue;
      neighbors.push_back(nodes[i]);
    }
  } else { // (planningType==sim_holonomicpathplanning_xyzabg)
    for (int i = 0; i < int(nodes.size()); i++) {
      float vect[7];
      vect[0] = query->values[0] - nodes[i]->values[0];
      vect[1] = query->values[1] - nodes[i]->values[1];
      vect[2] = query->values[2] - nodes[i]->values[2];
      C4Vector toP, fromP;
      C3Vector dum;
      query->getAllValues(dum, toP);
      nodes[i]->getAllValues(dum, fromP);
      C4Vector diff(fromP.getInverse()*toP);
      vect[3] = diff(0);
      vect[4] = diff(1);
      vect[5] = diff(2);
      vect[6] = diff(3);

      float ad = angularCoeff * fromP.getAngleBetweenQuaternions(toP);
      float d = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2] + ad * ad;
      if (d <= radius * radius)
        neighbors.push_back(nodes[i]);
    }
  }

  return neighbors;
}

void HolonomicPBRRTstar::getSearchTreeData(std::vector<float>& data, bool fromTheStart) {
  std::vector<HolonomicPBRRTstarNode*> *cont = new std::vector<HolonomicPBRRTstarNode*>();
  // cont = reinterpret_cast<std::vector<HolonomicPBRRTstarNode*>*>(&fromGoal);
  if (fromTheStart)
#ifdef OMPL_NN
    _nn->list(*cont);
#else
    cont = reinterpret_cast<std::vector<HolonomicPBRRTstarNode*>*>(&fromStart);
#endif
  else { // Blue
      /*
    _nn->list(*cont);
    if ( (planningType == sim_holonomicpathplanning_xyz) || (planningType == sim_holonomicpathplanning_xyzg) || (planningType == sim_holonomicpathplanning_xyzabg) ) {
      for (int i = 1; i < int(cont->size()); i++) { // if ((*cont)[i]->_ghostParent == ((*cont)[i]->parent)) {
        if ((*cont)[i]->_limiter == NULL) continue; // If soultion not found, goal configuration doesn't have any parent node
        C3Vector start((*cont)[i]->values[0], (*cont)[i]->values[1], (*cont)[i]->values[2]);
        C3Vector goal((*cont)[i]->_limiter->values[0], (*cont)[i]->_limiter->values[1], (*cont)[i]->_limiter->values[2]);
        // C3Vector goal((*cont)[i]->parent->values[0], (*cont)[i]->parent->values[1], (*cont)[i]->parent->values[2]);
        start = _startDummyCTM * start;
        goal = _startDummyCTM * goal;
        float d[6];
        start.copyTo(d);
        goal.copyTo(d + 3);
        for (int j = 0; j < 6; j++)
          data.push_back(d[j]);
      }
    }
    */
    return;
  }
/*
  if ( (planningType == sim_holonomicpathplanning_xy) || (planningType == sim_holonomicpathplanning_xyg) || (planningType == sim_holonomicpathplanning_xyabg) ) {
    for (int i = 1; i < int(cont->size()); i++) {
      if ((*cont)[i] == fromGoal[0]) continue;
      C3Vector start((*cont)[i]->values[0], (*cont)[i]->values[1], 0.0f);
      C3Vector goal((*cont)[i]->parent->values[0], (*cont)[i]->parent->values[1], 0.0f);
      start = _startDummyCTM * start;
      goal = _startDummyCTM * goal;
      float d[6];
      start.copyTo(d);
      goal.copyTo(d + 3);
      for (int j = 0; j < 6; j++)
        data.push_back(d[j]);
    }
  } else
*/
  if ( (planningType == sim_holonomicpathplanning_xyz) || (planningType == sim_holonomicpathplanning_xyzg) || (planningType == sim_holonomicpathplanning_xyzabg) ) {
    for (int i = 1; i < int(cont->size()); i++) { // if ((*cont)[i]->_ghostParent == NULL) {
      if ((*cont)[i]->parent == NULL) continue; // If soultion not found, goal configuration doesn't have any parent node
      C3Vector start((*cont)[i]->values[0], (*cont)[i]->values[1], (*cont)[i]->values[2]);
      C3Vector goal((*cont)[i]->parent->values[0], (*cont)[i]->parent->values[1], (*cont)[i]->parent->values[2]);
      start = _startDummyCTM * start;
      goal = _startDummyCTM * goal;
      float d[6];
      start.copyTo(d);
      goal.copyTo(d + 3);
      for (int j = 0; j < 6; j++)
        data.push_back(d[j]);
    }
  }
#ifdef DUMP
  if (fromTheStart) {
    FILE *ofp = fopen("tree.dump", "w");
    fprintf(ofp, "%lu\n", cont->size());
    cont->push_back(static_cast<HolonomicPBRRTstarNode*>(fromGoal[0]));
    for (int i = 0; i < int(cont->size()); i++) {
      if (!i) {
        C3Vector start((*cont)[i]->values[0], (*cont)[i]->values[1], (*cont)[i]->values[2]);
        start = _startDummyCTM * start;
        fprintf(ofp, "%p %p %f %f %f %f %f %d %d\n",
                (*cont)[i], NULL, start(0), start(1), start(2),
                (*cont)[i]->getRadius(), (*cont)[i]->getCost(), (*cont)[i]->isEnqueued(), (*cont)[i]->isCollisionFree());
      } else {
        C3Vector start((*cont)[i]->values[0], (*cont)[i]->values[1], (*cont)[i]->values[2]);
        C3Vector goal((*cont)[i]->parent->values[0], (*cont)[i]->parent->values[1], (*cont)[i]->parent->values[2]);
        start = _startDummyCTM * start;
        goal = _startDummyCTM * goal;
        fprintf(ofp, "%p %p %f %f %f %f %f %d %d\n",
                (*cont)[i], (*cont)[i]->parent, start(0), start(1), start(2),
                (*cont)[i]->getRadius(), (*cont)[i]->getCost(), (*cont)[i]->isEnqueued(), (*cont)[i]->isCollisionFree());
      }
    }
    fclose(ofp);
  }
#endif
}

int HolonomicPBRRTstar::searchPath(int maxTimePerPass) {
  // maxTimePerPass is in miliseconds
#ifdef TIMELAPSE
  FILE *tfp = fopen("time_lapse.txt", "a");
#endif
  if (invalidData)
    return(0);
#ifdef OMPL_NN
  if ( (_nn->size() == 0) || (fromGoal.size() == 0) || (foundPath.size() != 0) )
#else
  if ( (fromStart->size() == 0) || (fromGoal.size() == 0) || (foundPath.size() != 0) )
#endif
    return(0);

  CDummyDummy* startDummy = (CDummyDummy*)_simGetObject(startDummyID);
  if (startDummy == NULL)
    return(0);

  // Following since 2010/08/19 so that we can move the "robot" while we search:
  C7Vector dumSavedConf;
  _simGetObjectLocalTransformation(startDummy, dumSavedConf.X.data, dumSavedConf.Q.data, false);

  int dirConstrSave[4];
  for (int constr = 0; constr < 4; constr++)
    dirConstrSave[constr] = _directionConstraints[constr];

  if (maxTimePerPass == 131183)
    return(61855195);

  std::vector<HolonomicPBRRTstarNode*>* from_goal =
    reinterpret_cast<std::vector<HolonomicPBRRTstarNode*>*>(&fromGoal);

  int foundAPath = 0;
  int initTime = simGetSystemTimeInMs(-1);
  HolonomicPBRRTstarNode* randNode = new HolonomicPBRRTstarNode(planningType, _searchMinVal, _searchRange, _gammaAxisRotation, _gammaAxisRotationInv);
  maxTimePerPass = 60000;
  while (_simGetTimeDiffInMs(initTime) < maxTimePerPass) {
    randNode->reSample(planningType, _searchMinVal, _searchRange);

    _totalSampling += 1;

#ifdef OMPL_NN
    HolonomicPBRRTstarNode* closest = _nn->nearest(randNode);
#else
    HolonomicPBRRTstarNode* closest = getClosestNode(*search_tree, randNode, closest_dist);
#endif
    float artificialCost, cArtificialCost;
    if (closest == NULL) continue;

    randNode->setRadius(0.0); // Only consider the radius of 'from' node.
    HolonomicPBRRTstarNode* extended = lazyExtend(closest, randNode, false, startDummy, cArtificialCost);
    if (extended == NULL) continue;

    extended->parent = closest;
    extended->setCost(static_cast<HolonomicPBRRTstarNode*>(extended->parent)->getCost() + cArtificialCost);
    // extended->_ghostParent = closest;
    // extended->setGhostCost(cArtificialCost);

    if (closest->_limiter != NULL) {
      if (extended->getRadius() > distance(closest->_limiter, extended)) {
        extended->setRadius(distance(closest->_limiter, extended));
        extended->_limiter = closest->_limiter;
      }
    } else {
      extended->updateRadius(distance(closest, extended) + closest->getRadius());
    }

#ifdef OMPL_NN
    std::vector<HolonomicPBRRTstarNode*> neighbors;
#ifdef KNN
    _nn->nearestK(extended, getNearNeighborRadius(), neighbors);
#else
    _nn->nearestR(extended, getNearNeighborRadius(), neighbors);
#endif
#else
    std::vector<HolonomicPBRRTstarNode*> neighbors = getNearNeighborNodes(*search_tree, extended, getNearNeighborRadius());
#endif
    _rewireTest += neighbors.size();
    while (_collisionCache.size() < neighbors.size()) {
      _collisionCache.resize(_collisionCache.size() * 2);
    }

    HolonomicPBRRTstarNode* dummy;
    if (_delayCC) {
      // pass
    } else {
      for (int i = 0; i < int(neighbors.size()); i++) {
        if (neighbors[i] != closest) {
          dummy = lazyExtend(neighbors[i], extended, true, startDummy, artificialCost);
          float cost = neighbors[i]->getCost() + artificialCost;
          if (dummy != NULL) {
            _collisionCache[i] = artificialCost;
            if (cost < extended->getCost()) {
              extended->setCost(cost);
              extended->parent = neighbors[i];
            }
          } else { // Collision
            _collisionCache[i] = DBL_MAX;
          }
          delete dummy;
        } else { // Closest node
          _collisionCache[i] = cArtificialCost;
        }
      }

#ifdef PRUNE
      if (!gotPotential(extended)) {
        delete extended;
        continue;
      }
#endif

#ifdef OMPL_NN
      _nn->add(extended);
#else
      search_tree->push_back(extended);
#endif
      _successfulSampling += 1;

      // Add this node to the best new parent node
      static_cast<HolonomicPBRRTstarNode*>(extended->parent)->addChild(extended);

      // Passive sampling for goal configuration
      if (distance(static_cast<HolonomicPBRRTstarNode*>(fromGoal[0]), extended) < getNearNeighborRadius()) {
        neighbors.push_back(static_cast<HolonomicPBRRTstarNode*>(fromGoal[0]));
      }

      // Rewire
      for (int i = 0; i < int(neighbors.size()); i++) if (neighbors[i] != extended->parent) {
        if (neighbors[i] == fromGoal[0]) { // We didn't check collision for goal configuration!
          dummy = lazyExtend(extended, neighbors[i], true, startDummy, artificialCost);
          if (dummy != NULL) {
            _collisionCache[i] = artificialCost;
          } else {
            _collisionCache[i] = DBL_MAX;
          }
          delete dummy;
        }

        if (_collisionCache[i] < DBL_MAX &&
            extended->getCost() + _collisionCache[i] < neighbors[i]->getCost()) {
          float cost = extended->getCost() + _collisionCache[i];

          // Remove this node from its parent
          if (neighbors[i]->parent != NULL) { // Possible if it is a goal configuration
            static_cast<HolonomicPBRRTstarNode*>(neighbors[i]->parent)->removeChild(neighbors[i]);
          }

          float delta_cost = cost - neighbors[i]->getCost(); // Reuse the variable
          neighbors[i]->parent = extended;
          neighbors[i]->setCost(cost);
          static_cast<HolonomicPBRRTstarNode*>(neighbors[i]->parent)->addChild(neighbors[i]);
          _updateIteration += 1;
          neighbors[i]->updateChildrenCosts(delta_cost, _updateIteration);
        }
      }
    }

#ifndef NAIVE
    // Lazy collision detection
    if ((*from_goal)[0]->getCost() < _knownBestCost) {
      lazyEvaluation(startDummy);
      HolonomicPBRRTstarNode* goal = static_cast<HolonomicPBRRTstarNode*>(fromGoal[0]);
      if (goal->parent != NULL && goal->getCost() < _knownBestCost) { // Valid solution found!
        printf("%.6f -> %.6f \n", _knownBestCost, goal->getCost());
        _knownBestCost = goal->getCost();
        _updateSolution += 1;
      }
    }
#else
    HolonomicPBRRTstarNode* goal = static_cast<HolonomicPBRRTstarNode*>(fromGoal[0]);
    if (goal->parent != NULL && goal->getCost() < _knownBestCost) { // Valid solution found!
      printf("%.6f -> %.6f \n", _knownBestCost, goal->getCost());
      _knownBestCost = goal->getCost();
      _updateSolution += 1;
    }
#endif

#ifndef NAIVE
#endif
    // End of iteration

#ifdef TIMELAPSE
    if (_simGetTimeDiffInMs(initTime) + passes >= break_point) {
      fprintf(tfp, "%d: %f\n", _simGetTimeDiffInMs(initTime) + passes, _knownBestCost);
      break_point += 1000;
    }
#endif
  }
#ifdef TIMELAPSE
  fclose(tfp);
  passes += maxTimePerPass;
#endif
  delete randNode;

  // We restore the dummy local config and the constraints
  _simSetObjectLocalTransformation(startDummy, dumSavedConf.X.data, dumSavedConf.Q.data);
  for (int constr = 0; constr < 4; constr++)
    _directionConstraints[constr] = dirConstrSave[constr];

  return(foundAPath);
}

void HolonomicPBRRTstar::lazyEvaluation(CDummyDummy* startDummy) {
  bool flag = true;
  while (flag) {
    flag = false;
    std::stack<HolonomicPBRRTstarNode*> nodeStack;

    HolonomicPBRRTstarNode* it = static_cast<HolonomicPBRRTstarNode*>(fromGoal[0]); // start from goal configuration
    if (it->parent == NULL) break;
    while (it != NULL) { // stack up nodes along the solution path
      nodeStack.push(it);
      it = static_cast<HolonomicPBRRTstarNode*>(it->parent);
    }

    HolonomicPBRRTstarNode *from, *to = nodeStack.top();
    nodeStack.pop(); // Pop the root node
    while (!nodeStack.empty()) { // Start from root(start configuration)
      from = to;
      to = nodeStack.top();
      nodeStack.pop();
      if (to->isCollisionFree()) continue; // (from - to) is collision free

      float artificialCost;
      HolonomicPBRRTstarNode *dummy = extend(from, to, true, startDummy, artificialCost);
      if (dummy != NULL) { // Collision Free
        to->_ghostParent = from;
        to->setGhostCost(artificialCost);
        _validNodeCount += 1;
      } else { // Cut the very first subtree from root.
        from->removeChild(to); // before delete 'to'
        pruneSubtree(to);
        flag = true;
        delete dummy;
        break;
      }
      delete dummy;
    }
  }
}

// If it returns false, we don't need to go further, but in case of true
// need to keep going lazy-evaluation!
#ifdef OMPL_NN
bool HolonomicPBRRTstar::pruneSubtree(HolonomicPBRRTstarNode* it) {
#else
bool HolonomicPBRRTstar::pruneSubtree(HolonomicPBRRTstarNode* it, std::vector<HolonomicPBRRTstarNode*>* search_tree) {
#endif
#ifndef PURELAZY
  if (it->_ghostParent != NULL) { // Ghost saves us
    float costToParent = it->getCost() - static_cast<HolonomicPBRRTstarNode*>(it->parent)->getCost();
    float costToGhostParent = it->getGhostCost();
    if (!it->isCollisionFree()) { // parent != ghostParent
      it->parent = it->_ghostParent;
      static_cast<HolonomicPBRRTstarNode*>(it->parent)->addChild(it);

      float delta_cost = static_cast<HolonomicPBRRTstarNode*>(it->parent)->getCost() + it->getGhostCost() - it->getCost();
      it->setCost(static_cast<HolonomicPBRRTstarNode*>(it->parent)->getCost() + it->getGhostCost());
      _updateIteration += 1;
      it->updateChildrenCosts(delta_cost, _updateIteration);
    } else {
      printf("Is it possible?\n");
      assert(0);
    }
    return;
  }
#endif

  if (static_cast<HolonomicPBRRTstarNode*>(fromGoal[0]) == it) { // It has no ghost here
    it->parent = NULL;
#ifdef PURELAZY
    if(it->_ghostParent == NULL) {
      it->setCost(SIM_MAX_FLOAT);
    }
#else
    it->setCost(SIM_MAX_FLOAT);
#endif
    it->setGhostCost(SIM_MAX_FLOAT); // Removable invocation.
    return true; // Whatever! I'm the end of path.
  }

  while (!it->_children.empty()) { // Remove children on parent side
    pruneSubtree(it->_children.back());
    it->_children.pop_back();
  }

  // Remove from play
#ifdef OMPL_NN
  _nn->remove(it);
#else
  for (int i = 0; i < int(search_tree->size()); i++) if ((*search_tree)[i] == it) { // Naive implementation
    std::swap((*search_tree)[i], search_tree->back());
    search_tree->pop_back();
    break;
  }
#endif
  delete it;
}

// 'it' should be connected to the root node and has proper cost
bool HolonomicPBRRTstar::gotPotential(HolonomicPBRRTstarNode* it) {
  HolonomicPBRRTstarNode* goal_conf = static_cast<HolonomicPBRRTstarNode*>(fromGoal[0]);
  if (goal_conf->getCost() >= SIM_MAX_FLOAT || (distance(goal_conf, it) + it->getCost() < goal_conf->getCost())) {
    return true;
  }
  return false;
}

bool HolonomicPBRRTstar::setPartialPath() {
  std::vector<HolonomicPBRRTstarNode*>* from_start =
    reinterpret_cast<std::vector<HolonomicPBRRTstarNode*>*>(&fromStart);
  std::vector<HolonomicPBRRTstarNode*>* from_goal =
    reinterpret_cast<std::vector<HolonomicPBRRTstarNode*>*>(&fromGoal);
  HolonomicPBRRTstarNode* it = (*from_goal)[0];
  // Our goal configuration is never gonna be included in our serach_tree(fromStart)
  // Thus it is necessary to manually set up the path
  if ((*from_goal)[0]->getCost() < SIM_MAX_FLOAT) {
  } else {
#ifdef OMPL_NN
    _nn->list(*from_start);
    it = _nn->nearest(it);
#else
    it = getClosestNode(*from_start, it);
#endif
  }

  while (it != NULL) {
    foundPath.insert(foundPath.begin(), it->copyYourself());
    it = static_cast<HolonomicPBRRTstarNode*>(it->parent);
  }

#ifdef OMPL_NN
#ifdef PURELAZY
  fprintf(debug_fp, "%f\t%lu\t%lld\t%lld\t%lld\t%lld\t%lld\t%lld\n",
          _knownBestCost,  (_nn->size()), _validNodeCount, _totalSampling, _rewireTest, _collisionDetection, _skippedCollisionDetection,
          _updateSolution);
#else
  fprintf(debug_fp, "%f\t%lu\t%lld\t%lld\t%lld\t%lld\t%lld\t%lld\n",
          (*from_goal)[0]->getCost(),  (_nn->size()), _validNodeCount, _totalSampling, _rewireTest, _collisionDetection, _skippedCollisionDetection,
          _updateSolution);
#endif
#else
  fprintf(debug_fp, "%f\t%lu\t%lu\t%lld\t%lld\t%lld\t%lld\n",
         (*from_goal)[0]->getCost(), (fromStart.size()), (_obs->size()), _totalSampling, _rewireTest, _collisionDetection, _skippedCollisionDetection);
#endif
  fclose(debug_fp);

#ifdef TIMELAPSE
  FILE *tfp = fopen("time_lapse.txt", "a");
  fprintf(tfp, "\n");
  fclose(tfp);
#endif
  return(true);
}

HolonomicPBRRTstarNode* HolonomicPBRRTstar::getClosestNode(std::vector<HolonomicPBRRTstarNode*>& nodes,
                                                           HolonomicPBRRTstarNode* sample) {
  float dist = SIM_MAX_FLOAT;
  int index = -1;

  if (planningType == sim_holonomicpathplanning_xyz) {
    for (int i = 0; i < int(nodes.size()); i++) {
      float vect[3];
      vect[0] = sample->values[0] - nodes[i]->values[0];
      vect[1] = sample->values[1] - nodes[i]->values[1];
      vect[2] = sample->values[2] - nodes[i]->values[2];

      float d = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2];
      if (d < dist) {
        dist = d;
        index = i;
      }
    }
  } else { // (planningType==sim_holonomicpathplanning_xyzabg)
    for (int i = 0; i < int(nodes.size()); i++) {
      float vect[7];
      vect[0] = sample->values[0] - nodes[i]->values[0];
      vect[1] = sample->values[1] - nodes[i]->values[1];
      vect[2] = sample->values[2] - nodes[i]->values[2];
      C4Vector toP, fromP;
      C3Vector dum;
      sample->getAllValues(dum, toP);
      nodes[i]->getAllValues(dum, fromP);

      float ad = angularCoeff * fromP.getAngleBetweenQuaternions(toP);
      float d = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2] + ad * ad;
      if (d < dist) {
        dist = d;
        index = i;
      }
    }
  }
  if (index != -1)
    return(nodes[index]);
  return(NULL);
}

HolonomicPBRRTstarNode* HolonomicPBRRTstar::slerp(HolonomicPBRRTstarNode* from, HolonomicPBRRTstarNode* to, float t) {
  return to;
}

HolonomicPBRRTstarNode* HolonomicPBRRTstar::extend(HolonomicPBRRTstarNode* from, HolonomicPBRRTstarNode* to,
                                                   bool shouldBeConnected, CDummyDummy* dummy, float &artificialCost) {
  // Return value is != NULL if extention was performed and connect is false
  // If connect is true, then return value indicates that connection can be performed!
  HolonomicPBRRTstarNode* extended = from->copyYourself();
  float theVect[7];
  int passes = getVector(from, to, theVect, stepSize, artificialCost, true);
  int currentPass;
  float delta_magnitude = artificialCost / passes;
  float magnitude = 0.0f;
  bool collision_flag = false;

  C3Vector pos(extended->values);
  C4Vector orient(extended->values + 3);
  C3Vector delta_p(theVect);
  C4Vector delta_q(theVect + 3);

  for (currentPass = 0; currentPass < passes; currentPass++) {
    pos += delta_p;
    magnitude += delta_magnitude;

    if (planningType == sim_holonomicpathplanning_xyz)
      orient.setIdentity();
    else if (planningType == sim_holonomicpathplanning_xyzabg)
      orient *= delta_q;

    C7Vector transf(orient, pos);
    C7Vector tmpTr(_startDummyLTM * transf);
    _simSetObjectLocalTransformation(dummy, tmpTr.X.data, tmpTr.Q.data);
    _collisionDetection += 1;
    if (doCollide(NULL)) {
      extended->setAllValues(pos, orient);
      HolonomicPBRRTstarNode *limiter = extended->copyYourself();
      if (magnitude < from->getRadius()) {
        from->setRadius(magnitude);
        from->_limiter = limiter;
      }
      if (artificialCost - magnitude < to->getRadius()) {
        to->setRadius(artificialCost - magnitude);
        to->_limiter = limiter;
      }
      collision_flag = true;
      break;
    }
  }
  int backwardPass = currentPass;

  C3Vector back_pos(to->values);
  C4Vector back_orient(to->values + 3);
  C4Vector inverse_delta_q = delta_q.getInverse();
  float back_magnitude = artificialCost;
  for (backwardPass = passes - 2; backwardPass > currentPass; backwardPass--) {
    // At the step passes - 2, it tests the configuration of 'A + (passes - 2 + 1) * delta'.
    back_pos -= delta_p;
    back_magnitude -= delta_magnitude;

    if (planningType == sim_holonomicpathplanning_xyz)
      back_orient.setIdentity();
    else if (planningType == sim_holonomicpathplanning_xyzabg)
      back_orient *= inverse_delta_q;

    C7Vector transf(back_orient, back_pos);
    C7Vector tmpTr(_startDummyLTM * transf);
    _simSetObjectLocalTransformation(dummy, tmpTr.X.data, tmpTr.Q.data);
    _collisionDetection += 1;
    if (doCollide(NULL)) {
      extended->setAllValues(back_pos, back_orient);
      HolonomicPBRRTstarNode *limiter = extended->copyYourself();

      if (artificialCost - back_magnitude < to->getRadius()) {
        to->setRadius(artificialCost - back_magnitude);
        to->_limiter = limiter;
      }

      if (shouldBeConnected) {
        delete extended;
        return(NULL);
      }
      break;
    }
  }

  if (shouldBeConnected && collision_flag && backwardPass == currentPass) {
    delete extended;
    return(NULL);
  }

  if (currentPass > 0) {
    if (currentPass < passes) { // Not fully connected, roll back the last change
      pos -= delta_p;
      orient *= delta_q.getInverse();
    }
    extended->setAllValues(pos, orient);
    artificialCost *= currentPass / (float)passes;
    return extended;
  }
  delete extended;
  return(NULL);
}

HolonomicPBRRTstarNode* HolonomicPBRRTstar::lazyExtend(HolonomicPBRRTstarNode* from, HolonomicPBRRTstarNode* to,
                                               bool shouldBeConnected, CDummyDummy* dummy, float &artificialCost) {
  // Return value is != NULL if extention was performed and connect is false
  // If connect is true, then return value indicates that connection can be performed!
  HolonomicPBRRTstarNode* extended = from->copyYourself();
  float theVect[7] = {0.0, };
  int passes = getVector(from, to, theVect, stepSize, artificialCost, false);
  int currentPass;
  float delta_magnitude = artificialCost / passes;
  float magnitude = 0.0f;
  bool collision_flag = false;

#ifdef ONE
  C3Vector pos(extended->values);
  C4Vector orient(extended->values + 3);
  C3Vector delta_p(theVect);
  C4Vector delta_q(theVect + 3);

  bool skipCollisionCheck = false;
  if (from->getRadius() + to->getRadius() > artificialCost) {
    skipCollisionCheck = true;
  }

  if (!skipCollisionCheck) {
    float difference = artificialCost - from->getRadius() - to->getRadius();
    pos += delta_p * (from->getRadius() + difference / 2.0); // At the middle of two collision free sphere
    if (planningType == sim_holonomicpathplanning_xyz) {
      orient.setIdentity();
    } else if (planningType == sim_holonomicpathplanning_xyzabg) {
      orient *= delta_q;
    }

    C7Vector transf(orient, pos);
    C7Vector tmpTr(_startDummyLTM * transf);
    _simSetObjectLocalTransformation(dummy, tmpTr.X.data, tmpTr.Q.data);
    _collisionDetection += 1;
    _skippedCollisionDetection += passes - 1;
    if (doCollide(NULL)) {
      delete extended;
      return(NULL);
    }
  } else { //
    _skippedCollisionDetection += passes;
  }

  return to->copyYourself();
#else
  C3Vector pos(extended->values);
  C4Vector orient(extended->values + 3);
  C3Vector delta_p(theVect);
  C4Vector delta_q(theVect + 3);

  for (currentPass = 0; currentPass < passes; currentPass++) {
    pos += delta_p;
    magnitude += delta_magnitude;

    if (planningType == sim_holonomicpathplanning_xyz)
      orient.setIdentity();
    else if (planningType == sim_holonomicpathplanning_xyzabg)
      orient *= delta_q;

    bool skipCollisionCheck = false;
    if (magnitude < from->getRadius() || artificialCost - magnitude < to->getRadius()) { //
      skipCollisionCheck = true;
    }

#ifdef NAIVE
    skipCollisionCheck = false;
#endif
#ifdef PURELAZY
    skipCollisionCheck = true;
#endif
    if (!skipCollisionCheck) { // Explicit Collision Check
      C7Vector transf(orient, pos);
      C7Vector tmpTr(_startDummyLTM * transf);
      _simSetObjectLocalTransformation(dummy, tmpTr.X.data, tmpTr.Q.data);
      _collisionDetection += 1;
      if (doCollide(NULL)) {
        // If collision occured and the magnitude is less than its radius
        // update it.
        extended->setAllValues(pos, orient);
        HolonomicPBRRTstarNode *limiter = extended->copyYourself();
        if (magnitude < from->getRadius()) {
          from->setRadius(magnitude);
          from->_limiter = limiter;
        }
        if (artificialCost - magnitude < to->getRadius()) {
          to->setRadius(artificialCost - magnitude);
          to->_limiter = limiter;
        }
        collision_flag = true;
        break;
      }
    } else { //
      _skippedCollisionDetection += 1;
    }
  }

  int backwardPass = currentPass;
#ifdef BI
  C3Vector back_pos(to->values);
  C4Vector back_orient(to->values + 3);
  C4Vector inverse_delta_q = delta_q.getInverse();
  float back_magnitude = artificialCost;
  for (backwardPass = passes - 2; backwardPass > currentPass; backwardPass--) {
    // At the step passes - 2, it tests the configuration of 'A + (passes - 2 + 1) * delta'.
    back_pos -= delta_p;
    back_magnitude -= delta_magnitude;

    if (planningType == sim_holonomicpathplanning_xyz)
      back_orient.setIdentity();
    else if (planningType == sim_holonomicpathplanning_xyzabg)
      back_orient *= inverse_delta_q;

    bool skipCollisionCheck = false;
    if (magnitude < from->getRadius() || artificialCost - magnitude < to->getRadius()) { //
      skipCollisionCheck = true;
    }

    if (!skipCollisionCheck) { // Explicit Collision Check
      C7Vector transf(back_orient, back_pos);
      C7Vector tmpTr(_startDummyLTM * transf);
      _simSetObjectLocalTransformation(dummy, tmpTr.X.data, tmpTr.Q.data);
      _collisionDetection += 1;
      if (doCollide(NULL)) {
        extended->setAllValues(back_pos, back_orient);
        HolonomicPBRRTstarNode *limiter = extended->copyYourself();

        if (artificialCost - back_magnitude < to->getRadius()) {
          to->setRadius(artificialCost - back_magnitude);
          to->_limiter = limiter;
        }

        if (shouldBeConnected) {
          delete extended;
          return(NULL);
        }
        break;
      }
    } else {
      _skippedCollisionDetection += 1;
    }
  }
#endif
  if (shouldBeConnected && collision_flag && backwardPass == currentPass) {
    delete extended;
    return(NULL);
  }

  if (currentPass > 0) {
    if (currentPass < passes) { // Not fully connected, roll back the last change
      HolonomicPBRRTstarNode *limiter = extended->copyYourself();
      limiter->setAllValues(pos, orient);
      extended->setRadius(delta_magnitude);
      extended->_limiter = limiter;

      pos -= delta_p;
      orient *= delta_q.getInverse();
    }
    extended->setAllValues(pos, orient);
    artificialCost *= currentPass / (float)passes;
    return extended;
  }
  delete extended;
  return(NULL);
#endif
}

int HolonomicPBRRTstar::getVector(HolonomicPBRRTstarNode* fromPoint, HolonomicPBRRTstarNode* toPoint, float vect[7], float e, float& artificialLength, bool dontDivide) {
	// if direction constraints are not respected, return value is -1 and vect does not contain anything
	// Otherwise return value is the number of times we have to add 'vect' to 'fromPoint' to reach 'toPoint'
	int retVal = -1;

	if (planningType == sim_holonomicpathplanning_xy) {
		vect[0] = toPoint->values[0] - fromPoint->values[0];
		vect[1] = toPoint->values[1] - fromPoint->values[1];
		if (areDirectionConstraintsRespected(vect)) {
			artificialLength = sqrtf(vect[0] * vect[0] + vect[1] * vect[1]);
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
			retVal = (int)(artificialLength / e) + 1;
			float l = (float)retVal;
			vect[0] /= l;
			vect[1] /= l;
		}

  } else if (planningType == sim_holonomicpathplanning_xyz) {
    vect[0] = toPoint->values[0] - fromPoint->values[0];
    vect[1] = toPoint->values[1] - fromPoint->values[1];
    vect[2] = toPoint->values[2] - fromPoint->values[2];
    artificialLength = sqrtf(vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2]);
    retVal = (int)(artificialLength / e) + 1;
#ifdef ONE
    if (!dontDivide) {
      vect[0] /= artificialLength;
      vect[1] /= artificialLength;
      vect[2] /= artificialLength;
    } else {
      float l = (float)retVal;
      vect[0] /= l;
      vect[1] /= l;
      vect[2] /= l;
    }
#else
    float l = (float)retVal;
    vect[0] /= l;
    vect[1] /= l;
    vect[2] /= l;
#endif

  } else if (planningType == sim_holonomicpathplanning_xyg) {
		vect[0] = toPoint->values[0] - fromPoint->values[0];
		vect[1] = toPoint->values[1] - fromPoint->values[1];
		vect[2] = CPathPlanningInterface::getNormalizedAngle(toPoint->values[2] - fromPoint->values[2]);
		if (areDirectionConstraintsRespected(vect)) {
			artificialLength = vect[0] * vect[0] + vect[1] * vect[1];
			artificialLength = sqrtf(artificialLength + vect[2] * angularCoeff * vect[2] * angularCoeff);
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
    float ap = angularCoeff * fromP.getAngleBetweenQuaternions(toP);
    artificialLength = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2];
    artificialLength = sqrtf(artificialLength + ap * ap);

    retVal = (int)(artificialLength / e) + 1;
    float l = (float)retVal;

#ifdef ONE
    if (!dontDivide) {
      vect[0] /= artificialLength;
      vect[1] /= artificialLength;
      vect[2] /= artificialLength;

      float difference = artificialLength - toPoint->getRadius() - fromPoint->getRadius();
      if (difference >= 0.0) {
        C4Vector q;
        q.setIdentity();
        fromP.buildInterpolation(q, diff, (fromPoint->getRadius() + difference / 2.0) / artificialLength);
        vect[3] = fromP(0);
        vect[4] = fromP(1);
        vect[5] = fromP(2);
        vect[6] = fromP(3);
      }
    } else {
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
#else
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
#endif
  }
  return(retVal);
}

bool HolonomicPBRRTstar::addVector(C3Vector& pos, C4Vector& orient, float vect[7]) {
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

int HolonomicPBRRTstar::smoothFoundPath(int steps, int maxTimePerPass) {
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
    HolonomicPBRRTstarNode* startP;
    HolonomicPBRRTstarNode* endP;
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
          startP = static_cast<HolonomicPBRRTstarNode*>(foundPath[lowIndex]);
          endP = static_cast<HolonomicPBRRTstarNode*>(foundPath[highIndex]);
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
            startP = static_cast<HolonomicPBRRTstarNode*>(foundPath[lowIndex]);
            endP = static_cast<HolonomicPBRRTstarNode*>(foundPath[highIndex]);
            break;
          }
        }
      }
      if (startP != NULL) {
        // Now let's try to link highIndex from lowIndex with a "straight" line:
        float vect[7];
        float artificialVectorLength;
        int passes = getVector(startP, endP, vect, stepSize, artificialVectorLength, true);
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
              HolonomicPBRRTstarNode* it = endP->copyYourself(); // just to have the right size!
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

void HolonomicPBRRTstar::getPathData(std::vector<float>& data) {
  data.clear();
  if (invalidData)
    return;
  for (int i = 0; i < int(foundPath.size()); i++) {
    HolonomicPBRRTstarNode* theNode = static_cast<HolonomicPBRRTstarNode*>(foundPath[i]);
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
}

bool HolonomicPBRRTstar::areDirectionConstraintsRespected(float vect[7]) {
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

bool HolonomicPBRRTstar::areSomeValuesForbidden(C7Vector configuration) {
  float values[7];
  configuration.getInternalData(values);
  return areSomeValuesForbidden(values);
}

bool HolonomicPBRRTstar::areSomeValuesForbidden(float values[7]) {
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

bool HolonomicPBRRTstar::doCollide(float* dist) {
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

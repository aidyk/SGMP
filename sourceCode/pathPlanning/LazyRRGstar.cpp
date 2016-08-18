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

#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/datastructures/NearestNeighborsLinear.h>
#include <ompl/datastructures/NearestNeighborsFLANN.h>
#include <functional>
#include <memory>
#include <queue>

#include "LazyRRGstar.h"
#include "pathPlanningInterface.h"
#include "v_repLib.h"

#define SIM_MIN(a,b) (((a)<(b)) ? (a) : (b))
#define SIM_MAX(a,b) (((a)>(b)) ? (a) : (b))

using namespace std::placeholders;

#define CONST_E 2.718281828

// #define RRG

#define CACHING
#define DYNAMIC
#define TIME
#define ONCE
#define ANN
#define KNN
#define VIS

LazyRRGstar::LazyRRGstar(int theStartDummyID, int theGoalDummyID,
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

  _start_node = new LazyRRGstarNode(planningType, sConf, _gammaAxisRotation, _gammaAxisRotationInv);
  _goal_node = new LazyRRGstarNode(planningType, goalDummyLocalConf, _gammaAxisRotation, _gammaAxisRotationInv);

  _start_node->setCost(0);
  _goal_node->setCost(SIM_MAX_FLOAT);

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

#ifdef ANN
	_nn.reset(new ompl::NearestNeighborsFLANNHierarchicalClustering<LazyRRGstarNode*>()); // Initialize NearestNeighbors structure
#else
	_nn.reset(new ompl::NearestNeighborsGNAT<LazyRRGstarNode*>()); // Initialize NearestNeighbors structure
	// _nn.reset(new ompl::NearestNeighborsLinear<LazyRRGstarNode*>()); // Initialize NearestNeighbors structure
#endif
	_nn->setDistanceFunction((std::bind(&LazyRRGstar::distance, this, _1, _2)));
  _nn->add(_start_node);
  _nn->add(_goal_node);

  // Set ballRadiusMax and ballRadiusConst to maximum extent
  _ballRadiusMax = _ballRadiusConst = sqrt(_searchRange[0] * _searchRange[0] + _searchRange[1] * _searchRange[1] + _searchRange[2] * _searchRange[2]);
  _best_cost = SIM_MAX_FLOAT;

  _kConstant = CONST_E + CONST_E / 3.0;

  // Initialize distance to the nearest collision-free space to MAX
  _start_node->free_radius = _ballRadiusMax;
  _goal_node->free_radius = _ballRadiusMax;

  _collision_detection_count = _skipped_collision_detection_count = 0;
  _dynamic_decrease_count = _dynamic_increase_count = 0;

  _collision_detection_time = _dynamic_increase_time = _dynamic_decrease_time = 0;
  _near_neighbor_search_time = 0;
  _remove_time = 0;

  buffer[0] = -1; // What the hell is this?
  invalidData = false;
}

LazyRRGstar::~LazyRRGstar() {
  for (int i = 0; i < int(fromStart.size()); i++)
    delete fromStart[i];
  fromStart.clear();
  for (int i = 0; i < int(fromGoal.size()); i++)
    delete fromGoal[i];
  fromGoal.clear();
  for (int i = 0; i < int(foundPath.size()); i++)
    delete foundPath[i];
  foundPath.clear();
  if (_nn) {
    std::vector<LazyRRGstarNode*> node_list;
    _nn->list(node_list);
    for (int i = 0; i < int(node_list.size()); i++) {
      delete node_list[i];
    }
    // _nn->clear();
  }
}

float LazyRRGstar::getNearNeighborRadius() {
  // return SIM_MIN(_ballRadiusMax, _ballRadiusConst * pow(log(1.0 + fromStart.size()) / (1.0 + fromStart.size()), 0.33));
    //if (planningType == sim_holonomicpathplanning_xyzabg) {
    //  return SIM_MIN(_ballRadiusMax, _ballRadiusConst * pow(log(1.0 + _nn->size()) / (1.0 + _nn->size()), 0.1666));
    //} else {
#ifdef KNN
  return ceil(log(1.0 + _nn->size()) * _kConstant);
#else
  return SIM_MIN(_ballRadiusMax, _ballRadiusConst * pow(log(1.0 + _nn->size()) / (1.0 + _nn->size()), 0.3333));
#endif
  //}
}

float LazyRRGstar::distance(LazyRRGstarNode* a, LazyRRGstarNode* b) {
  float dist = 0.0;
  if (planningType == sim_holonomicpathplanning_xy) {
    float vect[2];
    vect[0] = a->values[0] - b->values[0];
    vect[1] = a->values[1] - b->values[1];

    dist = vect[0] * vect[0] + vect[1] * vect[1];

  } else if (planningType == sim_holonomicpathplanning_xyz) {
    float vect[3];
    vect[0] = a->values[0] - b->values[0];
    vect[1] = a->values[1] - b->values[1];
    vect[2] = a->values[2] - b->values[2];

    dist = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2];

  } else if (planningType == sim_holonomicpathplanning_xyg) {
    float vect[3];
    vect[0] = a->values[0] - b->values[0];
    vect[1] = a->values[1] - b->values[1];
    vect[2] = CPathPlanningInterface::getNormalizedAngle(a->values[2] - b->values[2]);
    vect[2] *= angularCoeff;

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

bool cmpVis(LazyRRGstarNode* a, LazyRRGstarNode* b) {
	return a->free_radius < b->free_radius;
}

void LazyRRGstar::getSearchTreeData(std::vector<float>& data, bool fromTheStart) {
	std::vector<LazyRRGstarNode*> cont;
	_nn->list(cont);

	if (fromTheStart) {
		if ( (planningType == sim_holonomicpathplanning_xy) || (planningType == sim_holonomicpathplanning_xyg) || (planningType == sim_holonomicpathplanning_xyabg) ) {
			for (int i = 0; i < int(cont.size()); i++) {
				if (cont[i]->pred == NULL) continue;
				C3Vector start(cont[i]->values[0], cont[i]->values[1], 0.0f);
				C3Vector goal(cont[i]->pred->values[0], cont[i]->pred->values[1], 0.0f);
				start = _startDummyCTM * start;
				goal = _startDummyCTM * goal;
				float d[6];
				start.copyTo(d);
				goal.copyTo(d + 3);
				for (int j = 0; j < 6; j++)
					data.push_back(d[j]);
			}

		} else if ( (planningType == sim_holonomicpathplanning_xg) || (planningType == sim_holonomicpathplanning_xabg) ) {
			for (int i = 1; i < int(cont.size()); i++) {
				C3Vector start(cont[i]->values[0], 0.0f, 0.0f);
				C3Vector goal(cont[i]->pred->values[0], 0.0f, 0.0f);
				start = _startDummyCTM * start;
				goal = _startDummyCTM * goal;
				float d[6];
				start.copyTo(d);
				goal.copyTo(d + 3);
				for (int j = 0; j < 6; j++)
					data.push_back(d[j]);
			}

		} else if ( (planningType == sim_holonomicpathplanning_xyz) || (planningType == sim_holonomicpathplanning_xyzg) || (planningType == sim_holonomicpathplanning_xyzabg) ) {
			for (int i = 0; i < int(cont.size()); i++) {
				if (cont[i]->pred == NULL) continue;
				C3Vector start(cont[i]->values[0], cont[i]->values[1], cont[i]->values[2]);
				C3Vector goal(cont[i]->pred->values[0], cont[i]->pred->values[1], cont[i]->pred->values[2]);
				start = _startDummyCTM * start;
				goal = _startDummyCTM * goal;
				float d[6];
				start.copyTo(d);
				goal.copyTo(d + 3);
				for (int j = 0; j < 6; j++)
					data.push_back(d[j]);
			}
		}
	} else { // !fromTheStart - Blue-colored

#ifdef VIS
		std::vector<LazyRRGstarNode*> node_list;
		_nn->list(node_list);

		std::sort(node_list.begin(), node_list.end(), cmpVis);

		float ambient_color[3] = {1.0, 1.0, 1.0}, dynamic_color[3];
		simInt point_container = simAddDrawingObject(sim_drawing_points + sim_drawing_itemcolors + sim_drawing_overlay
																								 , 4, 0.01, -1, 1000000, NULL, NULL, NULL, NULL);
		for (unsigned int i = 0; i < node_list.size(); i++) {
			LazyRRGstarNode* it = node_list[i];
			// The closer, the brighter.
			C3Vector pos;
			if (planningType == sim_holonomicpathplanning_xyg) {
				pos.set(it->values[0], it->values[1], 0.0);
			} else {
				pos.set(it->values[0], it->values[1], it->values[2]);
			}
			pos = _startDummyCTM * pos;
			float point_data[6] = {pos.data[0], pos.data[1], pos.data[2], i / (float)node_list.size(),  i / (float)node_list.size(), i / (float)node_list.size()};
			simAddDrawingObjectItem(point_container, point_data);
		}
#endif
		return;
		/*
		if ( (planningType == sim_holonomicpathplanning_xy) || (planningType == sim_holonomicpathplanning_xyg) || (planningType == sim_holonomicpathplanning_xyabg) ) {
			for (int i = 0; i < int(cont.size()); i++) {
				if (cont[i]->pred == NULL || cont[i]->isCollisionFree() || cont[i]->witness == NULL) continue;
				C3Vector start(cont[i]->values[0], cont[i]->values[1], 0.0f);
				C3Vector goal(cont[i]->witness->values[0], cont[i]->witness->values[1], 0.0f);
				start = _startDummyCTM * start;
				goal = _startDummyCTM * goal;
				float d[6];
				start.copyTo(d);
				goal.copyTo(d + 3);
				for (int j = 0; j < 6; j++)
					data.push_back(d[j]);
			}
			*/

		/*
			for (int i = 0; i < int(cont.size()); i++) {
				if (cont[i]->pred == NULL || cont[i]->isCollisionFree()) continue;
				C3Vector start(cont[i]->values[0], cont[i]->values[1], 0.0f);
				C3Vector goal(cont[i]->pred->values[0], cont[i]->pred->values[1], 0.0f);
				start = _startDummyCTM * start;
				goal = _startDummyCTM * goal;
				float d[6];
				start.copyTo(d);
				goal.copyTo(d + 3);
				for (int j = 0; j < 6; j++)
					data.push_back(d[j]);
			}
			*/
		if ( (planningType == sim_holonomicpathplanning_xg) || (planningType == sim_holonomicpathplanning_xabg) ) {
			for (int i = 1; i < int(cont.size()); i++) {
				C3Vector start(cont[i]->values[0], 0.0f, 0.0f);
				C3Vector goal(cont[i]->pred->values[0], 0.0f, 0.0f);
				start = _startDummyCTM * start;
				goal = _startDummyCTM * goal;
				float d[6];
				start.copyTo(d);
				goal.copyTo(d + 3);
				for (int j = 0; j < 6; j++)
					data.push_back(d[j]);
			}

		} else if ( (planningType == sim_holonomicpathplanning_xyz) || (planningType == sim_holonomicpathplanning_xyzg) || (planningType == sim_holonomicpathplanning_xyzabg) ) {
			for (int i = 0; i < int(cont.size()); i++) {
				/*
				if (cont[i]->pred == NULL || cont[i]->isCollisionFree()) continue;
				C3Vector start(cont[i]->values[0], cont[i]->values[1], cont[i]->values[2]);
				C3Vector goal(cont[i]->pred->values[0], cont[i]->pred->values[1], cont[i]->pred->values[2]);
				*/
				if (cont[i]->pred == NULL || cont[i]->witness == NULL) continue;
				C3Vector start(cont[i]->values[0], cont[i]->values[1], cont[i]->values[2]);
				C3Vector goal(cont[i]->witness->values[0], cont[i]->witness->values[1], cont[i]->witness->values[2]);

				start = _startDummyCTM * start;
				goal = _startDummyCTM * goal;
				float d[6];
				start.copyTo(d);
				goal.copyTo(d + 3);
				for (int j = 0; j < 6; j++)
					data.push_back(d[j]);
			}
		}
	}
}

int LazyRRGstar::searchPath(int maxTimePerPass) {
#ifdef TIME
  FILE *tfp = fopen("time_lapse.txt", "a");
  int break_point = 1000;
#endif

  printf("stepSize : %f\n", stepSize);
  printf("angularCoeff : %f\n", angularCoeff);

  // maxTimePerPass is in miliseconds
  if (invalidData)
    return(0);
  if (foundPath.size() != 0)
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
  maxTimePerPass = 10000;

  int foundAPath = 0;
  int initTime = simGetSystemTimeInMs(-1);
  LazyRRGstarNode* randNode = new LazyRRGstarNode(planningType, _searchMinVal, _searchRange, _gammaAxisRotation, _gammaAxisRotationInv);
	while (_simGetTimeDiffInMs(initTime) < _maxTimebudget) {
    LazyRRGstarNode* dummy;

#ifdef TIME
		if (_simGetTimeDiffInMs(initTime) >= break_point) {
			float bc = _goal_node->getCost();
			fprintf(tfp, "%f\t%d\t%d\t%d\t%d\n", bc, _collision_detection_count, _dynamic_increase_count,
							_collision_detection_time, _dynamic_increase_time, _dynamic_decrease_time, _near_neighbor_search_time);
			break_point += 1000;
		}
#endif

    randNode->reSample(planningType, _searchMinVal, _searchRange);
#ifdef TIME
    int elapsed_time = simGetSystemTimeInMs(-1);
#endif
    LazyRRGstarNode* closest = _nn->nearest(randNode);
#ifdef TIME
    _near_neighbor_search_time += _simGetTimeDiffInMs(elapsed_time);
#endif
    float artificialCost;
    assert(closest != NULL); // Impossible!

    if (!isFree(randNode, startDummy)) continue;

    float dist = distance(randNode, closest);
    if (dist > _ballRadiusConst * _maxDistance) {
      randNode->interpolate(closest, _ballRadiusConst * _maxDistance, angularCoeff);
    }

    LazyRRGstarNode* extended = randNode->copyYourself(); // Hm...
    std::vector<LazyRRGstarNode*> neighbors;
#ifdef TIME
    elapsed_time = simGetSystemTimeInMs(-1);
#endif
#ifdef KNN
    _nn->nearestK(extended, getNearNeighborRadius(), neighbors);
#else
    _nn->nearestR(extended, fmin(getNearNeighborRadius(), _maxDistance), neighbors);
#endif
    _nn->add(extended);
#ifdef TIME
    _near_neighbor_search_time += _simGetTimeDiffInMs(elapsed_time);
#endif

// <LazyChooseParent
    for (unsigned int i = 0; i < neighbors.size(); i++) {
#ifdef RRG
      dummy = extend(extended, neighbors[i], true, startDummy, artificialCost);
#else
      dummy = lazyExtend(extended, neighbors[i], true, startDummy, artificialCost);
#endif
      // dummy ignored

      if (dummy == NULL) continue;
      extended->addNode(neighbors[i], artificialCost);
      neighbors[i]->addNode(extended, artificialCost);

#ifdef DYNAMIC
			if (neighbors[i]->witness != NULL) {
        float dist_to_witness = distance(extended, neighbors[i]->witness);

				if (extended->witness == NULL || extended->free_radius > dist_to_witness) {
          extended->free_radius = dist_to_witness;
          extended->witness = neighbors[i]->witness;
        }
      }
#endif

      // ChoosParent
      if (neighbors[i]->getCost() + artificialCost < extended->getCost()) {
        extended->pred = neighbors[i];
        extended->setCost(neighbors[i]->getCost() + artificialCost);
      }
    }

    if (extended->pred != NULL) { // Update children infomation later.
      extended->pred->addChild(extended);
    }

// >
#ifdef TIME
    elapsed_time = simGetSystemTimeInMs(-1);
#endif
    DynamicDecrease(extended); // Similar to UpdateChildren
#ifdef TIME
    _dynamic_decrease_time += _simGetTimeDiffInMs(elapsed_time);
#endif
    DynamicShortestPathUpdate(startDummy); // LazyUpdate including DynamicIncrease
  }
  delete randNode;

#ifdef TIME
  fprintf(tfp, "\n");
  fclose(tfp);
#endif
  // We restore the dummy local config and the constraints
  _simSetObjectLocalTransformation(startDummy, dumSavedConf.X.data, dumSavedConf.Q.data);
  for (int constr = 0; constr < 4; constr++)
    _directionConstraints[constr] = dirConstrSave[constr];

  return(foundAPath);
}

void LazyRRGstar::DynamicShortestPathUpdate(CDummyDummy *startDummy) {
  LazyRRGstarNode *tracer = _goal_node;
  vector<LazyRRGstarNode*> solution_path;
  if (_goal_node->getCost() >= _best_cost) return; // No better 'potential' solution path found this iteration

  // Cutting point priority? from start or goal?
  // From start is more plausible, stable and would be faster!

  while (true) {
    int i;
    LazyRRGstarNode *from, *to;

    tracer = _goal_node;
    solution_path.clear();
    while (tracer != NULL) {
      solution_path.push_back(tracer);
      tracer = tracer->pred;
    }

    for (i = solution_path.size() - 1; i >= 1; i--) {
      from = solution_path[i];
      to = solution_path[i- 1];

#ifdef CACHING
      if (to->isCollisionFree()) continue;
#endif
      float artificialCost;
      LazyRRGstarNode* dummy = extend(from, to, true, startDummy, artificialCost);
      if (dummy == NULL) { // collision found
        // Remove invalid edge and handle other things to maintain the shortestpath tree.
        from->removeNode(to);
        from->removeChild(to);

        to->pred = NULL;
        to->isCollisionFree(false);
        to->removeNode(from);

#ifdef TIME
        int elapsed_time = simGetSystemTimeInMs(-1);
#endif
        DynamicIncrease(from, to);
#ifdef TIME
        _dynamic_increase_time += _simGetTimeDiffInMs(elapsed_time);
#endif
        break;
      } else {
        to->isCollisionFree(true);
      }
    }

    if (i == 0) { // If there is no collision along the solution path.
			if (_goal_node->getCost() < _best_cost) {
        printf("%f -> %f\n", _best_cost, _goal_node->getCost());
				_best_cost = _goal_node->getCost();
			}
      break;
    } else if (to->pred == NULL) { // We couldn't connect 'something from root' - to.
      // There is no way to get a solution now.
      // Need more samples, drop by next time!
      // ToDo: is it right?
      break;
    }
  }
}

// Pre-condition : Deprecated, Use DynamicIncrease instead.
void LazyRRGstar::DynamicDelete(LazyRRGstarNode *from, LazyRRGstarNode *to) {
}

// Pre-condition : A newly added node with uninitialized cost(max-value) connected
// to its near-neighbors. Rewire-similar work is done here.
// It propagtates cost changes downstream from the initial input node.
void LazyRRGstar::DynamicDecrease(LazyRRGstarNode *node) {
  typedef pair<float, LazyRRGstarNode*> weight_node;
  std::priority_queue<weight_node> pq; // Like max-heap (by default)

  _dynamic_decrease_count += 1;

  pq.push(weight_node(-node->getCost(), node));
  while (!pq.empty()) {
    weight_node top = pq.top();
    pq.pop();

    float cost = -top.first; // Inverse the cost value for working as min-heap.
    LazyRRGstarNode* node = top.second;

    if (cost > node->getCost()) continue;

    std::vector<LazyRRGstarNode::Edge> &edges = node->edges();
    for (unsigned int i = 0; i < edges.size(); i++) { // Rewire-similar
      LazyRRGstarNode *neighbor = edges[i].node();
      float c = edges[i].cost();
      if (neighbor->getCost() > node->getCost() + c) {
        neighbor->setCost(node->getCost() + c);

        if (neighbor->pred != NULL) {
          neighbor->pred->removeChild((neighbor));
        }
        neighbor->pred = node;
        neighbor->isCollisionFree(false);
        node->addChild(neighbor);

        pq.push(weight_node(-neighbor->getCost(), neighbor));
      }
    }
  }
}

// Invoked after weight of an edge (q, z) and D(z) are updated.
void LazyRRGstar::DynamicIncrease(LazyRRGstarNode *from, LazyRRGstarNode *to) {
  static int is_white = 0;
  // A color code of a node whose color value is
  // equal to or less than 'is_white' means 'white', 'red' otherwise.
  is_white += 1;

  _dynamic_increase_count += 1;

  std::vector<LazyRRGstarNode*> reds;
  typedef pair<float, LazyRRGstarNode*> weight_node;
	priority_queue<weight_node> pq; // Like max-heap (by default)
	pq.push(weight_node(-to->getCost(), to));

  // <Step 2. Coloring
  while (!pq.empty()) {
    weight_node top = pq.top();
    pq.pop();

		float cost = -top.first;
    LazyRRGstarNode* node = top.second;

    if (cost > node->getCost()) continue; // Instead of heap_improve

    // If there exists a non-red neighbor q of z such that D(q) + w_(q,z) = D(z)
    // set pink (Don't care at this time)
    // otherwise, set red and enqueue all the children of z

    bool pink_flag = false;
    std::vector<LazyRRGstarNode::Edge> &edges = node->edges();
    for (unsigned int i = 0; i < edges.size(); i++) {
      LazyRRGstarNode *neighbor = edges[i].node();
      float c = edges[i].cost();

      if (neighbor->color != is_white + 1 && neighbor->getCost() + c == cost) {
        // Actually, '<' should not be happened all the time.
        // and even '==' would very rarely occur, but theoretically possible.
        // Set pink
        pink_flag = true;

        if (node->pred != NULL) {
          node->pred->removeChild(node);
        }
        node->pred = neighbor;
        node->isCollisionFree(false);
        neighbor->addChild(node);
      }
    }
    if (pink_flag) continue;

    node->color = is_white + 1; // Set 'red'
    reds.push_back(node);
    std::vector<LazyRRGstarNode*> &children = node->children();
    for (unsigned int i = 0; i < children.size(); i++) {
      LazyRRGstarNode *child = children[i];
			pq.push(weight_node(-child->getCost(), child));
    }
  }
  // >

  // <Step 3-a. Find best non-red parent for each red node.
  for (unsigned int i = 0; i < reds.size(); i++) {
    std::vector<LazyRRGstarNode::Edge> &edges = reds[i]->edges();

    // Initialize cost : Need to be verified
    reds[i]->setCost(SIM_MAX_FLOAT);
    if (reds[i]->pred != NULL) {
      reds[i]->pred->removeChild(reds[i]);
      reds[i]->pred = NULL;
      reds[i]->isCollisionFree(false);
    }

    for (unsigned int j = 0; j < edges.size(); j++) {
      LazyRRGstarNode *neighbor = edges[j].node();

      if (neighbor->color == is_white + 1) continue; // If red, then skip.
      if (reds[i]->getCost() > neighbor->getCost() + edges[j].cost()) {
        reds[i]->setCost(neighbor->getCost() + edges[j].cost());
        reds[i]->pred = neighbor;
      }
    }
    if (reds[i]->pred != NULL) {
      (reds[i]->pred)->addChild(reds[i]);
    }
    // Need to be verified.
		pq.push(weight_node(-reds[i]->getCost(), reds[i]));
  }
  // >

  // <Step 3-b. Propagate the changes; Check a 'red' node can be a better parent node for its neighbors.
  while (!pq.empty()) {
    weight_node top = pq.top();
    pq.pop();

		float cost = -top.first;
    LazyRRGstarNode* node = top.second;

    if (node->getCost() < cost) continue; // Rejected by delayed priority update.

    std::vector<LazyRRGstarNode::Edge> &edges = node->edges();
    for (unsigned int i = 0; i < edges.size(); i++) {
      LazyRRGstarNode *neighbor = edges[i].node();
      if (neighbor->color != is_white + 1) continue; // If not red, then skip.

      if (cost + edges[i].cost() < neighbor->getCost()) {
        neighbor->setCost(cost + edges[i].cost());
        if (neighbor->pred != NULL) {
          neighbor->pred->removeChild(neighbor);
        }
        neighbor->pred = node;
        neighbor->isCollisionFree(false);
        node->addChild(neighbor);
				pq.push(weight_node(-neighbor->getCost(), neighbor));
      }
    }
  }
  // >

  // The end!
  // Restoring the original white color for all the red vertices will be automatically done by
  // increasing is_white variable.
}

// Return best solution path in the current roadmap using traditional A* algorithm.
float LazyRRGstar::getBestSolutionPath(LazyRRGstarNode* goal_node) {
  priority_queue<LazyRRGstarNode*> pq;
  LazyRRGstarNode* s = _start_node;

  s->color = 1; // Gray.
  s->d = 0;
  s->f = distance(s, goal_node);
  pq.push(s);

  while (!pq.empty()) {
    LazyRRGstarNode* top = pq.top();
    pq.pop();

    for (unsigned int i = 0; i < top->_edges.size(); i++) {
      LazyRRGstarNode::Edge &edge = top->_edges[i];

      float dist = edge.cost();
      if (dist + top->d < edge.node()->d) {
        edge.node()->d = dist + top->d;
        edge.node()->f = edge.node()->d + distance(edge.node(), goal_node);
        edge.node()->pred = top;

        if (edge.node()->color == 0) { // White.
          edge.node()->color = 1;
          pq.push(edge.node());
        } else if (edge.node()->color == 2) { // Black.
          edge.node()->color = 1;
          pq.push(edge.node());
        }
      }
    }
    top->color = 2;
  }

  if (goal_node->color != 2) { // Failed to find a solution path
    return 0.0;
  } else { // It must be black!
    LazyRRGstarNode* back_track = goal_node;
    while (back_track->pred != NULL) {
      foundPath.push_back(back_track);
      back_track = back_track->pred;
    }
    foundPath.push_back(back_track);
  }
  reverse(foundPath.begin(), foundPath.end());
  return 1.0;
}

// Return true if 'it' can be part of new best solution.
bool LazyRRGstar::gotPotential(LazyRRGstarNode* it) {
  LazyRRGstarNode* goal_conf = static_cast<LazyRRGstarNode*>(fromGoal[0]);
  if (goal_conf->getCost() >= SIM_MAX_FLOAT || (distance(goal_conf, it) + it->getCost() < goal_conf->getCost())) {
    return true;
  }
  return false;
}

void LazyRRGstar::getPathData(std::vector<float>& data) {
  data.clear();
  if (invalidData)
    return;
  for (int i = 0; i < int(foundPath.size()); i++) {
    LazyRRGstarNode* theNode = static_cast<LazyRRGstarNode*>(foundPath[i]);
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

bool LazyRRGstar::setPartialPath() {
  printf("%lu\n", _nn->size());

  vector<LazyRRGstarNode*> nodes;
  _nn->list(nodes);

  int sum_degree = 0;
  for (unsigned int i = 0; i < nodes.size(); i++) {
    sum_degree += nodes[i]->_edges.size();
  }

  LazyRRGstarNode *it = _goal_node;
  while (it != NULL) {
    foundPath.insert(foundPath.begin(), it->copyYourself());
    it = it->pred;
  }

  printf("Avg. Degree : %f\n", sum_degree / (double)nodes.size());
  printf("DD : %d // DI : %d\n", _dynamic_decrease_count, _dynamic_increase_count);
  printf("Final solution cost : %f\n", _best_cost);
  printf("Collision Detection : %d\n", _collision_detection_count);

  FILE *ofp = NULL;
  char file_name[256] = "LazyRRGstar";
#ifdef CACHING
  strcat(file_name, "_caching");
#endif
#ifdef DYNAMIC
  strcat(file_name, "_dynamic");
#endif
  strcat(file_name, ".log");
  ofp = fopen(file_name, "a");
  if (ofp == NULL) {
    fprintf(stderr, "File Open Error!\n");
    exit(1);
  }

  fprintf(ofp, "%lu\t%f\t%d\t%d\t%d\t%d",
          _nn->size(), _best_cost, _collision_detection_count, _skipped_collision_detection_count,
          _dynamic_decrease_count, _dynamic_increase_count);
#ifdef TIME
  fprintf(ofp, "\t%f\t%f\t%f\t%f\t%f\t",
          sum_degree / (double)nodes.size(), _collision_detection_time / 1000.0f, _dynamic_decrease_time / 1000.0f,
          _dynamic_increase_time / 1000.0f, _near_neighbor_search_time / 1000.0f);
#endif

  fprintf(ofp, "\n");
  fclose(ofp);

  return true;
}

LazyRRGstarNode* LazyRRGstar::slerp(LazyRRGstarNode* from, LazyRRGstarNode* to, float t) {
  return to;
}

LazyRRGstarNode* LazyRRGstar::extend(LazyRRGstarNode* from, LazyRRGstarNode* to,
                             bool shouldBeConnected, CDummyDummy* dummy, float &artificialCost) {
  // Return value is != NULL if extention was performed and connect is false
  // If connect is true, then return value indicates that connection can be performed!
  LazyRRGstarNode* extended = from->copyYourself();
  float theVect[7] = {0.0, };
  int passes = getVector(from, to, theVect, stepSize, artificialCost, false);
  int currentPass;
#ifdef DYNAMIC
  float delta_magnitude = artificialCost / passes;
  float magnitude = 0.0f;
#endif

#ifdef TIME
  int elapsed_time = simGetSystemTimeInMs(-1);
#endif

  C3Vector pos(extended->values);
  C4Vector orient(extended->values + 3);
  C3Vector delta_p(theVect);
  C4Vector delta_q(theVect + 3);

  if (planningType == sim_holonomicpathplanning_xyg) {
    orient = _gammaAxisRotation * C4Vector(C3Vector(0.0f, 0.0f, pos(2))) * _gammaAxisRotationInv;
  }

  for (currentPass = 0; currentPass < passes; currentPass++) {
    pos += delta_p;
#ifdef DYNAMIC
    magnitude += delta_magnitude;
#endif

    if (planningType == sim_holonomicpathplanning_xyz || planningType == sim_holonomicpathplanning_xy)
      orient.setIdentity();
    else if (planningType == sim_holonomicpathplanning_xyzabg)
      orient *= delta_q;
    else if (planningType == sim_holonomicpathplanning_xyg) {
      orient *= _gammaAxisRotation * C4Vector(C3Vector(0.0f, 0.0f, delta_p(2))) * _gammaAxisRotationInv;
      pos(2) = 0.0;
    }

    C7Vector transf(orient, pos);
    C7Vector tmpTr(_startDummyLTM * transf);
    _simSetObjectLocalTransformation(dummy, tmpTr.X.data, tmpTr.Q.data);
    if (doCollide(NULL)) { // Collision Check
#ifdef DYNAMIC
      extended->setAllValues(pos, orient);
      LazyRRGstarNode *witness = extended->copyYourself();

      from->updateWitness(magnitude, witness);
      to->updateWitness(artificialCost - magnitude, witness);
#endif
      if (shouldBeConnected) {
        // delete extended;
#ifdef TIME
    _collision_detection_time += _simGetTimeDiffInMs(elapsed_time);
#endif
        return(NULL);
      }
      break;
    }
  }

  if (currentPass > 0) {
    if (currentPass < passes) { // Not fully connected, roll back the last change
      if (planningType == sim_holonomicpathplanning_xyg) {
        pos -= delta_p;
        pos(2) = 0.0;
        orient *= _gammaAxisRotation * C4Vector(C3Vector(0.0f, 0.0f, -delta_p(2))) * _gammaAxisRotationInv;
      } else if (planningType == sim_holonomicpathplanning_xyz ||
                 planningType == sim_holonomicpathplanning_xyzabg ||
                 planningType == sim_holonomicpathplanning_xy) {
        pos -= delta_p;
        orient *= delta_q.getInverse();
      }
    }
    extended->setAllValues(pos, orient);
    artificialCost *= currentPass / (float)passes;
#ifdef TIME
    _collision_detection_time += _simGetTimeDiffInMs(elapsed_time);
#endif
    return extended;
  }
  // delete extended;
#ifdef TIME
    _collision_detection_time += _simGetTimeDiffInMs(elapsed_time);
#endif
  return(NULL);
}

LazyRRGstarNode* LazyRRGstar::lazyExtend(LazyRRGstarNode* from, LazyRRGstarNode* to,
                                         bool shouldBeConnected, CDummyDummy* dummy, float &artificialCost) {
  // Return value is != NULL if extention was performed and connect is false
  // If connect is true, then return value indicates that connection can be performed!
  LazyRRGstarNode* extended = from->copyYourself();
	float theVect[7] = {0.0, };
	int passes = getVector(from, to, theVect, stepSize, artificialCost, false);

#ifndef DYNAMIC
	_skipped_collision_detection_count += passes;
  artificialCost = distance(from, to);
  return extended;
#endif

#ifdef TIME
  int elapsed_time = simGetSystemTimeInMs(-1);
#endif

  int currentPass;
  float delta_magnitude = artificialCost / passes;
  float magnitude = 0.0f;

  C3Vector pos(extended->values);
  C4Vector orient(extended->values + 3);
  C3Vector delta_p(theVect);
  C4Vector delta_q(theVect + 3);

  if (planningType == sim_holonomicpathplanning_xyg) {
    orient = _gammaAxisRotation * C4Vector(C3Vector(0.0f, 0.0f, pos(2))) * _gammaAxisRotationInv;
  }

  for (currentPass = 0; currentPass < passes; currentPass++) {
    pos += delta_p;
    magnitude += delta_magnitude;

    if (planningType == sim_holonomicpathplanning_xyz || planningType == sim_holonomicpathplanning_xy)
      orient.setIdentity();
    else if (planningType == sim_holonomicpathplanning_xyzabg)
      orient *= delta_q;
    else if (planningType == sim_holonomicpathplanning_xyg) {
      orient *= _gammaAxisRotation * C4Vector(C3Vector(0.0f, 0.0f, delta_p(2))) * _gammaAxisRotationInv;
      pos(2) = 0.0;
    }

    if (magnitude < from->free_radius || artificialCost - magnitude < to->free_radius) {
#ifdef TIME
  _skipped_collision_detection_count += 1;
#endif
      continue;
    }

    C7Vector transf(orient, pos);
    C7Vector tmpTr(_startDummyLTM * transf);
    _simSetObjectLocalTransformation(dummy, tmpTr.X.data, tmpTr.Q.data);
    if (doCollide(NULL)) { // Collision Check
      extended->setAllValues(pos, orient);
      LazyRRGstarNode *witness = extended;
#ifdef DYNAMIC
      from->updateWitness(magnitude, witness);
      to->updateWitness(artificialCost - magnitude, witness);
#endif
      if (shouldBeConnected) {
        // delete extended;
#ifdef TIME
    _collision_detection_time += _simGetTimeDiffInMs(elapsed_time);
#endif
        return(NULL);
      }
      break;
    } else {
#ifdef ONCE
      break;
#endif
    }
  }

#ifdef TIME
    _collision_detection_time += _simGetTimeDiffInMs(elapsed_time);
#endif
  return extended;
}

bool LazyRRGstar::isFree(LazyRRGstarNode* node, CDummyDummy* dummy) {
#ifdef TIME
  int elapsed_time = simGetSystemTimeInMs(-1);
#endif

  C3Vector pos(node->values);
  C4Vector orient(node->values + 3);

  if (planningType == sim_holonomicpathplanning_xyg) {
    orient = _gammaAxisRotation * C4Vector(C3Vector(0.0f, 0.0f, pos(2))) * _gammaAxisRotationInv;
    pos(2) = 0.0;
  } else if (planningType == sim_holonomicpathplanning_xyz || planningType == sim_holonomicpathplanning_xy)
    orient.setIdentity();

  C7Vector transf(orient, pos);
  C7Vector tmpTr(_startDummyLTM * transf);
  _simSetObjectLocalTransformation(dummy, tmpTr.X.data, tmpTr.Q.data);
  if (doCollide(NULL)) { // Collision Check
#ifdef TIME
    _collision_detection_time += _simGetTimeDiffInMs(elapsed_time);
#endif
    return false;
  }
#ifdef TIME
    _collision_detection_time += _simGetTimeDiffInMs(elapsed_time);
#endif
  return true;
}

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

#include "LazyRRGstar.h"
#include "pathPlanningInterface.h"
#include "v_repLib.h"

#define SIM_MIN(a,b) (((a)<(b)) ? (a) : (b))
#define SIM_MAX(a,b) (((a)>(b)) ? (a) : (b))

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
  _goal_node->setCost(SIM_MAX_FLOAT * 0.01f);

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

  _nn.reset(new ompl::NearestNeighborsLinear<LazyRRGstarNode*>()); // Initialize NearestNeighbors structure
  _nn->setDistanceFunction(boost::bind(&LazyRRGstar::distance, this, _1, _2));
  _nn->add(_start_node);
  _nn->add(_goal_node);

  // Set ballRadiusMax and ballRadiusConst to maximum extent
  _ballRadiusMax = _ballRadiusConst = sqrt(_searchRange[0] * _searchRange[0] + _searchRange[1] * _searchRange[1] + _searchRange[2] * _searchRange[2]);
  _best_cost = SIM_MAX_FLOAT;

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

void LazyRRGstar::setAngularCoefficient(float coeff) {
  angularCoeff = coeff;
}

void LazyRRGstar::setStepSize(float size) {
  stepSize = size;
}

float LazyRRGstar::getNearNeighborRadius() {
  // return SIM_MIN(_ballRadiusMax, _ballRadiusConst * pow(log(1.0 + fromStart.size()) / (1.0 + fromStart.size()), 0.33));
  return SIM_MIN(_ballRadiusMax, _ballRadiusConst * pow(log(1.0 + _nn->size()) / (1.0 + _nn->size()), 0.33));
}

float LazyRRGstar::distance(LazyRRGstarNode* a, LazyRRGstarNode* b) {
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

void LazyRRGstar::getSearchTreeData(std::vector<float>& data, bool fromTheStart) {
  std::vector<LazyRRGstarNode*> cont;
  _nn->list(cont);

  if (fromTheStart) {
    if ( (planningType == sim_holonomicpathplanning_xy) || (planningType == sim_holonomicpathplanning_xyg) || (planningType == sim_holonomicpathplanning_xyabg) ) {
      for (int i = 1; i < int(cont.size()); i++) {
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
  } else { // !fromTheStart
    if ( (planningType == sim_holonomicpathplanning_xyz) || (planningType == sim_holonomicpathplanning_xyzg) || (planningType == sim_holonomicpathplanning_xyzabg) ) {
      for (int i = 0; i < int(cont.size()); i++) {
        C3Vector start(cont[i]->values[0], cont[i]->values[1], cont[i]->values[2]);
        C3Vector goal(cont[i]->pred->values[0], cont[i]->pred->values[1], cont[i]->pred->values[2]);
        start = _startDummyCTM * start;
        goal = _startDummyCTM * goal;
        float d[6];
        start.copyTo(d);
        goal.copyTo(d + 3);
        for (int j = 0; j < 6; j++)
          data.push_back(d[j]);
        /*
        for (int j = 0; j < it->_nodes.size(); j++) {
          LazyRRGstarNode* jt = it->_nodes[j].node();
          if (jt == it->pred) continue;

          C3Vector start(it->values[0], it->values[1], it->values[2]);
          C3Vector goal(jt->values[0], jt->values[1], jt->values[2]);
          start = _startDummyCTM * start;
          goal = _startDummyCTM * goal;
          float d[6];
          start.copyTo(d);
          goal.copyTo(d + 3);
          for (int k = 0; k < 6; k++)
            data.push_back(d[k]);
        }
        */
      }
    }
  }
}

int LazyRRGstar::searchPath(int maxTimePerPass) {
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

  int foundAPath = 0;
  int initTime = simGetSystemTimeInMs(-1);
  LazyRRGstarNode* randNode = new LazyRRGstarNode(planningType, _searchMinVal, _searchRange, _gammaAxisRotation, _gammaAxisRotationInv);
  while (_simGetTimeDiffInMs(initTime) < maxTimePerPass) {
    randNode->reSample(planningType, _searchMinVal, _searchRange);

    if (!isFree(randNode, startDummy)) continue;

    LazyRRGstarNode* closest = _nn->nearest(randNode);
    float artificialCost, cArtificialCost;
    if (closest == NULL) continue;

    if (distance(randNode, closest) > _ballRadiusConst * _maxDistance) {
      randNode->interpolate(closest, _ballRadiusConst * _maxDistance, angularCoeff);
    }

    /*
    LazyRRGstarNode* extended = extend(closest, randNode, true, startDummy, cArtificialCost);
    if (extended == NULL) continue;
    */
    LazyRRGstarNode* extended = lazyExtend(closest, randNode, true, startDummy, cArtificialCost);

#ifdef PRESORT
    vector<pair<float, LazyRRGstarNode*> > associated_neighbors;
    associated_neighbors.push_back(pair<float, LazyRRGstarNode*>(closest->getCost() + cArtificialCost, closest));
#endif

    std::vector<LazyRRGstarNode*> neighbors;
    _nn->nearestR(extended, getNearNeighborRadius(), neighbors);
    _nn->add(extended);
    extended->addNode(closest, cArtificialCost);
    closest->addNode(extended, cArtificialCost);

    LazyRRGstarNode* dummy;
    for (int i = 0; i < neighbors.size(); i++) if (neighbors[i] != closest) {
      if (neighbors[i] == _goal_node)
        std::swap(neighbors[i], neighbors[neighbors.size() - 1]);
      dummy = lazyExtend(extended, neighbors[i], true, startDummy, artificialCost);

      extended->addNode(neighbors[i], artificialCost);
      neighbors[i]->addNode(extended, artificialCost);

#ifdef PRESORT
      associated_neighbors.push_back(pair<float, LazyRRGstarNode*>(neighbors[i]->getCost() + artificialCost, neighbors[i]));
#endif
    }
    DynamicDecrease(extended); // Similar to UpdateChildren

    DynamicShortestPathUpdate(startDummy); // LazyUpdate including DynamicIncrease
  }
  delete randNode;

  // We restore the dummy local config and the constraints
  _simSetObjectLocalTransformation(startDummy, dumSavedConf.X.data, dumSavedConf.Q.data);
  for (int constr = 0; constr < 4; constr++)
    _directionConstraints[constr] = dirConstrSave[constr];

  return(foundAPath);
}

/*
void LazyRRGstar::DynamicShortestPathUpdate(LazyRRGstarNode *node, vector<pair<float, LazyRRGstarNode *> > &associated_neighbors) {
#ifdef PRESORT
  // increasing-order in terms of v.cost + d(v, w)
  sort(associated_neighbors.begin(), associated_neighbors.end());
  for (int i = 0; i < associated_neighbors.size(); i++) {
    LazyRRGstarNode *neighbor = associated_neighbors[i].second;
    float new_cost = associated_neighbors[i].first;
    if (node->getCost() < new_cost) {
      node->setCost(new_cost);
      node->pred = neighbor;
    }
    float delta = neighbor->getCost() - (node->getCost() + new_cost - neighbor->getCost());
    if (delta < 0) {
      neighbor->setCost(node->getCost() + new_cost - neighbor->getCost());
      neighbor->pred = node;
    }
  }
#else
#endif

  LazyRRGstarNode *tracer = _goal_node;
  vector<LazyRRGstarNode*> solution_path;
  if (_goal_node->getCost() < _best_cost) {

  }
}
*/

void LazyRRGstar::DynamicShortestPathUpdate(CDummyDummy *startDummy) {
  LazyRRGstarNode *tracer = _goal_node;
  vector<LazyRRGstarNode*> solution_path;
  if (_goal_node->getCost() >= _best_cost) return; // No better 'potential' solution path found this iteration

  // Cutting point priority? from start or goal?
  // From start is more plausible, stable and would be faster!

  while (tracer->pred != NULL) {
    solution_path.push_back(tracer);
    tracer = tracer->pred;
  }

  for (int i = solution_path.size() - 1; i >= 1; i--) {
    LazyRRGstarNode* from = solution_path[i];
    LazyRRGstarNode* to = solution_path[i- 1];

    float artificialCost;
    LazyRRGstarNode* dummy = extend(from, to, true, startDummy, artificialCost);
    if (!dummy) { // collision found
      // Remove invalid edge and handle other things to maintain the shortestpath tree.
      from->removeNode(to);
      to->removeNode(from);
      // needless to say 'to->pred == from'
      DynamicDelete(from, to);
    }
  }
}

// Pre-condition :
void LazyRRGstar::DynamicIncrease(LazyRRGstarNode *from, LazyRRGstarNode *to) {
  typedef pair<float, LazyRRGstarNode*> weight_node;
  priority_queue<weight_node> M;
  priority_queue<weight_node> Q;
  /*
  to->setCost(SIM_MAX_FLOAT);
  to->pred = NULL;
  */

  // Color 0 : white, 1 : pink, 2 : red

  M.push(weight_node(to->getCost(), to));
  while (!M.empty()) {
    weight_node top = M.top();
    M.pop();

    LazyRRGstarNode *node = top.second;
    for (int i = 0; i < node->edges().size(); i++) {
      LazyRRGstarNode *adj = node->edges()[i].node();
      float cost = adj->getCost();
      if (adj->getCost() + cost == top.first && adj->color != 2) {
        // would really rarely happens.
        node->pred = adj;
        node->color = 1;
      } else {
        node->color = 2;

        for (int i = 0; i < node->edges().size(); i++) {
          LazyRRGstarNode *child = node->edges()[i].node();
        }
      }
    }
  }
}

// Pre-condition : to->pred == from
void LazyRRGstar::DynamicDelete(LazyRRGstarNode *from, LazyRRGstarNode *to) {
}

// Pre-condition : node has its own closeset parent node, and to be updated to find
// better parent. It propagtates cost changes downstream from the initial input node.
void LazyRRGstar::DynamicDecrease(LazyRRGstarNode *node) {
  typedef pair<float, LazyRRGstarNode*> weight_node;
  priority_queue<weight_node> pq;
  pq.push(weight_node(node->getCost(), node));

  while (!pq.empty()) {
    weight_node top = pq.top();
    pq.pop();
    float cost = top.first;
    LazyRRGstarNode* node = top.second;

    if (cost > node->getCost()) continue;

    std::vector<LazyRRGstarNode::Edge> &edges = node->edges();
    for (int i = 0; i < edges.size(); i++) {
      LazyRRGstarNode *adj = edges[i].node();
      float c = edges[i].cost();
      if (adj->getCost() > node->getCost() + c) {
        adj->setCost(node->getCost() + c);
        adj->pred = node;
        pq.push(weight_node(adj->getCost(), adj));
      }
    }
  }
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

    for (int i = 0; i < top->_edges.size(); i++) {
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

// 'it' should be connected to the root node and has proper cost
bool LazyRRGstar::gotPotential(LazyRRGstarNode* it) {
  LazyRRGstarNode* goal_conf = static_cast<LazyRRGstarNode*>(fromGoal[0]);
  if (goal_conf->getCost() >= SIM_MAX_FLOAT || (distance(goal_conf, it) + it->getCost() < goal_conf->getCost())) {
    return true;
  }
  return false;
}

bool LazyRRGstar::setPartialPath() {
  vector<LazyRRGstarNode*> path;

  printf("%d\n", _nn->size());

  vector<LazyRRGstarNode*> nodes;
  _nn->list(nodes);

  int sum_degree = 0;
  for (int i = 0; i < nodes.size(); i++) {
    sum_degree += nodes[i]->_edges.size();
  }

  printf("Avg. Degree : %f\n", sum_degree / (double)nodes.size());

  if (getBestSolutionPath(_goal_node) == 0.0)
    return false;
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

  C3Vector pos(extended->values);
  C4Vector orient(extended->values + 3);
  C3Vector delta_p(theVect);
  C4Vector delta_q(theVect + 3);

  if (planningType == sim_holonomicpathplanning_xyg) {
    orient = _gammaAxisRotation * C4Vector(C3Vector(0.0f, 0.0f, pos(2))) * _gammaAxisRotationInv;
  }

  for (currentPass = 0; currentPass < passes; currentPass++) {
    pos += delta_p;

    if (planningType == sim_holonomicpathplanning_xyz)
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
      if (shouldBeConnected) {
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
                 planningType == sim_holonomicpathplanning_xyzabg) {
        pos -= delta_p;
        orient *= delta_q.getInverse();
      }
    }
    extended->setAllValues(pos, orient);
    artificialCost *= currentPass / (float)passes;
    return extended;
  }
  return(NULL);
}

LazyRRGstarNode* LazyRRGstar::lazyExtend(LazyRRGstarNode* from, LazyRRGstarNode* to,
                                         bool shouldBeConnected, CDummyDummy* dummy, float &artificialCost) {
  // Return value is != NULL if extention was performed and connect is false
  // If connect is true, then return value indicates that connection can be performed!
  LazyRRGstarNode* extended = to->copyYourself();
  artificialCost = distance(from, to);

  return extended;
}

bool LazyRRGstar::isFree(LazyRRGstarNode* node, CDummyDummy* dummy) {
  C3Vector pos(node->values);
  C4Vector orient(node->values + 3);

  if (planningType == sim_holonomicpathplanning_xyg) {
    orient = _gammaAxisRotation * C4Vector(C3Vector(0.0f, 0.0f, pos(2))) * _gammaAxisRotationInv;
    pos(2) = 0.0;
  } else if (planningType == sim_holonomicpathplanning_xyz)
    orient.setIdentity();

  C7Vector transf(orient, pos);
  C7Vector tmpTr(_startDummyLTM * transf);
  _simSetObjectLocalTransformation(dummy, tmpTr.X.data, tmpTr.Q.data);
  if (doCollide(NULL)) // Collision Check
    return true;
  return false;
}

  /*
  if (planningType == sim_holonomicpathplanning_xyg) {
    orient = _gammaAxisRotation * C4Vector(C3Vector(0.0f, 0.0f, pos(2))) * _gammaAxisRotationInv;
  }

  for (currentPass = 0; currentPass < passes; currentPass++) {
    pos += delta_p;
    magnitude += delta_magnitude;

    if (planningType == sim_holonomicpathplanning_xyz)
      orient.setIdentity();
    else if (planningType == sim_holonomicpathplanning_xyzabg)
      orient *= delta_q;
    else if (planningType == sim_holonomicpathplanning_xyg) {
      orient *= _gammaAxisRotation * C4Vector(C3Vector(0.0f, 0.0f, delta_p(2))) * _gammaAxisRotationInv;
      pos(2) = 0.0;
    }

    bool skipCollisionCheck = false;
    if (magnitude < from->radius() || artificialCost - magnitude < to->radius()) { //
      skipCollisionCheck = true;
    }

    if (!skipCollisionCheck) {
      C7Vector transf(orient, pos);
      C7Vector tmpTr(_startDummyLTM * transf);
      _simSetObjectLocalTransformation(dummy, tmpTr.X.data, tmpTr.Q.data);
      if (doCollide(NULL)) { // Collision Check
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

        if (shouldBeConnected) {
          return(NULL);
        }
        break;
      }
    }
  }

  if (currentPass > 0) {
    if (currentPass < passes) { // Not fully connected, roll back the last change
      if (planningType == sim_holonomicpathplanning_xyg) {
        pos -= delta_p;
        pos(2) = 0.0;
        orient *= _gammaAxisRotation * C4Vector(C3Vector(0.0f, 0.0f, -delta_p(2))) * _gammaAxisRotationInv;
      } else if (planningType == sim_holonomicpathplanning_xyz ||
                 planningType == sim_holonomicpathplanning_xyzabg) {
        pos -= delta_p;
        orient *= delta_q.getInverse();
      }
    }
    extended->setAllValues(pos, orient);
    artificialCost *= currentPass / (float)passes;
    return extended;
  }
  return(NULL);
  */

int LazyRRGstar::getVector(LazyRRGstarNode* fromPoint, LazyRRGstarNode* toPoint, float vect[7], float e, float& artificialLength, bool dontDivide) {
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

bool LazyRRGstar::addVector(C3Vector& pos, C4Vector& orient, float vect[7]) {
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

int LazyRRGstar::smoothFoundPath(int steps, int maxTimePerPass) {
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
    LazyRRGstarNode* startP;
    LazyRRGstarNode* endP;
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
          startP = static_cast<LazyRRGstarNode*>(foundPath[lowIndex]);
          endP = static_cast<LazyRRGstarNode*>(foundPath[highIndex]);
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
            startP = static_cast<LazyRRGstarNode*>(foundPath[lowIndex]);
            endP = static_cast<LazyRRGstarNode*>(foundPath[highIndex]);
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
              LazyRRGstarNode* it = endP->copyYourself(); // just to have the right size!
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

bool LazyRRGstar::areDirectionConstraintsRespected(float vect[7]) {
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

bool LazyRRGstar::areSomeValuesForbidden(C7Vector configuration) {
  float values[7];
  configuration.getInternalData(values);
  return areSomeValuesForbidden(values);
}

bool LazyRRGstar::areSomeValuesForbidden(float values[7]) {
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

bool LazyRRGstar::doCollide(float* dist) {
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

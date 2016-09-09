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

#include "HolonomicPathNode.h"
#include "pathPlanningInterface.h"
#include "v_repLib.h"

CHolonomicPathNode::CHolonomicPathNode(const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv) {
  _rotAxisRot = rotAxisRot;
  _rotAxisRotInv = rotAxisRotInv;
  parent = NULL;
  _nodeType = -1;
  values = NULL;
}

CHolonomicPathNode::CHolonomicPathNode(int theType, const C7Vector& conf, const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv) {
  _rotAxisRot = rotAxisRot;
  _rotAxisRotInv = rotAxisRotInv;
  parent = NULL;
  _nodeType = theType;
  values = NULL;
  int s = getSize();
  values = new float[s];
  setAllValues(conf.X, conf.Q);
}

CHolonomicPathNode::CHolonomicPathNode(int theType, float searchMin[4], float searchRange[4], const C4Vector& rotAxisRot, const C4Vector& rotAxisRotInv) {
  _rotAxisRot = rotAxisRot;
  _rotAxisRotInv = rotAxisRotInv;
  parent = NULL;
  _nodeType = theType;
  values = NULL;

  if (theType == sim_holonomicpathplanning_xy) {
    values = new float[2];
    values[0] = searchMin[0] + searchRange[0] * SIM_RAND_FLOAT;
    values[1] = searchMin[1] + searchRange[1] * SIM_RAND_FLOAT;
  } else if (theType == sim_holonomicpathplanning_xyz) {
    values = new float[3];
    values[0] = searchMin[0] + searchRange[0] * SIM_RAND_FLOAT;
    values[1] = searchMin[1] + searchRange[1] * SIM_RAND_FLOAT;
    values[2] = searchMin[2] + searchRange[2] * SIM_RAND_FLOAT;
  } else if (theType == sim_holonomicpathplanning_xyg) {
    values = new float[3];
    values[0] = searchMin[0] + searchRange[0] * SIM_RAND_FLOAT;
    values[1] = searchMin[1] + searchRange[1] * SIM_RAND_FLOAT;
    values[2] = CPathPlanningInterface::getNormalizedAngle( + searchRange[3] * SIM_RAND_FLOAT);
  } else if (theType == sim_holonomicpathplanning_xyzabg) {
    values = new float[7];
    values[0] = searchMin[0] + searchRange[0] * SIM_RAND_FLOAT;
    values[1] = searchMin[1] + searchRange[1] * SIM_RAND_FLOAT;
    values[2] = searchMin[2] + searchRange[2] * SIM_RAND_FLOAT;

    C4Vector d;
    d.buildRandomOrientation();
    values[3] = d(0);
    values[4] = d(1);
    values[5] = d(2);
    values[6] = d(3);
  }
}

CHolonomicPathNode::~CHolonomicPathNode() {
  delete[] values;
}

void CHolonomicPathNode::reSample(int theType, float searchMin[4], float searchRange[4]) {
  if (theType == sim_holonomicpathplanning_xy) {
    values[0] = searchMin[0] + searchRange[0] * SIM_RAND_FLOAT;
    values[1] = searchMin[1] + searchRange[1] * SIM_RAND_FLOAT;
  } else if (theType == sim_holonomicpathplanning_xyz) {
    values[0] = searchMin[0] + searchRange[0] * SIM_RAND_FLOAT;
    values[1] = searchMin[1] + searchRange[1] * SIM_RAND_FLOAT;
    values[2] = searchMin[2] + searchRange[2] * SIM_RAND_FLOAT;
  } else if (theType == sim_holonomicpathplanning_xyg) {
    values[0] = searchMin[0] + searchRange[0] * SIM_RAND_FLOAT;
    values[1] = searchMin[1] + searchRange[1] * SIM_RAND_FLOAT;
		values[2] = CPathPlanningInterface::getNormalizedAngle(searchMin[3] + searchRange[3] * SIM_RAND_FLOAT);
  } else if (theType == sim_holonomicpathplanning_xyzabg) {
    values[0] = searchMin[0] + searchRange[0] * SIM_RAND_FLOAT;
    values[1] = searchMin[1] + searchRange[1] * SIM_RAND_FLOAT;
    values[2] = searchMin[2] + searchRange[2] * SIM_RAND_FLOAT;

    C4Vector d;
    d.buildRandomOrientation();
    values[3] = d(0);
    values[4] = d(1);
    values[5] = d(2);
    values[6] = d(3);
  }
}

void CHolonomicPathNode::interpolate(CHolonomicPathNode* from, float t, float angularCoeff) {
  float vect[7];

  if (_nodeType == sim_holonomicpathplanning_xy) {
    vect[0] = values[0] - from->values[0];
    vect[1] = values[1] - from->values[1];

    float l = sqrtf(vect[0] * vect[0] + vect[1] * vect[1]);
    float bound = t / l;

    values[0] = vect[0] * bound + from->values[0];
    values[1] = vect[1] * bound + from->values[1];
  } else if (_nodeType == sim_holonomicpathplanning_xyz) {
    vect[0] = values[0] - from->values[0];
    vect[1] = values[1] - from->values[1];
    vect[2] = values[2] - from->values[2];

    float l = sqrtf(vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2]);
    float bound = t / l;

    values[0] = vect[0] * bound + from->values[0];
    values[1] = vect[1] * bound + from->values[1];
    values[2] = vect[2] * bound + from->values[2];
		/*
    for (int i = 0; i < 3; i++) {
      values[i] = from->values[i] * (1.0 - bound) + values[i] * bound;
    }
		*/
  } else if (_nodeType == sim_holonomicpathplanning_xyg) {
    vect[0] = values[0] - from->values[0];
    vect[1] = values[1] - from->values[1];
    vect[2] = CPathPlanningInterface::getNormalizedAngle(values[2] - from->values[2]);

    float l = vect[0] * vect[0] + vect[1] * vect[1];
    l = sqrtf(l + vect[2] * angularCoeff * vect[2] * angularCoeff);
    float bound = t / l;

    values[0] = vect[0] * bound + from->values[0];
    values[1] = vect[1] * bound + from->values[1];
    values[2] = CPathPlanningInterface::getNormalizedAngle(vect[2] * bound + from->values[2]);
  } else if (_nodeType == sim_holonomicpathplanning_xyzabg) {
    vect[0] = values[0] - from->values[0];
    vect[1] = values[1] - from->values[1];
    vect[2] = values[2] - from->values[2];
    C4Vector toP, fromP;
    C3Vector dum;
    getAllValues(dum, toP);
    from->getAllValues(dum, fromP);
    C4Vector diff(fromP.getInverse()*toP);
    vect[3] = diff(0);
    vect[4] = diff(1);
    vect[5] = diff(2);
    vect[6] = diff(3);

    float ap = angularCoeff * fromP.getAngleBetweenQuaternions(toP);
    float l = vect[0] * vect[0] + vect[1] * vect[1] + vect[2] * vect[2];
    l = sqrtf(l + ap * ap);
    float bound = t / l;

    values[0] = vect[0] * bound + from->values[0];
    values[1] = vect[1] * bound + from->values[1];
    values[2] = vect[2] * bound + from->values[2];
    C4Vector q;
    q.setIdentity();
    fromP.buildInterpolation(q, diff, bound);
    values[3] = fromP(0);
    values[4] = fromP(1);
    values[5] = fromP(2);
    values[6] = fromP(3);
  }
}

void CHolonomicPathNode::setAllValues(const C3Vector& pos, const C4Vector& orient) {
  if (_nodeType == sim_holonomicpathplanning_xy) {
    values[0] = pos(0);
    values[1] = pos(1);
  }
  if (_nodeType == sim_holonomicpathplanning_xg) {
    values[0] = pos(0);
    C4Vector o(_rotAxisRotInv * orient * _rotAxisRot);
    values[1] = o.getEulerAngles()(2);
  }
  if (_nodeType == sim_holonomicpathplanning_xyz) {
    values[0] = pos(0);
    values[1] = pos(1);
    values[2] = pos(2);
  }
  if (_nodeType == sim_holonomicpathplanning_xyg) {
    values[0] = pos(0);
    values[1] = pos(1);
    C4Vector o(_rotAxisRotInv * orient * _rotAxisRot);
    values[2] = o.getEulerAngles()(2);
  }
  if (_nodeType == sim_holonomicpathplanning_abg) {
    values[0] = orient(0);
    values[1] = orient(1);
    values[2] = orient(2);
    values[3] = orient(3);
  }
  if (_nodeType == sim_holonomicpathplanning_xyzg) {
    values[0] = pos(0);
    values[1] = pos(1);
    values[2] = pos(2);
    C4Vector o(_rotAxisRotInv * orient * _rotAxisRot);
    values[3] = o.getEulerAngles()(2);
  }
  if (_nodeType == sim_holonomicpathplanning_xabg) {
    values[0] = pos(0);
    values[1] = orient(0);
    values[2] = orient(1);
    values[3] = orient(2);
    values[4] = orient(3);
  }
  if (_nodeType == sim_holonomicpathplanning_xyabg) {
    values[0] = pos(0);
    values[1] = pos(1);
    values[2] = orient(0);
    values[3] = orient(1);
    values[4] = orient(2);
    values[5] = orient(3);
  }
  if (_nodeType == sim_holonomicpathplanning_xyzabg) {
    values[0] = pos(0);
    values[1] = pos(1);
    values[2] = pos(2);
    values[3] = orient(0);
    values[4] = orient(1);
    values[5] = orient(2);
    values[6] = orient(3);
  }
}

void CHolonomicPathNode::getAllValues(C3Vector& pos, C4Vector& orient) {
  pos.clear();
  orient.setIdentity();
  if (_nodeType == sim_holonomicpathplanning_xy) {
    pos(0) = values[0];
    pos(1) = values[1];
  }
  if (_nodeType == sim_holonomicpathplanning_xg) {
    pos(0) = values[0];
    orient = _rotAxisRot * (C4Vector(C3Vector(0.0f, 0.0f, values[1])) * _rotAxisRotInv);
  }
  if (_nodeType == sim_holonomicpathplanning_xyz) {
    pos(0) = values[0];
    pos(1) = values[1];
    pos(2) = values[2];
  }
  if (_nodeType == sim_holonomicpathplanning_xyg) {
    pos(0) = values[0];
    pos(1) = values[1];
    orient = _rotAxisRot * (C4Vector(C3Vector(0.0f, 0.0f, values[2])) * _rotAxisRotInv);
  }
  if (_nodeType == sim_holonomicpathplanning_abg) {
    orient(0) = values[0];
    orient(1) = values[1];
    orient(2) = values[2];
    orient(3) = values[3];
  }
  if (_nodeType == sim_holonomicpathplanning_xyzg) {
    pos(0) = values[0];
    pos(1) = values[1];
    pos(2) = values[2];
    orient = _rotAxisRot * (C4Vector(C3Vector(0.0f, 0.0f, values[3])) * _rotAxisRotInv);
  }
  if (_nodeType == sim_holonomicpathplanning_xabg) {
    pos(0) = values[0];
    orient(0) = values[1];
    orient(1) = values[2];
    orient(2) = values[3];
    orient(3) = values[4];
  }
  if (_nodeType == sim_holonomicpathplanning_xyabg) {
    pos(0) = values[0];
    pos(1) = values[1];
    orient(0) = values[2];
    orient(1) = values[3];
    orient(2) = values[4];
    orient(3) = values[5];
  }
  if (_nodeType == sim_holonomicpathplanning_xyzabg) {
    pos(0) = values[0];
    pos(1) = values[1];
    pos(2) = values[2];
    orient(0) = values[3];
    orient(1) = values[4];
    orient(2) = values[5];
    orient(3) = values[6];
  }
}

CHolonomicPathNode* CHolonomicPathNode::copyYourself() {
  CHolonomicPathNode* newNode = new CHolonomicPathNode(_rotAxisRot, _rotAxisRotInv);
  newNode->_nodeType = _nodeType;
  int s = getSize();
  newNode->values = new float[s];
  for (int i = 0; i < s; i++)
    newNode->values[i] = values[i];
  return(newNode);
}

void CHolonomicPathNode::setAllValues(float* v) {
  int s = getSize();
  for (int i = 0; i < s; i++)
    values[i] = v[i];
}

int CHolonomicPathNode::getSize() {
  if (_nodeType == sim_holonomicpathplanning_xy)
    return(2);
  if (_nodeType == sim_holonomicpathplanning_xg)
    return(2);
  if (_nodeType == sim_holonomicpathplanning_xyz)
    return(3);
  if (_nodeType == sim_holonomicpathplanning_xyg)
    return(3);
  if (_nodeType == sim_holonomicpathplanning_abg)
    return(4);
  if (_nodeType == sim_holonomicpathplanning_xyzg)
    return(4);
  if (_nodeType == sim_holonomicpathplanning_xabg)
    return(5);
  if (_nodeType == sim_holonomicpathplanning_xyabg)
    return(6);
  if (_nodeType == sim_holonomicpathplanning_xyzabg)
    return(7);
  return(0);
}

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

#include "mpPhase2Node.h"
#include "v_repLib.h"

Node::Node(int jointCount,const float* _jointPositions,const C7Vector& _tipTransf)
{
	jointPositions=new float[jointCount];
	tipTransf=_tipTransf;
	for (int i=0;i<jointCount;i++)
		jointPositions[i]=_jointPositions[i];
    parent=NULL;
    _cost = SIM_MAX_FLOAT;
    color = 0; // White
    pred = witness = NULL;
    is_collision_free = false;
    free_radius = 0.0f;
}

Node::~Node()
{
	delete[] jointPositions;
}

Node* Node::copyYourself(int jointCount)
{
    Node* newNode=new Node(jointCount,jointPositions,tipTransf);
	return(newNode);
}

void Node::removeNode(Node *node) {
  for (int i = 0; i < int(_edges.size()); i++) {
    if (_edges[i].node() == node) { // If the order of nodes doesn't matter, it's faster than vector::erase()
      std::swap(_edges[i], _edges.back());
      _edges.pop_back();
      break;
    }
  }
}

void Node::updateWitness(float dist, Node *node) {
  if (witness == NULL || dist < free_radius) {
    free_radius = dist;
    witness = node;
  }
}

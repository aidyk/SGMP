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

#include "7Vector.h"
#include <vector>

class Node
{
public:
  class Edge {
  public:
    Edge(Node* n, float c) {
      _node = n; _cost = c;
    }
    Node* node() { return _node; }
    float cost() { return _cost; }
    void cost(float c) { _cost = c; }
  private:
    Node* _node;
    float _cost;
  };

public:
    Node(int jointCount,const float* _jointPositions,const C7Vector& _tipTransf);
    virtual ~Node();

    Node* copyYourself(int jointCount);

    float* jointPositions;
    C7Vector tipTransf; // relative to base object!
    Node* parent;
    Node* parentNode;

    // <Set/Getters
    void setCost(float cost) { _cost = cost; }
    float getCost() { return _cost; }

    void addChild(Node* node) { _children.push_back(node); }
    void addNode(Node* node, float cost) { _edges.push_back(Edge(node, cost)); }
    void removeNode(Node* node);
    void removeChild(Node* node) { // Can be optimized by storing index on child side.
      for (unsigned int i = 0; i < _children.size(); i++) if (_children[i] == node){
        std::swap(_children[i], _children.back());
        _children.pop_back();
        break;
      }
    }
    void updateChildrenCosts(float delta_cost) {
      for (int i = 0; i < int(_children.size()); i++) {
        _children[i]->setCost(_children[i]->getCost() + delta_cost);
        _children[i]->updateChildrenCosts(delta_cost);
      }
    }

    void updateWitness(float dist, Node* witness);
    void setAllValues(std::vector<float> joint, int jointCount) {
      for (int i = 0; i < jointCount; i++) {
        jointPositions[i] = joint[i];
      }
    }

    std::vector<Edge>& edges() { return _edges; }
    std::vector<Node*>& children() { return _children; }

    bool isCollisionFree() { return is_collision_free; }
    void isCollisionFree(bool value) { is_collision_free = value; }
    // >
    // for shortestpath computation
    Node* pred;

    bool is_collision_free; // It means a connection (parent -> this) is collision free.

    // <Dynamic CSA
    Node* witness;
    float free_radius;
    // >
    int color;

    std::vector<Edge> _edges;
    std::vector<Node*> _children;

  private:
    float _cost;
};

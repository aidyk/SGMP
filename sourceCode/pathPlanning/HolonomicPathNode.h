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

#include "3Vector.h"
#include "4Vector.h"
#include "7Vector.h"

class CHolonomicPathNode  
{
public:
	CHolonomicPathNode(const C4Vector& rotAxisRot,const C4Vector& rotAxisRotInv);
	CHolonomicPathNode(int theType,const C7Vector& conf,const C4Vector& rotAxisRot,const C4Vector& rotAxisRotInv);
	CHolonomicPathNode(int theType,float searchMin[4],float searchRange[4],const C4Vector& rotAxisRot,const C4Vector& rotAxisRotInv);
	virtual ~CHolonomicPathNode();

	virtual CHolonomicPathNode* copyYourself();
	virtual int getSize();
	virtual void setAllValues(float* v);
	virtual void setAllValues(const C3Vector& pos,const C4Vector& orient);
	virtual void getAllValues(C3Vector& pos,C4Vector& orient);
	virtual void reSample(int theType, float searchMin[4], float serachRange[4]);

	CHolonomicPathNode* parent;
	float* values;
protected:
	int _nodeType;
	C4Vector _rotAxisRot;
	C4Vector _rotAxisRotInv;
};

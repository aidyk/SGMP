# This file is part of the PATH PLANNING PLUGIN for V-REP
# 
# Copyright 2006-2014 Coppelia Robotics GmbH. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# The PATH PLANNING PLUGIN is licensed under the terms of EITHER:
#   1. PATH PLANNING PLUGIN commercial license (contact us for details)
#   2. PATH PLANNING PLUGIN educational license (see below)
# 
# PATH PLANNING PLUGIN educational license:
# -------------------------------------------------------------------
# The PATH PLANNING PLUGIN educational license applies only to EDUCATIONAL
# ENTITIES composed by following people and institutions:
# 
# 1. Hobbyists, students, teachers and professors
# 2. Schools and universities
# 
# EDUCATIONAL ENTITIES do NOT include companies, research institutions,
# non-profit organisations, foundations, etc.
# 
# An EDUCATIONAL ENTITY may use, modify, compile and distribute the
# modified/unmodified PATH PLANNING PLUGIN under following conditions:
#  
# 1. Distribution should be free of charge.
# 2. Distribution should be to EDUCATIONAL ENTITIES only.
# 3. Usage should be non-commercial.
# 4. Altered source versions must be plainly marked as such and distributed
#    along with any compiled code.
# 5. When using the PATH PLANNING PLUGIN in conjunction with V-REP, the "EDU"
#    watermark in the V-REP scene view should not be removed.
# 6. The origin of the PATH PLANNING PLUGIN must not be misrepresented. you must
#    not claim that you wrote the original software.
# 
# THE PATH PLANNING PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR
# IMPLIED WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
# -------------------------------------------------------------------
#
# This file was automatically created for V-REP release V3.1.3 on Sept. 30th 2014

TARGET = v_repExtPathPlanning
TEMPLATE = lib

DEFINES -= UNICODE

CONFIG += shared

*-msvc* {
	QMAKE_CXXFLAGS += -O2
	QMAKE_CXXFLAGS += -W3
}
*-g++* {
	QMAKE_CXXFLAGS += -O3
	QMAKE_CXXFLAGS += -Wall
	QMAKE_CXXFLAGS += -Wno-unused-parameter
	QMAKE_CXXFLAGS += -Wno-strict-aliasing
	QMAKE_CXXFLAGS += -Wno-empty-body
	QMAKE_CXXFLAGS += -Wno-write-strings
	QMAKE_CXXFLAGS += -std=c++11

	# Best would be to have a switch based on compiler version, but apparently that doesn't exist. So we use the Qt version..
	greaterThan(QT_MAJOR_VERSION,4): QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
	greaterThan(QT_MAJOR_VERSION,4): QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
	greaterThan(QT_MAJOR_VERSION,4): QMAKE_CXXFLAGS += -Wno-narrowing

	QMAKE_CFLAGS += -O3
	QMAKE_CFLAGS += -Wall
	QMAKE_CFLAGS += -Wno-strict-aliasing
	QMAKE_CFLAGS += -Wno-unused-parameter
	QMAKE_CFLAGS += -Wno-unused-but-set-variable
	QMAKE_CFLAGS += -Wno-unused-local-typedefs
}

win32 {
	DEFINES += WIN_VREP
}

macx {
	DEFINES += MAC_VREP
}

unix:!macx {
	DEFINES += LIN_VREP
}

DEFINES += COMPILING_EXTERNAL_PATHPLANNING_DLL
#DEFINES += QT_FRAMEWORK # when using this, make sure you use the exact same compiler and Qt version as the main V-REP library!


INCLUDEPATH += "sourceCode"
INCLUDEPATH += "sourceCode/pathPlanning"
INCLUDEPATH += "sourceCode/motionPlanning"
INCLUDEPATH += "sourceCode/stateSpace"
INCLUDEPATH += "../v_rep/sourceCode/interfaces"
INCLUDEPATH += "../programming/v_repMath"
INCLUDEPATH += "../programming/include"
INCLUDEPATH += "../programming/common"
INCLUDEPATH += "/opt/local/include"
INCLUDEPATH += "/Users/ai/Desktop/PB-RRT/ompl-1.0.0-Source/src"

HEADERS += ../v_rep/sourceCode/interfaces/pathPlanningInterface.h \
	../v_rep/sourceCode/interfaces/dummyClasses.h \
	../programming/include/v_repLib.h \
	../programming/v_repMath/3Vector.h \
	../programming/v_repMath/4Vector.h \
	../programming/v_repMath/7Vector.h \
	../programming/v_repMath/3X3Matrix.h \
	../programming/v_repMath/4X4Matrix.h \
	../programming/v_repMath/MyMath.h \
	../ompl-1.0.0-Source/src/ompl/datastructures/NearestNeighborsGNAT.h \
    sourceCode/stateSpace/Real2DStateSpace.h \
    sourceCode/stateSpace/Real3DStateSpace.h \
    sourceCode/stateSpace/SE2StateSpace.h \
    sourceCode/stateSpace/SE3StateSpace.h \
    sourceCode/stateSpace/stateSpace.h \
    sourceCode/pathPlanning/RRGstar.h \
    sourceCode/pathPlanning/RRGstarNode.h \
    sourceCode/pathPlanning/LazyRRGstar.h \
    sourceCode/pathPlanning/LazyRRGstarNode.h
	
HEADERS += sourceCode/pathPlanning/HolonomicPathNode.h \
	sourceCode/pathPlanning/HolonomicPathPlanning.h \
	sourceCode/pathPlanning/NonHolonomicPathNode.h \
	sourceCode/pathPlanning/NonHolonomicPathPlanning.h \
	sourceCode/pathPlanning/HolonomicRRT.h \
	sourceCode/pathPlanning/HolonomicRRTNode.h \
	sourceCode/pathPlanning/HolonomicBiRRT.h \
	sourceCode/pathPlanning/HolonomicBiRRTNode.h \
	sourceCode/pathPlanning/HolonomicRRTstar.h \
	sourceCode/pathPlanning/HolonomicRRTstarNode.h \
	sourceCode/pathPlanning/PathPlanning.h \
	sourceCode/motionPlanning/mpObject.h \
	sourceCode/motionPlanning/mpPhase1Node.h \
	sourceCode/motionPlanning/mpPhase2Node.h \
	sourceCode/v_repExtPathPlanning.h \

SOURCES += ../v_rep/sourceCode/interfaces/pathPlanningInterface.cpp \
	../programming/common/v_repLib.cpp \
	../programming/v_repMath/3Vector.cpp \
	../programming/v_repMath/4Vector.cpp \
	../programming/v_repMath/7Vector.cpp \
	../programming/v_repMath/3X3Matrix.cpp \
	../programming/v_repMath/4X4Matrix.cpp \
	../programming/v_repMath/MyMath.cpp \
    sourceCode/stateSpace/Real2DStateSpace.cpp \
    sourceCode/stateSpace/Real3DStateSpace.cpp \
    sourceCode/stateSpace/SE2StateSpace.cpp \
    sourceCode/stateSpace/SE3StateSpace.cpp \
    sourceCode/stateSpace/stateSpace.cpp \
    sourceCode/pathPlanning/RRGstar.cpp \
    sourceCode/pathPlanning/RRGstarNode.cpp \
    sourceCode/pathPlanning/LazyRRGstar.cpp \
    sourceCode/pathPlanning/LazyRRGstarNode.cpp

SOURCES += sourceCode/pathPlanning/HolonomicPathNode.cpp \
	sourceCode/pathPlanning/HolonomicPathPlanning.cpp \
	sourceCode/pathPlanning/NonHolonomicPathNode.cpp \
	sourceCode/pathPlanning/NonHolonomicPathPlanning.cpp \
	sourceCode/pathPlanning/HolonomicRRT.cpp \
	sourceCode/pathPlanning/HolonomicRRTNode.cpp \
	sourceCode/pathPlanning/HolonomicBiRRT.cpp \
	sourceCode/pathPlanning/HolonomicBiRRTNode.cpp \
	sourceCode/pathPlanning/HolonomicRRTstar.cpp \
	sourceCode/pathPlanning/HolonomicRRTstarNode.cpp \
	sourceCode/pathPlanning/PathPlanning.cpp \
	sourceCode/motionPlanning/mpObject.cpp \
	sourceCode/motionPlanning/mpPhase1Node.cpp \
	sourceCode/motionPlanning/mpPhase2Node.cpp \
	sourceCode/v_repExtPathPlanning.cpp \

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /Users/ai/Desktop/V-REP_PRO_EDU_V3_1_3_rev2b_Mac/vrep.app/Contents/MacOS
    }
    INSTALLS += target
}

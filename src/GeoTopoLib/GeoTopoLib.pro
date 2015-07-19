include($$[STARLAB])
include($$[SURFACEMESH])
include($$[NANOFLANN])
StarlabTemplate(none)

TEMPLATE = lib
CONFIG += staticlib
QT += opengl

# Build flag
CONFIG(debug, debug|release) {CFG = debug} else {CFG = release}

# Library name and destination
TARGET = GeoTopoLib
DESTDIR = $$PWD/$$CFG/lib

SOURCES += \ 
    BatchProcess.cpp \
    DeformEnergy.cpp \
    DeformToFit.cpp \
    EnergyGuidedDeformation.cpp \
    EvaluateCorrespondence.cpp \
    PropagateProximity.cpp \
    PropagateSymmetry.cpp \
    StructureAnalysis.cpp

HEADERS += \ 
    AStarSearch.h \
    BatchProcess.h \
    DeformEnergy.h \
    DeformToFit.h \
    EnergyGuidedDeformation.h \
    EvaluateCorrespondence.h \
    PropagateProximity.h \
    PropagateSymmetry.h \
    ShapeGraph.h \
    StructureAnalysis.h

## External libraries:
# NURBS library
LIBS += -L$$PWD/../NURBS/$$CFG/lib -lNURBS
INCLUDEPATH += ../NURBS

# Structure Graph Library
LIBS += -L$$PWD/../StructureGraphLib/$$CFG/lib -lStructureGraphLib
INCLUDEPATH += ../StructureGraphLib

#pragma once

#include <QMap>
#include <QQueue>
#include <Eigen/Core>

namespace Structure{ struct Graph; struct Node; struct Link; }

class Task;

struct LinkConstraint{
    Structure::Link *link;
    Task *task, *otherTask;
    LinkConstraint(Structure::Link * l=NULL, Task* t=NULL, Task* otherT=NULL)
    { link = l; task = t; otherTask = otherT; }
};
typedef QMap< Task*, QVector<LinkConstraint> > TasksConstraints;

class Scheduler;

struct Relink
{
    Relink( Scheduler * scheduler );

    Scheduler * s;
    Structure::Graph *activeGraph, *targetGraph;
	
    TasksConstraints constraints;

	// Tracking
	typedef QPair<QString,int> PropagationEdge;
	typedef QVector< PropagationEdge > PropagationEdges;
	QMap< QString, PropagationEdges > propagationGraph;
	int propagationIndex;

	void execute();
	void fixTask( Task* task );

	// Helpers
	void moveByConstraints( Structure::Node * n, QVector<LinkConstraint> consts );
    Eigen::Vector3d getToDelta( Structure::Link * link, QString otherID );
	bool doesPropagate( Task* task );
	bool isInActiveGroup( Task* task );
};

#pragma once

#include <Eigen/Core>

#include "StructureCurve.h"
#include "StructureSheet.h"

// DEBUG:
#include "RenderObjectExt.h"

class QGLWidget;

namespace Structure{
	typedef QVector< QVector<QString> > NodeGroups;

	struct Graph
	{
		// Properties
		QVector<Node*> nodes;
		QVector<Link*> edges;
		NodeGroups groups;

		Eigen::AlignedBox3d bbox(bool isSkipUnready = false);
		Eigen::AlignedBox3d cached_bbox();

		PropertyMap property;
		int ueid;
		QString name();

		int valence(Node * n);
	
		// Constructors
		Graph();
		Graph(QString fileName);
		Graph(const Graph & other);
		~Graph();
		void init();
	
		// Modifiers
		Node * addNode( Node * n );
		Link * addEdge( Node * n1, Node * n2 );
		Link * addEdge( Node *n1, Node *n2, Array1D_Vector4 coord1, Array1D_Vector4 coord2, QString linkName = "" );
		Link * addEdge( QString n1_id, QString n2_id );

		void removeNode( QString nodeID );
		void removeEdge( int uid );
		void removeEdge( Node * n1, Node * n2 );
		void removeEdge( QString n1_id, QString n2_id );
		void removeEdges( QString nodeID );
		void removeIsolatedNodes();

		void addGroup(QVector<QString> nodes);
		void removeGroup(int groupIDX);
		void removeGroup(QVector<QString> groupElements);
		QVector< QVector<QString> > groupsOf( QString nodeID );
        QVector< QVector<QString> > nodesAsGroups();

		QString linkName( QString n1_id, QString n2_id );
		QString linkName( Node * n1, Node * n2 );

		// Node-wide Operations
		void setPropertyAll( QString prop_name, QVariant value );
		void setVisPropertyAll( QString prop_name, QVariant value );
		void setPropertyFor( QVector<QString> nodeIDs, QString prop_name, QVariant value );
		void setColorAll( QColor newNodesColor );
		void setColorFor( QString nodeID, QColor newColor );
	
		QVector<Structure::Node*> nodesWithProperty( QString propertyName );
		QVector<Structure::Node*> nodesWithProperty( QString propertyName,  QVariant value );

		QVector<Structure::Node*> path(Structure::Node * from, Structure::Node * to);

		bool shareEdge( Node * n1, Node * n2 );
		bool shareEdge( QString nid1, QString nid2 );
		void renameNode( QString oldNodeID, QString newNodeID );

		// Accessors
        Node* getNode(QString nodeID);
		Link* getEdge(int edgeUID);
		Link* getEdge(QString id1, QString id2);
		Curve* getCurve(Link * l);
		QVector<Link*> getEdges( QString nodeID );
		QVector<Link*> getEdges( QVector<int> edgeUIDs );
		QVector<int> getEdgeIDs( QVector<Link*> forEdges );
		QMap< Link*, Array1D_Vector4 > linksCoords( QString nodeID );
		QVector<Link*> nodeEdges( QString nodeID );
		QVector<Node*> adjNodes( Node * node );
		void replaceCoords( QString nodeA, QString nodeB, Array1D_Vector4 coordA, Array1D_Vector4 coordB );
		int indexOfNode( Node * node );
        SurfaceMeshModel* getMesh( QString nodeID );
		QList<Link*> furthermostEdges( QString nodeID );
        Vector3 position(QString nodeID, Vector4 &coord );
		Vector3 nodeIntersection( Node * n1, Node * n2 );
        Eigen::AlignedBox3d robustBBox();
        Eigen::AlignedBox3d robustBBox(QString nodeID, double eps = 1e-6);

		// Input / Output
		void saveToFile(QString fileName, bool isOutParts = true) const;
                bool loadFromFile(QString fileName);

		void exportAsOBJ( QString filename );

		// TopoBlend related
		static Structure::Graph * actualGraph(Structure::Graph * fromGraph);

		// Visualization
        void draw(QGLWidget *drawArea = 0 );
		void drawAABB();
        void draw2D(int width, int height);
        void drawNodeMesh(QString nid, QColor meshColor);
		void drawNodeMeshNames( int & offSet );
		QImage fontImage;

		// Analysis
		QSet<QString> nodesCanVisit( Node * node );
		int numCanVisit( Structure::Node * node );
		QVector< QSet<QString> > connectedComponents(); // inefficient

		bool isConnected();
		bool isCutNode( QString nodeID );
		bool isInCutGroup( QString nodeID );
		bool isBridgeEdge( Structure::Link * link );
		QVector< QVector<Node*> > split( QString nodeID );

		QVector<Node*> articulationPoints();
		void articulationDFS( int & cnt, int u, int v, QVector<int> & low, QVector<int> & pre );

		QVector<Node*> leaves();

		Node * rootBySize();
		Node * rootByValence();

		// Modifier
        void moveBottomCenterToOrigin(bool isKeepMeshes = false);
		void normalize();
		void translate( Vector3 delta, bool isKeepMeshes = false);
		void rotate(double angle, Vector3 axis);
		void scale(double scaleFactor);
        void transform(QMatrix4x4 mat,bool isKeepMeshes = false);
		void moveCenterTo( Vector3 newCenter, bool isKeepMeshes );

        // Topology modifiers
        bool hasDoubleEdges(QString nodeID);
        void cutNode(QString nodeID, int cutCount);
		void joinNodes(QStringList nodeIDs);

        Array2D_Vector3 getAllControlPoints();
        void setAllControlPoints(Array2D_Vector3 all_points);

		// Point Landmarks
		QVector<POINT_ID> selectedControlPointsByColor(QColor color);

		// DEBUG:
		std::vector<Vector3> debugPoints,debugPoints2,debugPoints3;
		starlab::VectorSoup vs,vs2,vs3;
		starlab::PolygonSoup ps,ps2,ps3;
		starlab::SphereSoup spheres, spheres2;
		QMap< QString, void* > misc;
        QVector<RenderObject::Base*> debug;

		// Clean up
		void clearDebug();
		void clearAll();
        void clearSelections();
    };
}

Q_DECLARE_METATYPE( QSharedPointer<SurfaceMeshModel> )
Q_DECLARE_METATYPE( Structure::Graph * )
Q_DECLARE_METATYPE( Structure::NodeGroups )

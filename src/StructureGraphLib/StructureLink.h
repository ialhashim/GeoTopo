#pragma once

#include "StructureGlobal.h"
#include "NurbsDraw.h"

typedef Array1D_Vector4 LinkCoords;
typedef QPair< QString,Eigen::Vector4d > NodeCoord;
typedef QPair< QString,LinkCoords > NodeCoords;

using namespace opengp::SurfaceMesh;

namespace Structure{

typedef Eigen::Vector4d Vector4;

struct Node;

static QString CURVE = "CURVE";
static QString SHEET = "SHEET";
static QString POINT_EDGE = "POINT";
static QString LINE_EDGE = "LINE";

struct Link
{
	// Properties
	Node *n1, *n2;	
	std::vector<LinkCoords> coord;
	QString id;
	QString type;
	PropertyMap property;

	bool hasProperty(QString propertyName) { return property.contains(propertyName); }

	template<typename T>
	void setProperty( QString propertyName, T propertyValue ){
		property[propertyName].setValue( propertyValue );
	}

    void setCoord( QString nodeID, Array1D_Vector4 newCoord );
    void setCoordOther( QString nodeID, Array1D_Vector4 newCoord );

    Array1D_Vector4 getCoord(QString nodeID);
    Array1D_Vector4 getCoordOther(QString nodeID);
    Vector4 getMiddleCoord(QString nodeID);
	void invertCoords( QString nodeID );
	Node * getNode(QString nodeID);
	Node * otherNode(QString nodeID);

	Node * getNodeHasProperty(QString propertyName, QVariant propertyValue);

	// Modify
    void replace(QString oldNodeID, Node *newNode, Array1D_Vector4 newCoord);
    void replaceForced(QString oldNodeID, Node *newNode, Array1D_Vector4 newCoord);

	// Constructors
    Link(Node * node1, Node * node2, LinkCoords coord_n1, LinkCoords coord_n2, QString link_type, QString ID);
	Link(){	n1 = n2 = NULL; coord.resize(2); }
	~Link();

	// Accessors
	bool hasNode(QString nodeID);
	bool hasNodeProperty(QString propertyName, QVariant propertyValue);
	Vector3 position(QString nodeID);
	Vector3 positionOther(QString nodeID);

	// Visualization
    void draw(bool isShowLine = false);

    bool operator== ( const Link & other ) const{
        return id == other.id;
    }

	/// Helpers:
	static QVector<Link*> haveProperty( QVector<Link*> links, QString propertyName );
	static QVector<Link*> haveProperty( QVector<Link*> links, QString propertyName, QVariant propertyValue );

    Vector3 delta();

	// State
	void pushState();
	void popState();
	void clearState();
	bool isInState(QString nodeID);

private:
	QMap< QString, QVariant> state;
};

}

Q_DECLARE_METATYPE( Structure::Link* )
Q_DECLARE_METATYPE( NodeCoord )
Q_DECLARE_METATYPE( NodeCoords )
Q_DECLARE_METATYPE( QVector< NodeCoord > )

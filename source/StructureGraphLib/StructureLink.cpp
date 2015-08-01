#include "StructureLink.h"
#include "StructureNode.h"

using namespace Structure;

Q_DECLARE_METATYPE(Structure::Node*)
Q_DECLARE_METATYPE(std::vector<Array1D_Vector4>)

Link::Link( Node * node1, Node * node2, LinkCoords coord_n1, LinkCoords coord_n2, QString link_type, QString ID )
{
	this->n1 = node1;
	this->n2 = node2;
	this->type = link_type;
	this->id = ID;

	this->coord.push_back( coord_n1 );
	this->coord.push_back( coord_n2 );
}

Link::~Link()
{
	coord.clear();
	property.clear();
}

void Link::setCoord( QString nodeID, Array1D_Vector4 newCoord )
{
	if(n1->id == nodeID) coord[0] = newCoord;
	if(n2->id == nodeID) coord[1] = newCoord;

	//qDebug() << QString("Coordinates changed for %1").arg(nodeID);
}

void Link::setCoordOther( QString nodeID, Array1D_Vector4 newCoord )
{
	if(n1->id == nodeID) coord[1] = newCoord;
	if(n2->id == nodeID) coord[0] = newCoord;

	//qDebug() << QString("Coordinates changed for %1").arg(n1->id == nodeID ? n2->id : n1->id);
}

Array1D_Vector4 Link::getCoord( QString nodeID )
{
	if(n1->id == nodeID) return coord[0];
	if(n2->id == nodeID) return coord[1];
	return Array1D_Vector4(1,Vector4d(0,0,0,0));
}

Array1D_Vector4 Link::getCoordOther( QString nodeID )
{
	return getCoord(otherNode(nodeID)->id);
}

Vector4d Link::getMiddleCoord( QString nodeID )
{
	Array1D_Vector4 nodeCoords = getCoord(nodeID);
	return nodeCoords[ nodeCoords.size() / 2 ];
}

void Link::replace(QString oldNodeID, Node *newNode, Array1D_Vector4 newCoord)
{
	if(!newNode || oldNodeID == newNode->id) return;

	if(n1->id == oldNodeID){
		n1 = newNode;
		coord[0] = newCoord;
	} else if(n2->id == oldNodeID){
		n2 = newNode;
		coord[1] = newCoord;
	} else {
		qDebug() << "Warning: link replace should not happen!";
		return;
	}

	// Change my ID
	QString oldID = id;
	id =  QString("%1 : %2").arg(n1->id).arg(n2->id);

    qDebug() << QString("Link replace [%1] to [%2]").arg(oldID, id);
}

void Link::replaceForced(QString oldNodeID, Node *newNode, Array1D_Vector4 newCoord)
{
    if(n1->id != oldNodeID && n2->id != oldNodeID) return;
    if(!newNode) return;
    if(n1->id == oldNodeID){
        n1 = newNode;
        coord[0] = newCoord;
    } else if(n2->id == oldNodeID){
        n2 = newNode;
        coord[1] = newCoord;
    }
    // Change my ID
    id =  QString("%1 : %2").arg(n1->id).arg(n2->id);
}

Node * Link::otherNode( QString nodeID )
{
	if(n1->id == nodeID) return n2;
	else return n1;
}

Node * Link::getNode( QString nodeID )
{
	if(n1->id == nodeID) return n1;
	else return n2;
}

void Link::draw(bool isShowLine)
{
	glDisable( GL_LIGHTING );
	glEnable( GL_POINT_SMOOTH );

	std::vector<Vector3> linkPos;
	std::vector<Node*> ns;

	for(int j = 0; j < (int)coord[0].size(); j++)
	{
		Vector3 p1(0,0,0), p2(0,0,0);

        std::vector<Vector3> nf = noFrame();

        n1->get(coord[0][j], p1, nf);
        n2->get(coord[1][j], p2, nf);

		linkPos.push_back(p1);
		linkPos.push_back(p2);

		ns.push_back(n1);
		ns.push_back(n2);
	}

	for(int i = 0; i < (int)linkPos.size(); i++)
	{
		// Blue
		glPointSize(3);
		if(ns[i]->type() == CURVE) 
			glColor3d(0,0,1);
		else
			glColor3d(0,1,1);
		glBegin(GL_POINTS);glVector3(linkPos[i]);glEnd();

		// White
		glPointSize(5);
		glColor3d(1,1,1);glBegin(GL_POINTS);glVector3(linkPos[i]);glEnd();
	}

    if(isShowLine)
    {
        glColor3d(0,0,0);
        glLineWidth(2);
        glBegin(GL_LINES);
        for(int i = 0; i < (int)linkPos.size() * 0.5; i++)
        {
            glVector3(linkPos[(i*2)]);
            glVector3(linkPos[(i*2)+1]);
        }
        glEnd();
    }

	glEnable(GL_LIGHTING);
}

bool Link::hasNode( QString nodeID )
{
	return n1->id == nodeID || n2->id == nodeID;
}

bool Link::hasNodeProperty( QString propertyName, QVariant propertyValue )
{
	bool pn1 = n1->hasProperty(propertyName) && n1->property[propertyName] == propertyValue;
	bool pn2 = n2->hasProperty(propertyName) && n2->property[propertyName] == propertyValue;

	return pn1 || pn2;
}

Node * Link::getNodeHasProperty( QString propertyName, QVariant propertyValue )
{
	bool pn1 = n1->hasProperty(propertyName) && n1->property[propertyName] == propertyValue;
	bool pn2 = n2->hasProperty(propertyName) && n2->property[propertyName] == propertyValue;

	if(pn1) return n1;
	if(pn2) return n2;
	return NULL;
}

SurfaceMesh::Vector3 Link::position( QString nodeID )
{
	Node * n = n1->id == nodeID ? n1 : n2;
	assert(n->id == nodeID);

	Vector3 pos(0,0,0);

    std::vector<Vector3> nf = noFrame();

    n->get( getMiddleCoord(nodeID), pos, nf);
	return pos;
}

SurfaceMesh::Vector3 Link::positionOther( QString nodeID )
{
	return position(otherNode(nodeID)->id);
}

QVector<Link*> Link::haveProperty( QVector<Link*> links, QString propertyName )
{
	QVector<Link*> result;
	foreach(Link* l, links){
		if(l->hasProperty(propertyName))
			result.push_back(l);
	}
	return result;
}

QVector<Link*> Link::haveProperty( QVector<Link*> links, QString propertyName, QVariant propertyValue )
{
	QVector<Link*> result;
	QVector<Link*> linksWithProp = Link::haveProperty(links, propertyName);
	foreach(Link* l, links){
		if(l->property[propertyName] == propertyValue)
			result.push_back(l);
	}
	return result;
}

void Link::invertCoords( QString nodeID )
{
	if(nodeID == n1->id) coord[0] = inverseCoords(coord[0]);
	if(nodeID == n2->id) coord[1] = inverseCoords(coord[1]);
}

Vector3 Link::delta()
{
    Vector3 d(0,0,0), p1(0,0,0), p2(0,0,0);

	if(n1 && n2)
	{
		p1 = position(n2->id);
		p2 = position(n1->id);
	}

	d = p1 - p2;
	return d;
}

void Link::pushState()
{
	state["n1"].setValue(n1);
	state["n2"].setValue(n2);
	state["id"].setValue(id);
	state["type"].setValue(type);
	state["coord"].setValue(coord);
	//state["property"].setValue(property); // Issue: this changes over time

	property["modified"] = true;
}

void Link::popState()
{
	if(!state.contains("n1")) return;

	n1 = state["n1"].value<Node*>();
	n2 = state["n2"].value<Node*>();
	id = state["id"].toString();
	type = state["type"].toString();
	coord = state["coord"].value< std::vector<LinkCoords> >();
	//property = state["coord"].value<PropertyMap>();

	property["modified"] = false;
}

void Link::clearState()
{
	state.clear();
}

bool Link::isInState( QString nodeID )
{
	if(!state.contains("n1")) return false;
	return state["n1"].value<Node*>()->id == nodeID || state["n2"].value<Node*>()->id == nodeID;
}

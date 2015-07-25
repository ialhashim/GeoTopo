#include "SurfaceMeshModel.h"
using namespace opengp;
using namespace SurfaceMesh;

SurfaceMeshModel::SurfaceMeshModel(QString path, QString name) : path(path), name(name){
}

SurfaceMeshModel *SurfaceMeshModel::clone()
{
    std::vector<Surface_mesh::Face> selected_faces;
    for(Face f : faces()) selected_faces.push_back(f);
    return clone(selected_faces);
}

SurfaceMeshModel *SurfaceMeshModel::clone(std::vector<Surface_mesh::Vertex> subset)
{
    /// Remove possible duplicates
    std::vector<Surface_mesh::Face> selected_faces;
    std::sort( subset.begin(), subset.end() );
    subset.erase( unique( subset.begin(), subset.end() ), subset.end() );

    for(Vertex v : subset)
        for(Halfedge h : halfedges(v))
            selected_faces.push_back(face(h));

    return clone(selected_faces);
}

SurfaceMeshModel *SurfaceMeshModel::clone(std::vector<Surface_mesh::Face> subset)
{
    /// Remove possible duplicates
    std::sort( subset.begin(), subset.end() );
    subset.erase( unique( subset.begin(), subset.end() ), subset.end() );

    SurfaceMeshModel * m = new SurfaceMeshModel("clone.obj", this->name + "_clone");

    Vector3VertexProperty points = vertex_coordinates();

    QSet<int> vertSet;
    QMap<Vertex,Vertex> vmap;
    for(Face f: subset){
        if(!is_valid(f)) continue;
        Surface_mesh::Vertex_around_face_circulator vit = vertices(f),vend=vit;
        do{ vertSet.insert(vit.operator *().idx()); } while(++vit != vend);
    }
    for(int vidx: vertSet){
        vmap[Vertex(vidx)] = Vertex(vmap.size());
        m->add_vertex( points[Vertex(vidx)] );
    }
    for(Face f: subset){
        if(!is_valid(f)) continue;
        std::vector<Vertex> pnts;
        Surface_mesh::Vertex_around_face_circulator vit = vertices(f),vend=vit;
        do{ pnts.push_back(vmap[vit.operator *()]); } while(++vit != vend);
        m->add_face(pnts);
    }

    m->update_face_normals();
    m->update_vertex_normals();
    m->updateBoundingBox();

    return m;
}

void SurfaceMeshModel::updateBoundingBox(){
    Vector3VertexProperty points = vertex_coordinates();
    _bbox.setNull();
    for(Vertex vit : this->vertices())
        _bbox.extend( points[vit] );
}

void SurfaceMeshModel::remove_vertex(Vertex v){
#if 0
    /// More needs to be done.. halfedges need to be cleaned up
    if( !is_valid(v) ) return;        
    for(Face f : this->faces(v))
        this->fdeleted_[f] = true;
#endif
    this->vdeleted_[v] = true;
    this->garbage_ = true;
}

Vector3VertexProperty SurfaceMeshModel::vertex_coordinates(bool create_if_missing){
    if(create_if_missing)
        return vertex_property<Vector3>(VPOINT,Vector3(0.0,0.0,0.0));
    else
        return get_vertex_property<Vector3>(VPOINT);
}

Vector3VertexProperty SurfaceMeshModel::vertex_normals(bool create_if_missing){
    if(create_if_missing)
        return vertex_property<Vector3>(VNORMAL,Vector3(0.0,0.0,1.0));
    else
        return get_vertex_property<Vector3>(VNORMAL);
}

Vector3FaceProperty SurfaceMeshModel::face_normals(bool create_if_missing){
    if(create_if_missing)
        return face_property<Vector3>(FNORMAL,Vector3(0.0,0.0,1.0));
    else
        return get_face_property<Vector3>(FNORMAL);
}

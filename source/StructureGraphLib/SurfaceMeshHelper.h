#pragma once
#include "SurfaceMeshModel.h"

namespace opengp{

/// Simplified access to property types and store names of defined properties
namespace SurfaceMesh{

/// @note when inheriting from this class, always use "class Subclass : public virtual SurfaceMeshHelper{}"
class SurfaceMeshHelper{
protected:
    SurfaceMeshModel* mesh;
    Vector3VertexProperty points;
    ScalarVertexProperty  varea;  /// NULL
    Vector3VertexProperty vnormal; /// NULL
    Vector3FaceProperty   fnormal; /// NULL
    ScalarFaceProperty   farea; /// NULL
    ScalarEdgeProperty    elenght;  /// NULL
    
public:
    SurfaceMeshHelper(SurfaceMeshModel* mesh) : mesh(mesh){
        points = mesh->vertex_property<Point>(VPOINT);
        vnormal = mesh->get_vertex_property<Vector3>(VNORMAL);
        varea = mesh->get_vertex_property<Scalar>(VAREA);
        fnormal = mesh->get_face_property<Vector3>(FNORMAL);
        farea = mesh->get_face_property<Scalar>(FAREA);
        elenght = mesh->get_edge_property<Scalar>(ELENGTH);
    }
    
    ScalarVertexProperty scalarVertexProperty(const std::string property, Scalar init){
        return mesh->vertex_property<Scalar>(property,init);
    }
    ScalarVertexProperty getScalarVertexProperty(const std::string property){
        ScalarVertexProperty prop = mesh->get_vertex_property<Scalar>(property);
        return prop;
    }
    ScalarHalfedgeProperty getScalarHalfedgeProperty(const std::string property){
        ScalarHalfedgeProperty prop = mesh->get_halfedge_property<Scalar>(property);
        return prop;
    }

    Vector3VertexProperty getVector3VertexProperty(const std::string property){
        Vector3VertexProperty prop = mesh->get_vertex_property<Vector3>(property);
        return prop;
    }
    Vector3VertexProperty defaultedVector3VertexProperty(const std::string property, Vector3 init){
        return mesh->vertex_property<Vector3>(property,init);
    }
    
    Vector3FaceProperty vector3FaceProperty(const std::string property, Vector3 init){
        return mesh->face_property<Vector3>(property,init);
    }
    
    Surface_mesh::Face_property<Scalar> computeFaceAreas(std::string property=FAREA){
        farea = mesh->face_property<Scalar>(property);

        QVector<Vector3> pnts;

        for (auto fit : mesh->faces()){
            // Collect points of face
            for(auto vit : mesh->vertices(fit)) pnts.push_back(points[vit]);

            farea[fit] = 0.5 * (pnts[1] - pnts[0]).cross(pnts[2] - pnts[0]).norm();
        }

        return farea;
    }
    
    ScalarEdgeProperty computeEdgeLengths(std::string property=ELENGTH){
        elenght = mesh->edge_property<Scalar>(property,0.0f);
        for(auto eit : mesh->edges())
            elenght[eit] = mesh->edge_length(eit);
        return elenght;
    }


    ScalarVertexProperty computeVertexBarycentricArea(const std::string property=VAREA){
        varea = mesh->vertex_property<Scalar>(property);
        // Scalar a;
        // Vertex v0,v1,v2;
        for(auto v : mesh->vertices())
            varea[v] = 0.0;
        //throw StarlabException("'computeVertexBarycentricArea' not implemented yet");
        return varea;
    }

    ScalarVertexProperty computeVertexVoronoiArea(const std::string property=VAREA){
        varea = mesh->vertex_property<Scalar>(property);
        Scalar a;
        Surface_mesh::Vertex v0,v1,v2;
        for(auto v : mesh->vertices())
            varea[v] = 0.0;
        for(auto f : mesh->faces()){
            Surface_mesh::Vertex_around_face_circulator vfit = mesh->vertices(f);
            v0 = vfit.operator *();
            v1 = (++vfit).operator *();
            v2 = (++vfit).operator *();
            
            // compute area
            a = 0.5 * (points[v1]-points[v0]).cross(points[v2]-points[v0]).norm();
            
            // distribute area to vertices
            varea[v0] += a/3.0;
            varea[v1] += a/3.0;
            varea[v2] += a/3.0;
        }
        return varea;
    }

    template<class Type>
    Surface_mesh::Vertex_property<Type> smoothVertexProperty(const std::string property, int iterations = 1, Type defaultValue = Type()){
        Surface_mesh::Vertex_property<Type> vprop = mesh->vertex_property<Type>(property, defaultValue);

        for(int i = 0; i < iterations; i++)
        {
            std::vector<Type> newValues(mesh->n_vertices(), defaultValue);

            // average the values of neighbors
            for( auto v : mesh->vertices() ){
                for( auto vj : mesh->vertices(v) )
                    newValues[ v.idx() ] += vprop[ vj ];
                newValues[ v.idx() ] /= mesh->valence(v);
            }

            // copy results back to property
            for(auto v : mesh->vertices())
                vprop[v] = newValues[v.idx()];
        }

        return vprop;
    }
};

/// name used by this helper to store information
const std::string FBARYCENTER = "f:barycenter";

/// @brief a helper class to compute face centroids of
/// Usage:
/// @see http://en.wikipedia.org/wiki/Centroid
class FaceBarycenterHelper : public SurfaceMeshHelper{
    Vector3FaceProperty fbarycenter;
    Vector3VertexProperty vpoints;

public:
    FaceBarycenterHelper(SurfaceMeshModel* mesh) : SurfaceMeshHelper(mesh){
        fbarycenter = mesh->face_property<Vector3>(FBARYCENTER);
        vpoints = mesh->vertex_property<Vector3>(VPOINT);
    }

    /// Allows the usage of:
    /// Vector3FaceProperty fbary = FaceBarycenterHelper(mesh);
    /// Uses SurfaceMesh::FBARYCENTER to store the property
    operator Vector3FaceProperty(){ return compute(); }

    /// main function
    Vector3FaceProperty compute(){
        for(auto fit : mesh->faces())
            fbarycenter[fit] = barycenter(fit);
        return fbarycenter;
    }

    /// helper function
    Vector3 barycenter(SurfaceMeshModel::Face fit){
        int counter = 0;
        Vector3 accumulator(0,0,0);
        for(auto v : mesh->vertices(fit)){
            accumulator += vpoints[v];
            counter++;
        }
        accumulator = accumulator / counter;
        return accumulator;
    }
};

}

}

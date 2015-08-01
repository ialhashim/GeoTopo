#pragma once

#include <QDebug>
#include <QString>

#include "surface_mesh/Surface_mesh.h"

//== NAMESPACE ================================================================
namespace opengp {
//=============================================================================

/// @defgroup surfacemesh SurfaceMesh
namespace SurfaceMesh{

/// @defgroup surfacemesh_basic_types Basic types
/// @ingroup surfacemesh
/// @{
    typedef Eigen::Vector3d         Vector3;             ///< 3D Vector type
    typedef Eigen::Vector3d         Point;               ///< Point type
    typedef Eigen::Vector3d         Normal;              ///< Point type
    typedef Eigen::Vector3d         Color;               ///< Point type
    typedef int                     Integer;             ///< int
    typedef unsigned int            Counter;             ///< To count stuff
    typedef unsigned int            Size;                ///< @obsolete To index stuff (i.e. matlab pointer)

/// @}

/// @defgroup surfacemesh_compatibility_types Back-compatibility types
/// @ingroup surfacemesh
/// @{
    // these are to avoid problems from conversion from Surface_mesh::Vector
    typedef Eigen::Vector3d Vec3d;
	typedef Eigen::Vector4d Vec4d;
	typedef Eigen::Vector3i Vec3i;
	typedef Eigen::Vector3f Vec3f;
	typedef Eigen::Vector2f Vec2f;
/// @}

/// @defgroup surfacemesh_property_names Default property names
/// The std::string constants you should use to access SurfaceMesh dynamic properties.
/// For example to obtain the coordinate property you should use: <br>
/// @code
/// Vector3VertexProperty points = mesh->vertex_property<Vector3>(VPOINT);
/// @endcode
/// 
/// @ingroup surfacemesh
/// @{ 
    const std::string VPOINT = "v:point";           ///< vertex coordinates
    const std::string VNORMAL = "v:normal";         ///< vertex normals
    const std::string VCOLOR = "v:color";           ///< vertex color
    const std::string VAREA = "v:area";             ///< vertex areas
    const std::string VQUALITY = "v:quality";       ///< vertex quality
    const std::string FNORMAL = "f:normal";         ///< face normals
    const std::string FAREA = "f:area";             ///< face area
    const std::string ELENGTH = "e:length";         ///< edge length
    const std::string FSELECTED = "f:selected";     ///< is face selected?    
/// @}
  
/// @defgroup surfacemesh_property_types Default property types
/// Some default property types, for example Surface_mesh::Vertex_property<Scalar> becomes ScalarVertexProperty
/// @ingroup surfacemesh
/// @{
    // Default Vertex properties
    typedef Surface_mesh::Vertex_property<Scalar>   ScalarVertexProperty;   ///< A scalar associated to a vertex.
    typedef Surface_mesh::Vertex_property<Integer>  IntegerVertexProperty;  ///< An (signed) integer number associated to a vertex.
    typedef Surface_mesh::Vertex_property<Vector3>  Vector3VertexProperty;  ///< An Vector3 associated to a vertex.
    typedef Surface_mesh::Vertex_property<bool>     BoolVertexProperty;     ///< A boolean associated to a vertex.
    // Default Face properties
    typedef Surface_mesh::Face_property<Scalar>     ScalarFaceProperty;     ///< A scalar associated to a face.
    typedef Surface_mesh::Face_property<Vector3>    Vector3FaceProperty;    ///< A Vector3 associated to a face.
    typedef Surface_mesh::Face_property<bool>       BoolFaceProperty;       ///< A boolean associated to a face.
    // Default Edge properties
    typedef Surface_mesh::Edge_property<Scalar>     ScalarEdgeProperty;     ///< A scalar associated to an edge.
    typedef Surface_mesh::Edge_property<bool>       BoolEdgeProperty;       ///< A boolean associated to an edge.
    // Default Halfedge properties
    typedef Surface_mesh::Halfedge_property<Scalar> ScalarHalfedgeProperty; ///< A scalar associated to an halfedge.
/// @}    

    class SurfaceMeshForEachOneRingEdgesHelper;

/**
 * @brief A starlab Model for the Surface_mesh datatype
 * @defgroup surfacemesh SurfaceMeshModel
 */
class SurfaceMeshModel : public QObject, public Surface_mesh {
    Q_OBJECT

/// @{ Basic Model Implementation
public:
    SurfaceMeshModel(QString path=QString(), QString name=QString());
    void updateBoundingBox();
    Eigen::AlignedBox3d bbox(){ return _bbox; }
/// @}

/// @{ Cloning operations
    SurfaceMeshModel * clone();
    SurfaceMeshModel * clone(std::vector<Surface_mesh::Vertex> subset);
    SurfaceMeshModel * clone(std::vector<Surface_mesh::Face> subset);
/// @}
    SurfaceMeshForEachOneRingEdgesHelper onering_hedges(Vertex v);

/// @}

/// @{ Query existence of basic properties
    bool has_vertex_normals(){ return has_vertex_property<Vector3>(VNORMAL); }
    bool has_face_normals(){ return has_face_property<Vector3>(FNORMAL); }
/// @}
    
/// @{ Access to default properties
    Vector3VertexProperty vertex_coordinates(bool create_if_missing=false);
    Vector3VertexProperty vertex_normals(bool create_if_missing=false);
    Vector3FaceProperty   face_normals(bool create_if_missing=false);
/// @}
    
/// @{ forced garbage collection!!
    void garbage_collection(){ garbage_ = true; Surface_mesh::garbage_collection(); }

/// @}

/// @{ Extra exposed functionality
    /// @brief Removes vertex
    void remove_vertex(Vertex v);
/// @}
///
    QString name;         /// name of the model (to be used in layer pane)
    QString path;         /// full path of the file associated to the model

protected:
    Eigen::AlignedBox3d _bbox;
};

} // namespace SurfaceMesh


//=============================================================================
} // namespace opengp
//=============================================================================

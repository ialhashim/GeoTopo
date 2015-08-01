//== INCLUDES =================================================================

#include <Surface_mesh.h>
#include <IO.h>
#include <cctype>
#include <algorithm>
#include <string>

//== NAMESPACE ================================================================


namespace opengp {


//== IMPLEMENTATION ===========================================================

namespace{
inline char easytolower(char in){
    return std::tolower(in);
} 
} // ::anonymous

bool read_mesh(Surface_mesh& mesh, const std::string& filename)
{
    // clear mesh before reading from file
    mesh.clear();

    // extract file extension
    std::string::size_type dot(filename.rfind("."));
    if (dot == std::string::npos) return false;
    std::string ext = filename.substr(dot+1, filename.length()-dot-1);
    std::transform(ext.begin(), ext.end(), ext.begin(), easytolower);

    // extension determines reader
    if (ext == "off")
    {
        return read_off(mesh, filename);
    }
    else if (ext == "obj")
    {
        return read_obj(mesh, filename);
    }
    else if (ext == "stl")
    {
        return read_stl(mesh, filename);
    }

    // we didn't find a reader module
    return false;
}


//-----------------------------------------------------------------------------


bool write_mesh(const Surface_mesh& mesh, const std::string& filename)
{
    // extract file extension
    std::string::size_type dot(filename.rfind("."));
    if (dot == std::string::npos) return false;
    std::string ext = filename.substr(dot+1, filename.length()-dot-1);
    std::transform(ext.begin(), ext.end(), ext.begin(), easytolower);


    // extension determines reader
    if (ext == "off")
    {
        return write_off(mesh, filename);
    }
    else if(ext=="obj")
    {
        return write_obj(mesh, filename);
    }

    // we didn't find a writer module
    return false;
}


//=============================================================================
} // namespace opengp
//=============================================================================

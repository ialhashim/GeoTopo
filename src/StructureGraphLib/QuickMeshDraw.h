#pragma once

#include <qgl.h>
#include "SurfaceMeshModel.h"

struct QuickMeshDraw{

    static void drawMeshSolid( SurfaceMeshModel * mesh, QColor c = QColor(255,255,255,255), Vector3 translation = Vector3(0,0,0) )
    {
        if(!mesh) return;

        if(!mesh->has_face_property<Vector3>("f:normal")) mesh->update_face_normals();

        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_LIGHTING);

        glColorQt(c);

        Surface_mesh::Vertex_property<Vector3> points = mesh->vertex_property<Vector3>("v:point");
        Surface_mesh::Face_property<Vector3> fnormals = mesh->face_property<Vector3>("f:normal");

        glPushMatrix();
        glTranslated(translation[0], translation[1], translation[2]);

        glBegin(GL_TRIANGLES);
        for (auto fit : mesh->faces()){
            glNormal3dv( fnormals[fit].data() );
            for(auto fvit : mesh->vertices(fit)) glVertex3dv(points[fvit].data());
        }
        glEnd();

        glPopMatrix();
    }

    static void drawMeshWireFrame( SurfaceMeshModel * mesh )
    {
        if(!mesh) return;

        Surface_mesh::Vertex_property<Vector3> points = mesh->vertex_property<Vector3>("v:point");
        Surface_mesh::Face_property<Vector3> fnormals = mesh->face_property<Vector3>("f:normal");

        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glColor4d(0,1,1, 0.25);
        glLineWidth(1.0f);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);

        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        for (auto fit : mesh->faces()){
            glBegin(GL_POLYGON);
            glNormal3dv( fnormals[fit].data() );

            for(auto fvit : mesh->vertices(fit)) glVertex3dv(points[fvit].data());
            glEnd();
        }
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );

        glDisable(GL_CULL_FACE);
    }

    static void drawMeshName( SurfaceMeshModel * mesh, int name = 0 )
    {
        glPushName( name );

        Surface_mesh::Vertex_property<Vector3> points = mesh->vertex_property<Vector3>("v:point");

        glBegin(GL_TRIANGLES);
        for (auto fit : mesh->faces()){
            for(auto fvit : mesh->vertices(fit)) glVertex3dv(points[fvit].data());
        }
        glEnd();

        glPopName();
    }
};

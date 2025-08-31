// Copyright (C) 2014- Danyal Zia
// Copyright (C) 2009-2013 Josiah Hartzell (Skyreign Software)
// This file is part of the "irrBullet" Bullet physics wrapper.
// For conditions of distribution and use, see copyright notice in irrBullet.h
// The above copyright notice and its accompanying information must remain here.

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include "irrBulletBvhTriangleMeshShape.h"
#include <iostream>

using namespace irr;
using namespace core;
using namespace scene;

IBvhTriangleMeshShape::IBvhTriangleMeshShape(ISceneNode *n, IMesh *collMesh, f32 m)
{
    node = n;
    mass = m;

	type = ECollisionShapeType::ECST_BVHTRIMESH;

    createShape(collMesh);
}


void IBvhTriangleMeshShape::createShape(IMesh *mesh)
{
    if(node->getType() == irr::scene::ESNT_TERRAIN)
    {
        CollisionMesh = createTriangleMesh((irr::scene::ITerrainSceneNode*) node);
    }
    else
    {
        CollisionMesh = createTriangleMesh(mesh);
    }

    std::cout << "TRIS: " << CollisionMesh->getNumTriangles() << std::endl;

    shape =  new btBvhTriangleMeshShape(CollisionMesh, false, true);

    calculateLocalInertia(getMass(), vector3df(0.0f,0.0f,0.0f));
}

IBvhTriangleMeshShape::~IBvhTriangleMeshShape()
{
    //dtor
}

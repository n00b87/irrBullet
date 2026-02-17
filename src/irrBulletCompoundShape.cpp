// Copyright (C) 2014- Danyal Zia
// Copyright (C) 2009-2013 Josiah Hartzell (Skyreign Software)
// This file is part of the "irrBullet" Bullet physics wrapper.
// For conditions of distribution and use, see copyright notice in irrBullet.h
// The above copyright notice and its accompanying information must remain here.

#include <ISceneNode.h>
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "irrBulletCompoundShape.h"

using namespace irr;
using namespace core;
using namespace scene;

ICompoundShape::ICompoundShape(ISceneNode* const n, f32 m, bool overrideMargin)
{
    node = n;
    mass = m;

	type = ECollisionShapeType::ECST_COMPOUND;

    createShape(overrideMargin);
}

void ICompoundShape::createShape(bool overrideMargin)
{
    node->updateAbsolutePosition();

    const vector3df& extent = node->getBoundingBox().getExtent() + f32((overrideMargin) ? 0.04:0.0);

    shape = new btCompoundShape();

	calculateLocalInertia(getMass(), vector3df(0.0f,0.0f,0.0f));
}

ICompoundShape::~ICompoundShape()
{
}


int ICompoundShape::addChildShape (irr::core::matrix4 &localTransform, ICollisionShape *child_shape)
{
    if(!child_shape)
        return -1;

    for(int i = 0; i < ChildShapes.size(); i++)
    {
        if(child_shape == ChildShapes[i])
            return i;
    }

    btCompoundShape* shape_ptr = (btCompoundShape*)this->getPointer();

    btTransform p_transform;
    btTransformFromIrrlichtMatrix(localTransform, p_transform);

    btCollisionShape* child_ptr = child_shape->getPointer();

    shape_ptr->addChildShape(p_transform, child_ptr);
    ChildShapes.push_back(child_shape);

    return ( ((int)(ChildShapes.size())) - 1);
}

void ICompoundShape::removeChildShape (ICollisionShape *child_shape)
{
    if(!child_shape)
        return;

    btCollisionShape* child_ptr = NULL;
    btCompoundShape* shape_ptr = (btCompoundShape*)this->getPointer();

    for(int i = 0; i < ChildShapes.size(); i++)
    {
        if(child_shape == ChildShapes[i])
        {
            child_ptr = ChildShapes[i]->getPointer();
            shape_ptr->removeChildShape(child_ptr);
            ChildShapes[i] = NULL;
            ChildShapes.erase(ChildShapes.begin() + i);
            break;
        }
    }
}

void ICompoundShape::removeChildShapeByIndex (int childShapeindex)
{
    if(childShapeindex < 0 || childShapeindex >= ChildShapes.size())
        return;

    ChildShapes.erase(ChildShapes.begin() + childShapeindex);

    btCompoundShape* shape_ptr = (btCompoundShape*)this->getPointer();
    shape_ptr->removeChildShapeByIndex(childShapeindex);
}

int ICompoundShape::getNumChildShapes ()
{
    return ChildShapes.size();
}

ICollisionShape* ICompoundShape::getChildShape (int index)
{
    if(index < 0 || index >= ChildShapes.size())
        return NULL;

    return ChildShapes[index];
}


irr::core::matrix4& ICompoundShape::getChildTransform (int index)
{
    btCompoundShape* shape_ptr = (btCompoundShape*)this->getPointer();
    btTransform p_transform = shape_ptr->getChildTransform(index);
    irr::core::matrix4 irr_mat;
    btTransformToIrrlichtMatrix(p_transform, irr_mat);
    return irr_mat;
}

void ICompoundShape::updateChildTransform (int childIndex, irr::core::matrix4 &newChildTransform, bool shouldRecalculateLocalAabb)
{
    btCompoundShape* shape_ptr = (btCompoundShape*)this->getPointer();
    btTransform p_transform;
    btTransformFromIrrlichtMatrix(newChildTransform, p_transform);
    shape_ptr->updateChildTransform(childIndex, p_transform, shouldRecalculateLocalAabb);

    btTransformToIrrlichtMatrix(p_transform, newChildTransform);
}

void ICompoundShape::getAabb (irr::core::matrix4 &t, irr::core::vector3df &aabbMin, irr::core::vector3df &aabbMax)
{
    btCompoundShape* shape_ptr = (btCompoundShape*)this->getPointer();
    btTransform p_transform;
    btTransformFromIrrlichtMatrix(t, p_transform);
    btVector3 min_vec(aabbMin.X, aabbMin.Y, aabbMin.Z);
    btVector3 max_vec(aabbMax.X, aabbMax.Y, aabbMax.Z);
    shape_ptr->getAabb(p_transform, min_vec, max_vec);

    aabbMin.set(min_vec.getX(), min_vec.getY(), min_vec.getZ());
    aabbMax.set(max_vec.getX(), max_vec.getY(), max_vec.getZ());

    btTransformToIrrlichtMatrix(p_transform, t);
}

void ICompoundShape::recalculateLocalAabb ()
{
    btCompoundShape* shape_ptr = (btCompoundShape*)this->getPointer();
    shape_ptr->recalculateLocalAabb();
}

btDbvt* ICompoundShape::getDynamicAabbTree ()
{
    btCompoundShape* shape_ptr = (btCompoundShape*)this->getPointer();
    return shape_ptr->getDynamicAabbTree();
}

void ICompoundShape::createAabbTreeFromChildren ()
{
    btCompoundShape* shape_ptr = (btCompoundShape*)this->getPointer();
    shape_ptr->createAabbTreeFromChildren();
}

void ICompoundShape::calculatePrincipalAxisTransform (float *masses, irr::core::matrix4 &principal, irr::core::vector3df &inertia)
{
    btCompoundShape* shape_ptr = (btCompoundShape*)this->getPointer();
    btTransform p_transform;
    btTransformFromIrrlichtMatrix(principal, p_transform);
    btVector3 in_vec(inertia.X, inertia.Y, inertia.Z);
    btScalar* m_float = (btScalar*)masses;
    shape_ptr->calculatePrincipalAxisTransform(m_float, p_transform, in_vec);

    btTransformToIrrlichtMatrix(p_transform, principal);

    inertia.set(in_vec.getX(), in_vec.getY(), in_vec.getZ());
}

int ICompoundShape::getUpdateRevision ()
{
    btCompoundShape* shape_ptr = (btCompoundShape*)this->getPointer();
    return shape_ptr->getUpdateRevision();
}

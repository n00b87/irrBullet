// Copyright (C) 2014- Danyal Zia
// Copyright (C) 2009-2013 Josiah Hartzell (Skyreign Software)
// This file is part of the "irrBullet" Bullet physics wrapper.
// For conditions of distribution and use, see copyright notice in irrBullet.h
// The above copyright notice and its accompanying information must remain here.

#pragma once

#include "irrBulletCollisionShape.h"
#include <vector>

class ICompoundShape : public ICollisionShape
{
public:
	ICompoundShape(irr::scene::ISceneNode* const n, irr::f32 m, bool overrideMargin = false);

	ICompoundShape(const ICompoundShape& other) = default;

	ICompoundShape& operator=(const ICompoundShape& other) = default;

	// move constructor
	ICompoundShape(const ICompoundShape&& other)
	{
		*this = std::move(other);
	}

	// Move assignment operator
	ICompoundShape& operator=(ICompoundShape&& other)
	{
		if (this != &other)
		{
			delete node;

			node = other.node;
			mass = other.mass;

			other.node = nullptr;
			other.mass = 0;
		}
		return *this;
	}

	virtual ~ICompoundShape();


	int addChildShape (irr::core::matrix4 &localTransform, ICollisionShape *child_shape);
	void removeChildShape (ICollisionShape *child_shape);
	void removeChildShapeByIndex (int childShapeindex);
	int getNumChildShapes ();

	ICollisionShape* getChildShape (int index);
    irr::core::matrix4& getChildTransform (int index);

    void updateChildTransform (int childIndex, irr::core::matrix4 &newChildTransform, bool shouldRecalculateLocalAabb=true);

    void getAabb (irr::core::matrix4 &t, irr::core::vector3df &aabbMin, irr::core::vector3df &aabbMax);

    void recalculateLocalAabb ();
    btDbvt* getDynamicAabbTree ();
    void createAabbTreeFromChildren ();
    void calculatePrincipalAxisTransform (float *masses, irr::core::matrix4 &principal, irr::core::vector3df &inertia);

    int getUpdateRevision ();

protected:
	void createShape(bool overrideMargin);

	std::vector<ICollisionShape*> ChildShapes;
};


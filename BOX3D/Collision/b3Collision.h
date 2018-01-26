/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef b3_COLLISION_H
#define b3_COLLISION_H

#include "Box3D/Common/b3Math.h"
#include <climits>

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

class b3Shape;
class b3CircleShape;
class b3EdgeShape;
class b3PolygonShape;

const uint8 b3_nullFeature = UCHAR_MAX;

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
struct b3ContactFeature
{
	enum Type
	{
		e_vertex = 0,
		e_face = 1
	};

	uint8 indexA;		///< Feature index on shapeA
	uint8 indexB;		///< Feature index on shapeB
	uint8 typeA;		///< The feature type on shapeA
	uint8 typeB;		///< The feature type on shapeB
};

/// Contact ids to facilitate warm starting.
union b3ContactID
{
	b3ContactFeature cf;
	uint32 key;					///< Used to quickly compare contact ids.
};

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
struct b3ManifoldPoint
{
	b3Vec2 localPoint;		///< usage depends on manifold type
	float32 normalImpulse;	///< the non-penetration impulse
	float32 tangentImpulse;	///< the friction impulse
	b3ContactID id;			///< uniquely identifies a contact point between two shapes
};

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
struct b3Manifold
{
	enum Type
	{
		e_circles,
		e_faceA,
		e_faceB
	};

	b3ManifoldPoint points[b3_maxManifoldPoints];	///< the points of contact
	b3Vec2 localNormal;								///< not use for Type::e_points
	b3Vec2 localPoint;								///< usage depends on manifold type
	Type type;
	int32 pointCount;								///< the number of manifold points
};

/// This is used to compute the current state of a contact manifold.
struct b3WorldManifold
{
	/// Evaluate the manifold with supplied transforms. This assumes
	/// modest motion from the original state. This does not change the
	/// point count, impulses, etc. The radii must come from the shapes
	/// that generated the manifold.
	void Initialize(const b3Manifold* manifold,
					const b3Transform& xfA, float32 radiusA,
					const b3Transform& xfB, float32 radiusB);

	b3Vec2 normal;								///< world vector pointing from A to B
	b3Vec2 points[b3_maxManifoldPoints];		///< world contact point (point of intersection)
	float32 separations[b3_maxManifoldPoints];	///< a negative value indicates overlap, in meters
};

/// This is used for determining the state of contact points.
enum b3PointState
{
	b3_nullState,		///< point does not exist
	b3_addState,		///< point was added in the update
	b3_persistState,	///< point persisted across the update
	b3_removeState		///< point was removed in the update
};

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
void b3GetPointStates(b3PointState state1[b3_maxManifoldPoints], b3PointState state2[b3_maxManifoldPoints],
					  const b3Manifold* manifold1, const b3Manifold* manifold2);

/// Used for computing contact manifolds.
struct b3ClipVertex
{
	b3Vec2 v;
	b3ContactID id;
};

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
struct b3RayCastInput
{
	b3Vec2 p1, p2;
	float32 maxFraction;
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b3RayCastInput.
struct b3RayCastOutput
{
	b3Vec2 normal;
	float32 fraction;
};

/// An axis aligned bounding box.
struct b3AABB
{
	/// Verify that the bounds are sorted.
	bool IsValid() const;

	/// Get the center of the AABB.
	b3Vec2 GetCenter() const
	{
		return 0.5f * (lowerBound + upperBound);
	}

	/// Get the extents of the AABB (half-widths).
	b3Vec2 GetExtents() const
	{
		return 0.5f * (upperBound - lowerBound);
	}

	/// Get the perimeter length
	float32 GetPerimeter() const
	{
		float32 wx = upperBound.x - lowerBound.x;
		float32 wy = upperBound.y - lowerBound.y;
		return 2.0f * (wx + wy);
	}

	/// Combine an AABB into this one.
	void Combine(const b3AABB& aabb)
	{
		lowerBound = b3Min(lowerBound, aabb.lowerBound);
		upperBound = b3Max(upperBound, aabb.upperBound);
	}

	/// Combine two AABBs into this one.
	void Combine(const b3AABB& aabb1, const b3AABB& aabb3)
	{
		lowerBound = b3Min(aabb1.lowerBound, aabb3.lowerBound);
		upperBound = b3Max(aabb1.upperBound, aabb3.upperBound);
	}

	/// Does this aabb contain the provided AABB.
	bool Contains(const b3AABB& aabb) const
	{
		bool result = true;
		result = result && lowerBound.x <= aabb.lowerBound.x;
		result = result && lowerBound.y <= aabb.lowerBound.y;
		result = result && aabb.upperBound.x <= upperBound.x;
		result = result && aabb.upperBound.y <= upperBound.y;
		return result;
	}

	bool RayCast(b3RayCastOutput* output, const b3RayCastInput& input) const;

	b3Vec2 lowerBound;	///< the lower vertex
	b3Vec2 upperBound;	///< the upper vertex
};

/// Compute the collision manifold between two circles.
void b3CollideCircles(b3Manifold* manifold,
					  const b3CircleShape* circleA, const b3Transform& xfA,
					  const b3CircleShape* circleB, const b3Transform& xfB);

/// Compute the collision manifold between a polygon and a circle.
void b3CollidePolygonAndCircle(b3Manifold* manifold,
							   const b3PolygonShape* polygonA, const b3Transform& xfA,
							   const b3CircleShape* circleB, const b3Transform& xfB);

/// Compute the collision manifold between two polygons.
void b3CollidePolygons(b3Manifold* manifold,
					   const b3PolygonShape* polygonA, const b3Transform& xfA,
					   const b3PolygonShape* polygonB, const b3Transform& xfB);

/// Compute the collision manifold between an edge and a circle.
void b3CollideEdgeAndCircle(b3Manifold* manifold,
							   const b3EdgeShape* polygonA, const b3Transform& xfA,
							   const b3CircleShape* circleB, const b3Transform& xfB);

/// Compute the collision manifold between an edge and a circle.
void b3CollideEdgeAndPolygon(b3Manifold* manifold,
							   const b3EdgeShape* edgeA, const b3Transform& xfA,
							   const b3PolygonShape* circleB, const b3Transform& xfB);

/// Clipping for contact manifolds.
int32 b3ClipSegmentToLine(b3ClipVertex vOut[2], const b3ClipVertex vIn[2],
							const b3Vec2& normal, float32 offset, int32 vertexIndexA);

/// Determine if two generic shapes overlap.
bool b3TestOverlap(	const b3Shape* shapeA, int32 indexA,
					const b3Shape* shapeB, int32 indexB,
					const b3Transform& xfA, const b3Transform& xfB);

// ---------------- Inline Functions ------------------------------------------

inline bool b3AABB::IsValid() const
{
	b3Vec2 d = upperBound - lowerBound;
	bool valid = d.x >= 0.0f && d.y >= 0.0f;
	valid = valid && lowerBound.IsValid() && upperBound.IsValid();
	return valid;
}

inline bool b3TestOverlap(const b3AABB& a, const b3AABB& b)
{
	b3Vec2 d1, d2;
	d1 = b.lowerBound - a.upperBound;
	d2 = a.lowerBound - b.upperBound;

	if (d1.x > 0.0f || d1.y > 0.0f)
		return false;

	if (d2.x > 0.0f || d2.y > 0.0f)
		return false;

	return true;
}

#endif

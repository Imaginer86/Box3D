/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include "Box3D/Collision/b3Collision.h"
#include "Box3D/Collision/b3Distance.h"

void b3WorldManifold::Initialize(const b3Manifold* manifold,
						  const b3Transform& xfA, float32 radiusA,
						  const b3Transform& xfB, float32 radiusB)
{
	if (manifold->pointCount == 0)
	{
		return;
	}

	switch (manifold->type)
	{
	case b3Manifold::e_circles:
		{
			normal.Set(1.0f, 0.0f);
			b3Vec2 pointA = b3Mul(xfA, manifold->localPoint);
			b3Vec2 pointB = b3Mul(xfB, manifold->points[0].localPoint);
			if (b3DistanceSquared(pointA, pointB) > b3_epsilon * b3_epsilon)
			{
				normal = pointB - pointA;
				normal.Normalize();
			}

			b3Vec2 cA = pointA + radiusA * normal;
			b3Vec2 cB = pointB - radiusB * normal;
			points[0] = 0.5f * (cA + cB);
			separations[0] = b3Dot(cB - cA, normal);
		}
		break;

	case b3Manifold::e_faceA:
		{
			normal = b3Mul(xfA.q, manifold->localNormal);
			b3Vec2 planePoint = b3Mul(xfA, manifold->localPoint);
			
			for (int32 i = 0; i < manifold->pointCount; ++i)
			{
				b3Vec2 clipPoint = b3Mul(xfB, manifold->points[i].localPoint);
				b3Vec2 cA = clipPoint + (radiusA - b3Dot(clipPoint - planePoint, normal)) * normal;
				b3Vec2 cB = clipPoint - radiusB * normal;
				points[i] = 0.5f * (cA + cB);
				separations[i] = b3Dot(cB - cA, normal);
			}
		}
		break;

	case b3Manifold::e_faceB:
		{
			normal = b3Mul(xfB.q, manifold->localNormal);
			b3Vec2 planePoint = b3Mul(xfB, manifold->localPoint);

			for (int32 i = 0; i < manifold->pointCount; ++i)
			{
				b3Vec2 clipPoint = b3Mul(xfA, manifold->points[i].localPoint);
				b3Vec2 cB = clipPoint + (radiusB - b3Dot(clipPoint - planePoint, normal)) * normal;
				b3Vec2 cA = clipPoint - radiusA * normal;
				points[i] = 0.5f * (cA + cB);
				separations[i] = b3Dot(cA - cB, normal);
			}

			// Ensure normal points from A to B.
			normal = -normal;
		}
		break;
	}
}

void b3GetPointStates(b3PointState state1[b3_maxManifoldPoints], b3PointState state2[b3_maxManifoldPoints],
					  const b3Manifold* manifold1, const b3Manifold* manifold2)
{
	for (int32 i = 0; i < b3_maxManifoldPoints; ++i)
	{
		state1[i] = b3_nullState;
		state2[i] = b3_nullState;
	}

	// Detect persists and removes.
	for (int32 i = 0; i < manifold1->pointCount; ++i)
	{
		b3ContactID id = manifold1->points[i].id;

		state1[i] = b3_removeState;

		for (int32 j = 0; j < manifold2->pointCount; ++j)
		{
			if (manifold2->points[j].id.key == id.key)
			{
				state1[i] = b3_persistState;
				break;
			}
		}
	}

	// Detect persists and adds.
	for (int32 i = 0; i < manifold2->pointCount; ++i)
	{
		b3ContactID id = manifold2->points[i].id;

		state2[i] = b3_addState;

		for (int32 j = 0; j < manifold1->pointCount; ++j)
		{
			if (manifold1->points[j].id.key == id.key)
			{
				state2[i] = b3_persistState;
				break;
			}
		}
	}
}

// From Real-time Collision Detection, p179.
bool b3AABB::RayCast(b3RayCastOutput* output, const b3RayCastInput& input) const
{
	float32 tmin = -b3_maxFloat;
	float32 tmax = b3_maxFloat;

	b3Vec2 p = input.p1;
	b3Vec2 d = input.p2 - input.p1;
	b3Vec2 absD = b3Abs(d);

	b3Vec2 normal;

	for (int32 i = 0; i < 2; ++i)
	{
		if (absD(i) < b3_epsilon)
		{
			// Parallel.
			if (p(i) < lowerBound(i) || upperBound(i) < p(i))
			{
				return false;
			}
		}
		else
		{
			float32 inv_d = 1.0f / d(i);
			float32 t1 = (lowerBound(i) - p(i)) * inv_d;
			float32 t2 = (upperBound(i) - p(i)) * inv_d;

			// Sign of the normal vector.
			float32 s = -1.0f;

			if (t1 > t2)
			{
				b3Swap(t1, t2);
				s = 1.0f;
			}

			// Push the min up
			if (t1 > tmin)
			{
				normal.SetZero();
				normal(i) = s;
				tmin = t1;
			}

			// Pull the max down
			tmax = b3Min(tmax, t2);

			if (tmin > tmax)
			{
				return false;
			}
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if (tmin < 0.0f || input.maxFraction < tmin)
	{
		return false;
	}

	// Intersection.
	output->fraction = tmin;
	output->normal = normal;
	return true;
}

// Sutherland-Hodgman clipping.
int32 b3ClipSegmentToLine(b3ClipVertex vOut[2], const b3ClipVertex vIn[2],
						const b3Vec2& normal, float32 offset, int32 vertexIndexA)
{
	// Start with no output points
	int32 numOut = 0;

	// Calculate the distance of end points to the line
	float32 distance0 = b3Dot(normal, vIn[0].v) - offset;
	float32 distance1 = b3Dot(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0f) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0f) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0f)
	{
		// Find intersection point of edge and plane
		float32 interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

		// VertexA is hitting edgeB.
		vOut[numOut].id.cf.indexA = static_cast<uint8>(vertexIndexA);
		vOut[numOut].id.cf.indexB = vIn[0].id.cf.indexB;
		vOut[numOut].id.cf.typeA = b3ContactFeature::e_vertex;
		vOut[numOut].id.cf.typeB = b3ContactFeature::e_face;
		++numOut;
	}

	return numOut;
}

bool b3TestOverlap(	const b3Shape* shapeA, int32 indexA,
					const b3Shape* shapeB, int32 indexB,
					const b3Transform& xfA, const b3Transform& xfB)
{
	b3DistanceInput input;
	input.proxyA.Set(shapeA, indexA);
	input.proxyB.Set(shapeB, indexB);
	input.transformA = xfA;
	input.transformB = xfB;
	input.useRadii = true;

	b3SimplexCache cache;
	cache.count = 0;

	b3DistanceOutput output;

	b3Distance(&output, &cache, &input);

	return output.distance < 10.0f * b3_epsilon;
}

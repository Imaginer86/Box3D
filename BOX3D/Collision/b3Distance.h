
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

#ifndef b3_DISTANCE_H
#define b3_DISTANCE_H

#include "Box3D/Common/b3Math.h"

class b3Shape;

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
struct b3DistanceProxy
{
	b3DistanceProxy() : m_vertices(nullptr), m_count(0), m_radius(0.0f) {}

	/// Initialize the proxy using the given shape. The shape
	/// must remain in scope while the proxy is in use.
	void Set(const b3Shape* shape, int32 index);

	/// Get the supporting vertex index in the given direction.
	int32 GetSupport(const b3Vec2& d) const;

	/// Get the supporting vertex in the given direction.
	const b3Vec2& GetSupportVertex(const b3Vec2& d) const;

	/// Get the vertex count.
	int32 GetVertexCount() const;

	/// Get a vertex by index. Used by b3Distance.
	const b3Vec2& GetVertex(int32 index) const;

	b3Vec2 m_buffer[2];
	const b3Vec2* m_vertices;
	int32 m_count;
	float32 m_radius;
};

/// Used to warm start b3Distance.
/// Set count to zero on first call.
struct b3SimplexCache
{
	float32 metric;		///< length or area
	uint16 count;
	uint8 indexA[3];	///< vertices on shape A
	uint8 indexB[3];	///< vertices on shape B
};

/// Input for b3Distance.
/// You have to option to use the shape radii
/// in the computation. Even 
struct b3DistanceInput
{
	b3DistanceProxy proxyA;
	b3DistanceProxy proxyB;
	b3Transform transformA;
	b3Transform transformB;
	bool useRadii;
};

/// Output for b3Distance.
struct b3DistanceOutput
{
	b3Vec2 pointA;		///< closest point on shapeA
	b3Vec2 pointB;		///< closest point on shapeB
	float32 distance;
	int32 iterations;	///< number of GJK iterations used
};

/// Compute the closest points between two shapes. Supports any combination of:
/// b3CircleShape, b3PolygonShape, b3EdgeShape. The simplex cache is input/output.
/// On the first call set b3SimplexCache.count to zero.
void b3Distance(b3DistanceOutput* output,
				b3SimplexCache* cache, 
				const b3DistanceInput* input);


//////////////////////////////////////////////////////////////////////////

inline int32 b3DistanceProxy::GetVertexCount() const
{
	return m_count;
}

inline const b3Vec2& b3DistanceProxy::GetVertex(int32 index) const
{
	b3Assert(0 <= index && index < m_count);
	return m_vertices[index];
}

inline int32 b3DistanceProxy::GetSupport(const b3Vec2& d) const
{
	int32 bestIndex = 0;
	float32 bestValue = b3Dot(m_vertices[0], d);
	for (int32 i = 1; i < m_count; ++i)
	{
		float32 value = b3Dot(m_vertices[i], d);
		if (value > bestValue)
		{
			bestIndex = i;
			bestValue = value;
		}
	}

	return bestIndex;
}

inline const b3Vec2& b3DistanceProxy::GetSupportVertex(const b3Vec2& d) const
{
	int32 bestIndex = 0;
	float32 bestValue = b3Dot(m_vertices[0], d);
	for (int32 i = 1; i < m_count; ++i)
	{
		float32 value = b3Dot(m_vertices[i], d);
		if (value > bestValue)
		{
			bestIndex = i;
			bestValue = value;
		}
	}

	return m_vertices[bestIndex];
}

#endif

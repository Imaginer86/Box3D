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

#ifndef b3_ISLAND_H
#define b3_ISLAND_H

#include "Box3D/Common/b3Math.h"
#include "Box3D/Dynamics/b3Body.h"
#include "Box3D/Dynamics/b3TimeStep.h"

class b3Contact;
class b3Joint;
class b3StackAllocator;
class b3ContactListener;
struct b3ContactVelocityConstraint;
struct b3Profile;

/// This is an internal class.
class b3Island
{
public:
	b3Island(int32 bodyCapacity, int32 contactCapacity, int32 jointCapacity,
			b3StackAllocator* allocator, b3ContactListener* listener);
	~b3Island();

	void Clear()
	{
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
	}

	void Solve(b3Profile* profile, const b3TimeStep& step, const b3Vec2& gravity, bool allowSleep);

	void SolveTOI(const b3TimeStep& subStep, int32 toiIndexA, int32 toiIndexB);

	void Add(b3Body* body)
	{
		b3Assert(m_bodyCount < m_bodyCapacity);
		body->m_islandIndex = m_bodyCount;
		m_bodies[m_bodyCount] = body;
		++m_bodyCount;
	}

	void Add(b3Contact* contact)
	{
		b3Assert(m_contactCount < m_contactCapacity);
		m_contacts[m_contactCount++] = contact;
	}

	void Add(b3Joint* joint)
	{
		b3Assert(m_jointCount < m_jointCapacity);
		m_joints[m_jointCount++] = joint;
	}

	void Report(const b3ContactVelocityConstraint* constraints);

	b3StackAllocator* m_allocator;
	b3ContactListener* m_listener;

	b3Body** m_bodies;
	b3Contact** m_contacts;
	b3Joint** m_joints;

	b3Position* m_positions;
	b3Velocity* m_velocities;

	int32 m_bodyCount;
	int32 m_jointCount;
	int32 m_contactCount;

	int32 m_bodyCapacity;
	int32 m_contactCapacity;
	int32 m_jointCapacity;
};

#endif

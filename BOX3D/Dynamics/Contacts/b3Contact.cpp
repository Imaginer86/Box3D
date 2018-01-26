/*
* Copyright (c) 2006-2009 Erin Catto http://www.Box3D.org
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

#include "Box3D/Dynamics/Contacts/b3Contact.h"
//#include "Box3D/Dynamics/Contacts/b3CircleContact.h"
//#include "Box3D/Dynamics/Contacts/b3PolygonAndCircleContact.h"
//#include "Box3D/Dynamics/Contacts/b3PolygonContact.h"
//#include "Box3D/Dynamics/Contacts/b3EdgeAndCircleContact.h"
//#include "Box3D/Dynamics/Contacts/b3EdgeAndPolygonContact.h"
//#include "Box3D/Dynamics/Contacts/b3ChainAndCircleContact.h"
//#include "Box3D/Dynamics/Contacts/b3ChainAndPolygonContact.h"
//#include "Box3D/Dynamics/Contacts/b3ContactSolver.h"

#include "Box3D/Collision/b3Collision.h"
#include "Box3D/Collision/b3TimeOfImpact.h"
#include "Box3D/Collision/Shapes/b3Shape.h"
#include "Box3D/Common/b3BlockAllocator.h"
#include "Box3D/Dynamics/b3Body.h"
#include "Box3D/Dynamics/b3Fixture.h"
#include "Box3D/Dynamics/b3World.h"

b3ContactRegister b3Contact::s_registers[b3Shape::e_typeCount][b3Shape::e_typeCount];
bool b3Contact::s_initialized = false;

void b3Contact::InitializeRegisters()
{
	//!!!
	//AddType(b3CircleContact::Create, b3CircleContact::Destroy, b3Shape::e_circle, b3Shape::e_circle);
	//AddType(b3PolygonAndCircleContact::Create, b3PolygonAndCircleContact::Destroy, b3Shape::e_polygon, b3Shape::e_circle);
	//AddType(b3PolygonContact::Create, b3PolygonContact::Destroy, b3Shape::e_polygon, b3Shape::e_polygon);
	//AddType(b3EdgeAndCircleContact::Create, b3EdgeAndCircleContact::Destroy, b3Shape::e_edge, b3Shape::e_circle);
	//AddType(b3EdgeAndPolygonContact::Create, b3EdgeAndPolygonContact::Destroy, b3Shape::e_edge, b3Shape::e_polygon);
	//AddType(b3ChainAndCircleContact::Create, b3ChainAndCircleContact::Destroy, b3Shape::e_chain, b3Shape::e_circle);
	//AddType(b3ChainAndPolygonContact::Create, b3ChainAndPolygonContact::Destroy, b3Shape::e_chain, b3Shape::e_polygon);
}

void b3Contact::AddType(b3ContactCreateFcn* createFcn, b3ContactDestroyFcn* destoryFcn,
						b3Shape::Type type1, b3Shape::Type type2)
{
	b3Assert(0 <= type1 && type1 < b3Shape::e_typeCount);
	b3Assert(0 <= type2 && type2 < b3Shape::e_typeCount);
	
	s_registers[type1][type2].createFcn = createFcn;
	s_registers[type1][type2].destroyFcn = destoryFcn;
	s_registers[type1][type2].primary = true;

	if (type1 != type2)
	{
		s_registers[type2][type1].createFcn = createFcn;
		s_registers[type2][type1].destroyFcn = destoryFcn;
		s_registers[type2][type1].primary = false;
	}
}

b3Contact* b3Contact::Create(b3Fixture* fixtureA, int32 indexA, b3Fixture* fixtureB, int32 indexB, b3BlockAllocator* allocator)
{
	if (s_initialized == false)
	{
		InitializeRegisters();
		s_initialized = true;
	}

	b3Shape::Type type1 = fixtureA->GetType();
	b3Shape::Type type2 = fixtureB->GetType();

	b3Assert(0 <= type1 && type1 < b3Shape::e_typeCount);
	b3Assert(0 <= type2 && type2 < b3Shape::e_typeCount);
	
	b3ContactCreateFcn* createFcn = s_registers[type1][type2].createFcn;
	if (createFcn)
	{
		if (s_registers[type1][type2].primary)
		{
			return createFcn(fixtureA, indexA, fixtureB, indexB, allocator);
		}
		else
		{
			return createFcn(fixtureB, indexB, fixtureA, indexA, allocator);
		}
	}
	else
	{
		return nullptr;
	}
}

void b3Contact::Destroy(b3Contact* contact, b3BlockAllocator* allocator)
{
	b3Assert(s_initialized == true);

	b3Fixture* fixtureA = contact->m_fixtureA;
	b3Fixture* fixtureB = contact->m_fixtureB;

	if (contact->m_manifold.pointCount > 0 &&
		fixtureA->IsSensor() == false &&
		fixtureB->IsSensor() == false)
	{
		fixtureA->GetBody()->SetAwake(true);
		fixtureB->GetBody()->SetAwake(true);
	}

	b3Shape::Type typeA = fixtureA->GetType();
	b3Shape::Type typeB = fixtureB->GetType();

	b3Assert(0 <= typeA && typeB < b3Shape::e_typeCount);
	b3Assert(0 <= typeA && typeB < b3Shape::e_typeCount);

	b3ContactDestroyFcn* destroyFcn = s_registers[typeA][typeB].destroyFcn;
	destroyFcn(contact, allocator);
}

b3Contact::b3Contact(b3Fixture* fA, int32 indexA, b3Fixture* fB, int32 indexB)
{
	m_flags = e_enabledFlag;

	m_fixtureA = fA;
	m_fixtureB = fB;

	m_indexA = indexA;
	m_indexB = indexB;

	m_manifold.pointCount = 0;

	m_prev = nullptr;
	m_next = nullptr;

	m_nodeA.contact = nullptr;
	m_nodeA.prev = nullptr;
	m_nodeA.next = nullptr;
	m_nodeA.other = nullptr;

	m_nodeB.contact = nullptr;
	m_nodeB.prev = nullptr;
	m_nodeB.next = nullptr;
	m_nodeB.other = nullptr;

	m_toiCount = 0;

	m_friction = b3MixFriction(m_fixtureA->m_friction, m_fixtureB->m_friction);
	m_restitution = b3MixRestitution(m_fixtureA->m_restitution, m_fixtureB->m_restitution);

	m_tangentSpeed = 0.0f;
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
void b3Contact::Update(b3ContactListener* listener)
{
	b3Manifold oldManifold = m_manifold;

	// Re-enable this contact.
	m_flags |= e_enabledFlag;

	bool touching = false;
	bool wasTouching = (m_flags & e_touchingFlag) == e_touchingFlag;

	bool sensorA = m_fixtureA->IsSensor();
	bool sensorB = m_fixtureB->IsSensor();
	bool sensor = sensorA || sensorB;

	b3Body* bodyA = m_fixtureA->GetBody();
	b3Body* bodyB = m_fixtureB->GetBody();
	const b3Transform& xfA = bodyA->GetTransform();
	const b3Transform& xfB = bodyB->GetTransform();

	// Is this contact a sensor?
	if (sensor)
	{
		const b3Shape* shapeA = m_fixtureA->GetShape();
		const b3Shape* shapeB = m_fixtureB->GetShape();
		touching = b3TestOverlap(shapeA, m_indexA, shapeB, m_indexB, xfA, xfB);

		// Sensors don't generate manifolds.
		m_manifold.pointCount = 0;
	}
	else
	{
		Evaluate(&m_manifold, xfA, xfB);
		touching = m_manifold.pointCount > 0;

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for (int32 i = 0; i < m_manifold.pointCount; ++i)
		{
			b3ManifoldPoint* mp2 = m_manifold.points + i;
			mp2->normalImpulse = 0.0f;
			mp2->tangentImpulse = 0.0f;
			b3ContactID id2 = mp2->id;

			for (int32 j = 0; j < oldManifold.pointCount; ++j)
			{
				b3ManifoldPoint* mp1 = oldManifold.points + j;

				if (mp1->id.key == id2.key)
				{
					mp2->normalImpulse = mp1->normalImpulse;
					mp2->tangentImpulse = mp1->tangentImpulse;
					break;
				}
			}
		}

		if (touching != wasTouching)
		{
			bodyA->SetAwake(true);
			bodyB->SetAwake(true);
		}
	}

	if (touching)
	{
		m_flags |= e_touchingFlag;
	}
	else
	{
		m_flags &= ~e_touchingFlag;
	}

	if (wasTouching == false && touching == true && listener)
	{
		listener->BeginContact(this);
	}

	if (wasTouching == true && touching == false && listener)
	{
		listener->EndContact(this);
	}

	if (sensor == false && touching && listener)
	{
		listener->PreSolve(this, &oldManifold);
	}
}

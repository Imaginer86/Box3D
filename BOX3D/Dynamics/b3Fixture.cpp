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

#include "Box3D/Dynamics/b3Fixture.h"
#include "Box3D/Dynamics/Contacts/b3Contact.h"
#include "Box3D/Dynamics/b3World.h"
//#include "Box3D/Collision/Shapes/b3CircleShape.h"
//#include "Box3D/Collision/Shapes/b3EdgeShape.h"
//#include "Box3D/Collision/Shapes/b3PolygonShape.h"
//#include "Box3D/Collision/Shapes/b3ChainShape.h"
#include "Box3D/Collision/b3BroadPhase.h"
#include "Box3D/Collision/b3Collision.h"
#include "Box3D/Common/b3BlockAllocator.h"

b3Fixture::b3Fixture()
{
	m_userData = nullptr;
	m_body = nullptr;
	m_next = nullptr;
	m_proxies = nullptr;
	m_proxyCount = 0;
	m_shape = nullptr;
	m_density = 0.0f;
}

void b3Fixture::Create(b3BlockAllocator* allocator, b3Body* body, const b3FixtureDef* def)
{
	m_userData = def->userData;
	m_friction = def->friction;
	m_restitution = def->restitution;

	m_body = body;
	m_next = nullptr;

	m_filter = def->filter;

	m_isSensor = def->isSensor;

	m_shape = def->shape->Clone(allocator);

	// Reserve proxy space
	int32 childCount = m_shape->GetChildCount();
	m_proxies = (b3FixtureProxy*)allocator->Allocate(childCount * sizeof(b3FixtureProxy));
	for (int32 i = 0; i < childCount; ++i)
	{
		m_proxies[i].fixture = nullptr;
		m_proxies[i].proxyId = b3BroadPhase::e_nullProxy;
	}
	m_proxyCount = 0;

	m_density = def->density;
}

void b3Fixture::Destroy(b3BlockAllocator* allocator)
{
	// The proxies must be destroyed before calling this.
	b3Assert(m_proxyCount == 0);

	// Free the proxy array.
	int32 childCount = m_shape->GetChildCount();
	allocator->Free(m_proxies, childCount * sizeof(b3FixtureProxy));
	m_proxies = nullptr;

	// Free the child shape.
	switch (m_shape->m_type)
	{
		//!!!
		/*
	case b3Shape::e_circle:
		{
			b3CircleShape* s = (b3CircleShape*)m_shape;
			s->~b3CircleShape();
			allocator->Free(s, sizeof(b3CircleShape));
		}
		break;

	case b3Shape::e_edge:
		{
			b3EdgeShape* s = (b3EdgeShape*)m_shape;
			s->~b3EdgeShape();
			allocator->Free(s, sizeof(b3EdgeShape));
		}
		break;

	case b3Shape::e_polygon:
		{
			b3PolygonShape* s = (b3PolygonShape*)m_shape;
			s->~b3PolygonShape();
			allocator->Free(s, sizeof(b3PolygonShape));
		}
		break;

	case b3Shape::e_chain:
		{
			b3ChainShape* s = (b3ChainShape*)m_shape;
			s->~b3ChainShape();
			allocator->Free(s, sizeof(b3ChainShape));
		}
		break;
		*/

	default:
		b3Assert(false);
		break;
	}

	m_shape = nullptr;
}

void b3Fixture::CreateProxies(b3BroadPhase* broadPhase, const b3Transform& xf)
{
	b3Assert(m_proxyCount == 0);

	// Create proxies in the broad-phase.
	m_proxyCount = m_shape->GetChildCount();

	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		b3FixtureProxy* proxy = m_proxies + i;
		m_shape->ComputeAABB(&proxy->aabb, xf, i);
		proxy->proxyId = broadPhase->CreateProxy(proxy->aabb, proxy);
		proxy->fixture = this;
		proxy->childIndex = i;
	}
}

void b3Fixture::DestroyProxies(b3BroadPhase* broadPhase)
{
	// Destroy proxies in the broad-phase.
	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		b3FixtureProxy* proxy = m_proxies + i;
		broadPhase->DestroyProxy(proxy->proxyId);
		proxy->proxyId = b3BroadPhase::e_nullProxy;
	}

	m_proxyCount = 0;
}

void b3Fixture::Synchronize(b3BroadPhase* broadPhase, const b3Transform& transform1, const b3Transform& transform2)
{
	if (m_proxyCount == 0)
	{	
		return;
	}

	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		b3FixtureProxy* proxy = m_proxies + i;

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		b3AABB aabb1, aabb3;
		m_shape->ComputeAABB(&aabb1, transform1, proxy->childIndex);
		m_shape->ComputeAABB(&aabb3, transform2, proxy->childIndex);
	
		proxy->aabb.Combine(aabb1, aabb3);

		b3Vec2 displacement = transform2.p - transform1.p;

		broadPhase->MoveProxy(proxy->proxyId, proxy->aabb, displacement);
	}
}

void b3Fixture::SetFilterData(const b3Filter& filter)
{
	m_filter = filter;

	Refilter();
}

void b3Fixture::Refilter()
{
	if (m_body == nullptr)
	{
		return;
	}

	// Flag associated contacts for filtering.
	b3ContactEdge* edge = m_body->GetContactList();
	while (edge)
	{
		b3Contact* contact = edge->contact;
		b3Fixture* fixtureA = contact->GetFixtureA();
		b3Fixture* fixtureB = contact->GetFixtureB();
		if (fixtureA == this || fixtureB == this)
		{
			contact->FlagForFiltering();
		}

		edge = edge->next;
	}

	b3World* world = m_body->GetWorld();

	if (world == nullptr)
	{
		return;
	}

	// Touch each proxy so that new pairs may be created
	b3BroadPhase* broadPhase = &world->m_contactManager.m_broadPhase;
	for (int32 i = 0; i < m_proxyCount; ++i)
	{
		broadPhase->TouchProxy(m_proxies[i].proxyId);
	}
}

void b3Fixture::SetSensor(bool sensor)
{
	if (sensor != m_isSensor)
	{
		m_body->SetAwake(true);
		m_isSensor = sensor;
	}
}

void b3Fixture::Dump(int32 bodyIndex)
{
	b3Log("    b3FixtureDef fd;\n");
	b3Log("    fd.friction = %.15lef;\n", m_friction);
	b3Log("    fd.restitution = %.15lef;\n", m_restitution);
	b3Log("    fd.density = %.15lef;\n", m_density);
	b3Log("    fd.isSensor = bool(%d);\n", m_isSensor);
	b3Log("    fd.filter.categoryBits = uint16(%d);\n", m_filter.categoryBits);
	b3Log("    fd.filter.maskBits = uint16(%d);\n", m_filter.maskBits);
	b3Log("    fd.filter.groupIndex = int16(%d);\n", m_filter.groupIndex);

	switch (m_shape->m_type)
	{
		//!!!
		/*
	case b3Shape::e_circle:
		{
			b3CircleShape* s = (b3CircleShape*)m_shape;
			b3Log("    b3CircleShape shape;\n");
			//!!!
			//b3Log("    shape.m_radius = %.15lef;\n", s->m_radius);
			//b3Log("    shape.m_p.Set(%.15lef, %.15lef);\n", s->m_p.x, s->m_p.y);
		}
		break;

	case b3Shape::e_edge:
		{
			b3EdgeShape* s = (b3EdgeShape*)m_shape;
			b3Log("    b3EdgeShape shape;\n");
			b3Log("    shape.m_radius = %.15lef;\n", s->m_radius);
			b3Log("    shape.m_vertex0.Set(%.15lef, %.15lef);\n", s->m_vertex0.x, s->m_vertex0.y);
			b3Log("    shape.m_vertex1.Set(%.15lef, %.15lef);\n", s->m_vertex1.x, s->m_vertex1.y);
			b3Log("    shape.m_vertex2.Set(%.15lef, %.15lef);\n", s->m_vertex2.x, s->m_vertex2.y);
			b3Log("    shape.m_vertex3.Set(%.15lef, %.15lef);\n", s->m_vertex3.x, s->m_vertex3.y);
			b3Log("    shape.m_hasVertex0 = bool(%d);\n", s->m_hasVertex0);
			b3Log("    shape.m_hasVertex3 = bool(%d);\n", s->m_hasVertex3);
		}
		break;

	case b3Shape::e_polygon:
		{
			b3PolygonShape* s = (b3PolygonShape*)m_shape;
			b3Log("    b3PolygonShape shape;\n");
			b3Log("    b3Vec2 vs[%d];\n", b3_maxPolygonVertices);
			for (int32 i = 0; i < s->m_count; ++i)
			{
				b3Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
			}
			b3Log("    shape.Set(vs, %d);\n", s->m_count);
		}
		break;

	case b3Shape::e_chain:
		{
			b3ChainShape* s = (b3ChainShape*)m_shape;
			b3Log("    b3ChainShape shape;\n");
			b3Log("    b3Vec2 vs[%d];\n", s->m_count);
			for (int32 i = 0; i < s->m_count; ++i)
			{
				b3Log("    vs[%d].Set(%.15lef, %.15lef);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
			}
			b3Log("    shape.CreateChain(vs, %d);\n", s->m_count);
			b3Log("    shape.m_prevVertex.Set(%.15lef, %.15lef);\n", s->m_prevVertex.x, s->m_prevVertex.y);
			b3Log("    shape.m_nextVertex.Set(%.15lef, %.15lef);\n", s->m_nextVertex.x, s->m_nextVertex.y);
			b3Log("    shape.m_hasPrevVertex = bool(%d);\n", s->m_hasPrevVertex);
			b3Log("    shape.m_hasNextVertex = bool(%d);\n", s->m_hasNextVertex);
		}
		break;
		*/

	default:
		return;
	}

	b3Log("\n");
	b3Log("    fd.shape = &shape;\n");
	b3Log("\n");
	b3Log("    bodies[%d]->CreateFixture(&fd);\n", bodyIndex);
}

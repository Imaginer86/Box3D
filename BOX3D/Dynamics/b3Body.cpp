/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include "Box3D/Dynamics/b3Body.h"
#include "Box3D/Dynamics/b3Fixture.h"
#include "Box3D/Dynamics/b3World.h"
#include "Box3D/Dynamics/Contacts/b3Contact.h"
#include "Box3D/Dynamics/Joints/b3Joint.h"

b3Body::b3Body(const b3BodyDef* bd, b3World* world)
{
	b3Assert(bd->position.IsValid());
	b3Assert(bd->linearVelocity.IsValid());
	b3Assert(b3IsValid(bd->angle));
	b3Assert(b3IsValid(bd->angularVelocity));
	b3Assert(b3IsValid(bd->angularDamping) && bd->angularDamping >= 0.0f);
	b3Assert(b3IsValid(bd->linearDamping) && bd->linearDamping >= 0.0f);

	m_flags = 0;

	if (bd->bullet)
	{
		m_flags |= e_bulletFlag;
	}
	if (bd->fixedRotation)
	{
		m_flags |= e_fixedRotationFlag;
	}
	if (bd->allowSleep)
	{
		m_flags |= e_autoSleepFlag;
	}
	if (bd->awake)
	{
		m_flags |= e_awakeFlag;
	}
	if (bd->active)
	{
		m_flags |= e_activeFlag;
	}

	m_world = world;

	m_xf.p = bd->position;
	m_xf.q.Set(bd->angle);

	m_sweep.localCenter.SetZero();
	m_sweep.c0 = m_xf.p;
	m_sweep.c = m_xf.p;
	m_sweep.a0 = bd->angle;
	m_sweep.a = bd->angle;
	m_sweep.alpha0 = 0.0f;

	m_jointList = nullptr;
	m_contactList = nullptr;
	m_prev = nullptr;
	m_next = nullptr;

	m_linearVelocity = bd->linearVelocity;
	m_angularVelocity = bd->angularVelocity;

	m_linearDamping = bd->linearDamping;
	m_angularDamping = bd->angularDamping;
	m_gravityScale = bd->gravityScale;

	m_force.SetZero();
	m_torque = 0.0f;

	m_sleepTime = 0.0f;

	m_type = bd->type;

	if (m_type == b3_dynamicBody)
	{
		m_mass = 1.0f;
		m_invMass = 1.0f;
	}
	else
	{
		m_mass = 0.0f;
		m_invMass = 0.0f;
	}

	m_I = 0.0f;
	m_invI = 0.0f;

	m_userData = bd->userData;

	m_fixtureList = nullptr;
	m_fixtureCount = 0;
}

b3Body::~b3Body()
{
	// shapes and joints are destroyed in b3World::Destroy
}

void b3Body::SetType(b3BodyType type)
{
	b3Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	if (m_type == type)
	{
		return;
	}

	m_type = type;

	ResetMassData();

	if (m_type == b3_staticBody)
	{
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0f;
		m_sweep.a0 = m_sweep.a;
		m_sweep.c0 = m_sweep.c;
		SynchronizeFixtures();
	}

	SetAwake(true);

	m_force.SetZero();
	m_torque = 0.0f;

	// Delete the attached contacts.
	b3ContactEdge* ce = m_contactList;
	while (ce)
	{
		b3ContactEdge* ce0 = ce;
		ce = ce->next;
		m_world->m_contactManager.Destroy(ce0->contact);
	}
	m_contactList = nullptr;

	// Touch the proxies so that new contacts will be created (when appropriate)
	b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		int32 proxyCount = f->m_proxyCount;
		for (int32 i = 0; i < proxyCount; ++i)
		{
			broadPhase->TouchProxy(f->m_proxies[i].proxyId);
		}
	}
}

b3Fixture* b3Body::CreateFixture(const b3FixtureDef* def)
{
	b3Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return nullptr;
	}

	b3BlockAllocator* allocator = &m_world->m_blockAllocator;

	void* memory = allocator->Allocate(sizeof(b3Fixture));
	b3Fixture* fixture = new (memory) b3Fixture;
	fixture->Create(allocator, this, def);

	if (m_flags & e_activeFlag)
	{
		b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		fixture->CreateProxies(broadPhase, m_xf);
	}

	fixture->m_next = m_fixtureList;
	m_fixtureList = fixture;
	++m_fixtureCount;

	fixture->m_body = this;

	// Adjust mass properties if needed.
	if (fixture->m_density > 0.0f)
	{
		ResetMassData();
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	m_world->m_flags |= b3World::e_newFixture;

	return fixture;
}

b3Fixture* b3Body::CreateFixture(const b3Shape* shape, float32 density)
{
	b3FixtureDef def;
	def.shape = shape;
	def.density = density;

	return CreateFixture(&def);
}

void b3Body::DestroyFixture(b3Fixture* fixture)
{
	if (fixture == NULL)
	{
		return;
	}

	b3Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	b3Assert(fixture->m_body == this);

	// Remove the fixture from this body's singly linked list.
	b3Assert(m_fixtureCount > 0);
	b3Fixture** node = &m_fixtureList;
	bool found = false;
	while (*node != nullptr)
	{
		if (*node == fixture)
		{
			*node = fixture->m_next;
			found = true;
			break;
		}

		node = &(*node)->m_next;
	}

	// You tried to remove a shape that is not attached to this body.
	b3Assert(found);

	// Destroy any contacts associated with the fixture.
	b3ContactEdge* edge = m_contactList;
	while (edge)
	{
		b3Contact* c = edge->contact;
		edge = edge->next;

		b3Fixture* fixtureA = c->GetFixtureA();
		b3Fixture* fixtureB = c->GetFixtureB();

		if (fixture == fixtureA || fixture == fixtureB)
		{
			// This destroys the contact and removes it from
			// this body's contact list.
			m_world->m_contactManager.Destroy(c);
		}
	}

	b3BlockAllocator* allocator = &m_world->m_blockAllocator;

	if (m_flags & e_activeFlag)
	{
		b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		fixture->DestroyProxies(broadPhase);
	}

	fixture->m_body = nullptr;
	fixture->m_next = nullptr;
	fixture->Destroy(allocator);
	fixture->~b3Fixture();
	allocator->Free(fixture, sizeof(b3Fixture));

	--m_fixtureCount;

	// Reset the mass data.
	ResetMassData();
}

void b3Body::ResetMassData()
{
	// Compute mass data from shapes. Each shape has its own density.
	m_mass = 0.0f;
	m_invMass = 0.0f;
	m_I = 0.0f;
	m_invI = 0.0f;
	m_sweep.localCenter.SetZero();

	// Static and kinematic bodies have zero mass.
	if (m_type == b3_staticBody || m_type == b3_kinematicBody)
	{
		m_sweep.c0 = m_xf.p;
		m_sweep.c = m_xf.p;
		m_sweep.a0 = m_sweep.a;
		return;
	}

	b3Assert(m_type == b3_dynamicBody);

	// Accumulate mass over all fixtures.
	b3Vec2 localCenter = b3Vec2_zero;
	for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		if (f->m_density == 0.0f)
		{
			continue;
		}

		b3MassData massData;
		f->GetMassData(&massData);
		m_mass += massData.mass;
		localCenter += massData.mass * massData.center;
		m_I += massData.I;
	}

	// Compute center of mass.
	if (m_mass > 0.0f)
	{
		m_invMass = 1.0f / m_mass;
		localCenter *= m_invMass;
	}
	else
	{
		// Force all dynamic bodies to have a positive mass.
		m_mass = 1.0f;
		m_invMass = 1.0f;
	}

	if (m_I > 0.0f && (m_flags & e_fixedRotationFlag) == 0)
	{
		// Center the inertia about the center of mass.
		m_I -= m_mass * b3Dot(localCenter, localCenter);
		b3Assert(m_I > 0.0f);
		m_invI = 1.0f / m_I;

	}
	else
	{
		m_I = 0.0f;
		m_invI = 0.0f;
	}

	// Move center of mass.
	b3Vec2 oldCenter = m_sweep.c;
	m_sweep.localCenter = localCenter;
	m_sweep.c0 = m_sweep.c = b3Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_linearVelocity += b3Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

void b3Body::SetMassData(const b3MassData* massData)
{
	b3Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	if (m_type != b3_dynamicBody)
	{
		return;
	}

	m_invMass = 0.0f;
	m_I = 0.0f;
	m_invI = 0.0f;

	m_mass = massData->mass;
	if (m_mass <= 0.0f)
	{
		m_mass = 1.0f;
	}

	m_invMass = 1.0f / m_mass;

	if (massData->I > 0.0f && (m_flags & b3Body::e_fixedRotationFlag) == 0)
	{
		m_I = massData->I - m_mass * b3Dot(massData->center, massData->center);
		b3Assert(m_I > 0.0f);
		m_invI = 1.0f / m_I;
	}

	// Move center of mass.
	b3Vec2 oldCenter = m_sweep.c;
	m_sweep.localCenter =  massData->center;
	m_sweep.c0 = m_sweep.c = b3Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_linearVelocity += b3Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

bool b3Body::ShouldCollide(const b3Body* other) const
{
	// At least one body should be dynamic.
	if (m_type != b3_dynamicBody && other->m_type != b3_dynamicBody)
	{
		return false;
	}

	// Does a joint prevent collision?
	for (b3JointEdge* jn = m_jointList; jn; jn = jn->next)
	{
		if (jn->other == other)
		{
			if (jn->joint->m_collideConnected == false)
			{
				return false;
			}
		}
	}

	return true;
}

void b3Body::SetTransform(const b3Vec2& position, float32 angle)
{
	b3Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	m_xf.q.Set(angle);
	m_xf.p = position;

	m_sweep.c = b3Mul(m_xf, m_sweep.localCenter);
	m_sweep.a = angle;

	m_sweep.c0 = m_sweep.c;
	m_sweep.a0 = angle;

	b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, m_xf, m_xf);
	}
}

void b3Body::SynchronizeFixtures()
{
	b3Transform xf1;
	xf1.q.Set(m_sweep.a0);
	xf1.p = m_sweep.c0 - b3Mul(xf1.q, m_sweep.localCenter);

	b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, xf1, m_xf);
	}
}

void b3Body::SetActive(bool flag)
{
	b3Assert(m_world->IsLocked() == false);

	if (flag == IsActive())
	{
		return;
	}

	if (flag)
	{
		m_flags |= e_activeFlag;

		// Create all proxies.
		b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->CreateProxies(broadPhase, m_xf);
		}

		// Contacts are created the next time step.
	}
	else
	{
		m_flags &= ~e_activeFlag;

		// Destroy all proxies.
		b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->DestroyProxies(broadPhase);
		}

		// Destroy the attached contacts.
		b3ContactEdge* ce = m_contactList;
		while (ce)
		{
			b3ContactEdge* ce0 = ce;
			ce = ce->next;
			m_world->m_contactManager.Destroy(ce0->contact);
		}
		m_contactList = nullptr;
	}
}

void b3Body::SetFixedRotation(bool flag)
{
	bool status = (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
	if (status == flag)
	{
		return;
	}

	if (flag)
	{
		m_flags |= e_fixedRotationFlag;
	}
	else
	{
		m_flags &= ~e_fixedRotationFlag;
	}

	m_angularVelocity = 0.0f;

	ResetMassData();
}

void b3Body::Dump()
{
	int32 bodyIndex = m_islandIndex;

	b3Log("{\n");
	b3Log("  b3BodyDef bd;\n");
	b3Log("  bd.type = b3BodyType(%d);\n", m_type);
	b3Log("  bd.position.Set(%.15lef, %.15lef);\n", m_xf.p.x, m_xf.p.y);
	b3Log("  bd.angle = %.15lef;\n", m_sweep.a);
	b3Log("  bd.linearVelocity.Set(%.15lef, %.15lef);\n", m_linearVelocity.x, m_linearVelocity.y);
	b3Log("  bd.angularVelocity = %.15lef;\n", m_angularVelocity);
	b3Log("  bd.linearDamping = %.15lef;\n", m_linearDamping);
	b3Log("  bd.angularDamping = %.15lef;\n", m_angularDamping);
	b3Log("  bd.allowSleep = bool(%d);\n", m_flags & e_autoSleepFlag);
	b3Log("  bd.awake = bool(%d);\n", m_flags & e_awakeFlag);
	b3Log("  bd.fixedRotation = bool(%d);\n", m_flags & e_fixedRotationFlag);
	b3Log("  bd.bullet = bool(%d);\n", m_flags & e_bulletFlag);
	b3Log("  bd.active = bool(%d);\n", m_flags & e_activeFlag);
	b3Log("  bd.gravityScale = %.15lef;\n", m_gravityScale);
	b3Log("  bodies[%d] = m_world->CreateBody(&bd);\n", m_islandIndex);
	b3Log("\n");
	for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		b3Log("  {\n");
		f->Dump(bodyIndex);
		b3Log("  }\n");
	}
	b3Log("}\n");
}

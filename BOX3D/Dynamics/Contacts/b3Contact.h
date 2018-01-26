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

#ifndef b3_CONTACT_H
#define b3_CONTACT_H

#include "Box3D/Common/b3Math.h"
#include "Box3D/Collision/b3Collision.h"
#include "Box3D/Collision/Shapes/b3Shape.h"
#include "Box3D/Dynamics/b3Fixture.h"

class b3Body;
class b3Contact;
class b3Fixture;
class b3World;
class b3BlockAllocator;
class b3StackAllocator;
class b3ContactListener;

/// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
/// For example, anything slides on ice.
inline float32 b3MixFriction(float32 friction1, float32 friction2)
{
	return b3Sqrt(friction1 * friction2);
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
inline float32 b3MixRestitution(float32 restitution1, float32 restitution2)
{
	return restitution1 > restitution2 ? restitution1 : restitution2;
}

typedef b3Contact* b3ContactCreateFcn(	b3Fixture* fixtureA, int32 indexA,
										b3Fixture* fixtureB, int32 indexB,
										b3BlockAllocator* allocator);
typedef void b3ContactDestroyFcn(b3Contact* contact, b3BlockAllocator* allocator);

struct b3ContactRegister
{
	b3ContactCreateFcn* createFcn;
	b3ContactDestroyFcn* destroyFcn;
	bool primary;
};

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
struct b3ContactEdge
{
	b3Body* other;			///< provides quick access to the other body attached.
	b3Contact* contact;		///< the contact
	b3ContactEdge* prev;	///< the previous contact edge in the body's contact list
	b3ContactEdge* next;	///< the next contact edge in the body's contact list
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
class b3Contact
{
public:

	/// Get the contact manifold. Do not modify the manifold unless you understand the
	/// internals of Box3D.
	b3Manifold* GetManifold();
	const b3Manifold* GetManifold() const;

	/// Get the world manifold.
	void GetWorldManifold(b3WorldManifold* worldManifold) const;

	/// Is this contact touching?
	bool IsTouching() const;

	/// Enable/disable this contact. This can be used inside the pre-solve
	/// contact listener. The contact is only disabled for the current
	/// time step (or sub-step in continuous collisions).
	void SetEnabled(bool flag);

	/// Has this contact been disabled?
	bool IsEnabled() const;

	/// Get the next contact in the world's contact list.
	b3Contact* GetNext();
	const b3Contact* GetNext() const;

	/// Get fixture A in this contact.
	b3Fixture* GetFixtureA();
	const b3Fixture* GetFixtureA() const;

	/// Get the child primitive index for fixture A.
	int32 GetChildIndexA() const;

	/// Get fixture B in this contact.
	b3Fixture* GetFixtureB();
	const b3Fixture* GetFixtureB() const;

	/// Get the child primitive index for fixture B.
	int32 GetChildIndexB() const;

	/// Override the default friction mixture. You can call this in b3ContactListener::PreSolve.
	/// This value persists until set or reset.
	void SetFriction(float32 friction);

	/// Get the friction.
	float32 GetFriction() const;

	/// Reset the friction mixture to the default value.
	void ResetFriction();

	/// Override the default restitution mixture. You can call this in b3ContactListener::PreSolve.
	/// The value persists until you set or reset.
	void SetRestitution(float32 restitution);

	/// Get the restitution.
	float32 GetRestitution() const;

	/// Reset the restitution to the default value.
	void ResetRestitution();

	/// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
	void SetTangentSpeed(float32 speed);

	/// Get the desired tangent speed. In meters per second.
	float32 GetTangentSpeed() const;

	/// Evaluate this contact with your own manifold and transforms.
	virtual void Evaluate(b3Manifold* manifold, const b3Transform& xfA, const b3Transform& xfB) = 0;

protected:
	friend class b3ContactManager;
	friend class b3World;
	friend class b3ContactSolver;
	friend class b3Body;
	friend class b3Fixture;

	// Flags stored in m_flags
	enum
	{
		// Used when crawling contact graph when forming islands.
		e_islandFlag		= 0x0001,

		// Set when the shapes are touching.
		e_touchingFlag		= 0x0002,

		// This contact can be disabled (by user)
		e_enabledFlag		= 0x0004,

		// This contact needs filtering because a fixture filter was changed.
		e_filterFlag		= 0x0008,

		// This bullet contact had a TOI event
		e_bulletHitFlag		= 0x0010,

		// This contact has a valid TOI in m_toi
		e_toiFlag			= 0x0020
	};

	/// Flag this contact for filtering. Filtering will occur the next time step.
	void FlagForFiltering();

	static void AddType(b3ContactCreateFcn* createFcn, b3ContactDestroyFcn* destroyFcn,
						b3Shape::Type typeA, b3Shape::Type typeB);
	static void InitializeRegisters();
	static b3Contact* Create(b3Fixture* fixtureA, int32 indexA, b3Fixture* fixtureB, int32 indexB, b3BlockAllocator* allocator);
	static void Destroy(b3Contact* contact, b3Shape::Type typeA, b3Shape::Type typeB, b3BlockAllocator* allocator);
	static void Destroy(b3Contact* contact, b3BlockAllocator* allocator);

	b3Contact() : m_fixtureA(nullptr), m_fixtureB(nullptr) {}
	b3Contact(b3Fixture* fixtureA, int32 indexA, b3Fixture* fixtureB, int32 indexB);
	virtual ~b3Contact() {}

	void Update(b3ContactListener* listener);

	static b3ContactRegister s_registers[b3Shape::e_typeCount][b3Shape::e_typeCount];
	static bool s_initialized;

	uint32 m_flags;

	// World pool and list pointers.
	b3Contact* m_prev;
	b3Contact* m_next;

	// Nodes for connecting bodies.
	b3ContactEdge m_nodeA;
	b3ContactEdge m_nodeB;

	b3Fixture* m_fixtureA;
	b3Fixture* m_fixtureB;

	int32 m_indexA;
	int32 m_indexB;

	b3Manifold m_manifold;

	int32 m_toiCount;
	float32 m_toi;

	float32 m_friction;
	float32 m_restitution;

	float32 m_tangentSpeed;
};

inline b3Manifold* b3Contact::GetManifold()
{
	return &m_manifold;
}

inline const b3Manifold* b3Contact::GetManifold() const
{
	return &m_manifold;
}

inline void b3Contact::GetWorldManifold(b3WorldManifold* worldManifold) const
{
	const b3Body* bodyA = m_fixtureA->GetBody();
	const b3Body* bodyB = m_fixtureB->GetBody();
	const b3Shape* shapeA = m_fixtureA->GetShape();
	const b3Shape* shapeB = m_fixtureB->GetShape();

	worldManifold->Initialize(&m_manifold, bodyA->GetTransform(), shapeA->m_radius, bodyB->GetTransform(), shapeB->m_radius);
}

inline void b3Contact::SetEnabled(bool flag)
{
	if (flag)
	{
		m_flags |= e_enabledFlag;
	}
	else
	{
		m_flags &= ~e_enabledFlag;
	}
}

inline bool b3Contact::IsEnabled() const
{
	return (m_flags & e_enabledFlag) == e_enabledFlag;
}

inline bool b3Contact::IsTouching() const
{
	return (m_flags & e_touchingFlag) == e_touchingFlag;
}

inline b3Contact* b3Contact::GetNext()
{
	return m_next;
}

inline const b3Contact* b3Contact::GetNext() const
{
	return m_next;
}

inline b3Fixture* b3Contact::GetFixtureA()
{
	return m_fixtureA;
}

inline const b3Fixture* b3Contact::GetFixtureA() const
{
	return m_fixtureA;
}

inline b3Fixture* b3Contact::GetFixtureB()
{
	return m_fixtureB;
}

inline int32 b3Contact::GetChildIndexA() const
{
	return m_indexA;
}

inline const b3Fixture* b3Contact::GetFixtureB() const
{
	return m_fixtureB;
}

inline int32 b3Contact::GetChildIndexB() const
{
	return m_indexB;
}

inline void b3Contact::FlagForFiltering()
{
	m_flags |= e_filterFlag;
}

inline void b3Contact::SetFriction(float32 friction)
{
	m_friction = friction;
}

inline float32 b3Contact::GetFriction() const
{
	return m_friction;
}

inline void b3Contact::ResetFriction()
{
	m_friction = b3MixFriction(m_fixtureA->m_friction, m_fixtureB->m_friction);
}

inline void b3Contact::SetRestitution(float32 restitution)
{
	m_restitution = restitution;
}

inline float32 b3Contact::GetRestitution() const
{
	return m_restitution;
}

inline void b3Contact::ResetRestitution()
{
	m_restitution = b3MixRestitution(m_fixtureA->m_restitution, m_fixtureB->m_restitution);
}

inline void b3Contact::SetTangentSpeed(float32 speed)
{
	m_tangentSpeed = speed;
}

inline float32 b3Contact::GetTangentSpeed() const
{
	return m_tangentSpeed;
}

#endif

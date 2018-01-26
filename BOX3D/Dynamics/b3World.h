/*
* Copyright (c) 2006-2011 Erin Catto http://www.Box3D.org
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

#ifndef b3_WORLD_H
#define b3_WORLD_H

#include "Box3D/Common/b3Math.h"
#include "Box3D/Common/b3BlockAllocator.h"
#include "Box3D/Common/b3StackAllocator.h"
#include "Box3D/Dynamics/b3ContactManager.h"
#include "Box3D/Dynamics/b3WorldCallbacks.h"
#include "Box3D/Dynamics/b3TimeStep.h"

struct b3AABB;
struct b3BodyDef;
struct b3Color;
struct b3JointDef;
class b3Body;
class b3Draw;
class b3Fixture;
class b3Joint;

/// The world class manages all physics entities, dynamic simulation,
/// and asynchronous queries. The world also contains efficient memory
/// management facilities.
class b3World
{
public:
	/// Construct a world object.
	/// @param gravity the world gravity vector.
	b3World(const b3Vec2& gravity);

	/// Destruct the world. All physics entities are destroyed and all heap memory is released.
	~b3World();

	/// Register a destruction listener. The listener is owned by you and must
	/// remain in scope.
	void SetDestructionListener(b3DestructionListener* listener);

	/// Register a contact filter to provide specific control over collision.
	/// Otherwise the default filter is used (b3_defaultFilter). The listener is
	/// owned by you and must remain in scope. 
	void SetContactFilter(b3ContactFilter* filter);

	/// Register a contact event listener. The listener is owned by you and must
	/// remain in scope.
	void SetContactListener(b3ContactListener* listener);

	/// Register a routine for debug drawing. The debug draw functions are called
	/// inside with b3World::DrawDebugData method. The debug draw object is owned
	/// by you and must remain in scope.
	void SetDebugDraw(b3Draw* debugDraw);

	/// Create a rigid body given a definition. No reference to the definition
	/// is retained.
	/// @warning This function is locked during callbacks.
	b3Body* CreateBody(const b3BodyDef* def);

	/// Destroy a rigid body given a definition. No reference to the definition
	/// is retained. This function is locked during callbacks.
	/// @warning This automatically deletes all associated shapes and joints.
	/// @warning This function is locked during callbacks.
	void DestroyBody(b3Body* body);

	/// Create a joint to constrain bodies together. No reference to the definition
	/// is retained. This may cause the connected bodies to cease colliding.
	/// @warning This function is locked during callbacks.
	b3Joint* CreateJoint(const b3JointDef* def);

	/// Destroy a joint. This may cause the connected bodies to begin colliding.
	/// @warning This function is locked during callbacks.
	void DestroyJoint(b3Joint* joint);

	/// Take a time step. This performs collision detection, integration,
	/// and constraint solution.
	/// @param timeStep the amount of time to simulate, this should not vary.
	/// @param velocityIterations for the velocity constraint solver.
	/// @param positionIterations for the position constraint solver.
	void Step(	float32 timeStep,
				int32 velocityIterations,
				int32 positionIterations);

	/// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
	/// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
	/// The purpose of this function is to support sub-stepping. Sub-stepping is often used to maintain
	/// a fixed sized time step under a variable frame-rate.
	/// When you perform sub-stepping you will disable auto clearing of forces and instead call
	/// ClearForces after all sub-steps are complete in one pass of your game loop.
	/// @see SetAutoClearForces
	void ClearForces();

	/// Call this to draw shapes and other debug draw data. This is intentionally non-const.
	void DrawDebugData();

	/// Query the world for all fixtures that potentially overlap the
	/// provided AABB.
	/// @param callback a user implemented callback class.
	/// @param aabb the query box.
	void QueryAABB(b3QueryCallback* callback, const b3AABB& aabb) const;

	/// Ray-cast the world for all fixtures in the path of the ray. Your callback
	/// controls whether you get the closest point, any point, or n-points.
	/// The ray-cast ignores shapes that contain the starting point.
	/// @param callback a user implemented callback class.
	/// @param point1 the ray starting point
	/// @param point2 the ray ending point
	void RayCast(b3RayCastCallback* callback, const b3Vec2& point1, const b3Vec2& point2) const;

	/// Get the world body list. With the returned body, use b3Body::GetNext to get
	/// the next body in the world list. A nullptr body indicates the end of the list.
	/// @return the head of the world body list.
	b3Body* GetBodyList();
	const b3Body* GetBodyList() const;

	/// Get the world joint list. With the returned joint, use b3Joint::GetNext to get
	/// the next joint in the world list. A nullptr joint indicates the end of the list.
	/// @return the head of the world joint list.
	b3Joint* GetJointList();
	const b3Joint* GetJointList() const;

	/// Get the world contact list. With the returned contact, use b3Contact::GetNext to get
	/// the next contact in the world list. A nullptr contact indicates the end of the list.
	/// @return the head of the world contact list.
	/// @warning contacts are created and destroyed in the middle of a time step.
	/// Use b3ContactListener to avoid missing contacts.
	b3Contact* GetContactList();
	const b3Contact* GetContactList() const;

	/// Enable/disable sleep.
	void SetAllowSleeping(bool flag);
	bool GetAllowSleeping() const { return m_allowSleep; }

	/// Enable/disable warm starting. For testing.
	void SetWarmStarting(bool flag) { m_warmStarting = flag; }
	bool GetWarmStarting() const { return m_warmStarting; }

	/// Enable/disable continuous physics. For testing.
	void SetContinuousPhysics(bool flag) { m_continuousPhysics = flag; }
	bool GetContinuousPhysics() const { return m_continuousPhysics; }

	/// Enable/disable single stepped continuous physics. For testing.
	void SetSubStepping(bool flag) { m_subStepping = flag; }
	bool GetSubStepping() const { return m_subStepping; }

	/// Get the number of broad-phase proxies.
	int32 GetProxyCount() const;

	/// Get the number of bodies.
	int32 GetBodyCount() const;

	/// Get the number of joints.
	int32 GetJointCount() const;

	/// Get the number of contacts (each may have 0 or more contact points).
	int32 GetContactCount() const;

	/// Get the height of the dynamic tree.
	int32 GetTreeHeight() const;

	/// Get the balance of the dynamic tree.
	int32 GetTreeBalance() const;

	/// Get the quality metric of the dynamic tree. The smaller the better.
	/// The minimum is 1.
	float32 GetTreeQuality() const;

	/// Change the global gravity vector.
	void SetGravity(const b3Vec2& gravity);
	
	/// Get the global gravity vector.
	b3Vec2 GetGravity() const;

	/// Is the world locked (in the middle of a time step).
	bool IsLocked() const;

	/// Set flag to control automatic clearing of forces after each time step.
	void SetAutoClearForces(bool flag);

	/// Get the flag that controls automatic clearing of forces after each time step.
	bool GetAutoClearForces() const;

	/// Shift the world origin. Useful for large worlds.
	/// The body shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const b3Vec2& newOrigin);

	/// Get the contact manager for testing.
	const b3ContactManager& GetContactManager() const;

	/// Get the current profile.
	const b3Profile& GetProfile() const;

	/// Dump the world into the log file.
	/// @warning this should be called outside of a time step.
	void Dump();

private:

	// m_flags
	enum
	{
		e_newFixture	= 0x0001,
		e_locked		= 0x0002,
		e_clearForces	= 0x0004
	};

	friend class b3Body;
	friend class b3Fixture;
	friend class b3ContactManager;
	friend class b3Controller;

	void Solve(const b3TimeStep& step);
	void SolveTOI(const b3TimeStep& step);

	void DrawJoint(b3Joint* joint);
	void DrawShape(b3Fixture* shape, const b3Transform& xf, const b3Color& color);

	b3BlockAllocator m_blockAllocator;
	b3StackAllocator m_stackAllocator;

	int32 m_flags;

	b3ContactManager m_contactManager;

	b3Body* m_bodyList;
	b3Joint* m_jointList;

	int32 m_bodyCount;
	int32 m_jointCount;

	b3Vec2 m_gravity;
	bool m_allowSleep;

	b3DestructionListener* m_destructionListener;
	b3Draw* g_debugDraw;

	// This is used to compute the time step ratio to
	// support a variable time step.
	float32 m_inv_dt0;

	// These are for debugging the solver.
	bool m_warmStarting;
	bool m_continuousPhysics;
	bool m_subStepping;

	bool m_stepComplete;

	b3Profile m_profile;
};

inline b3Body* b3World::GetBodyList()
{
	return m_bodyList;
}

inline const b3Body* b3World::GetBodyList() const
{
	return m_bodyList;
}

inline b3Joint* b3World::GetJointList()
{
	return m_jointList;
}

inline const b3Joint* b3World::GetJointList() const
{
	return m_jointList;
}

inline b3Contact* b3World::GetContactList()
{
	return m_contactManager.m_contactList;
}

inline const b3Contact* b3World::GetContactList() const
{
	return m_contactManager.m_contactList;
}

inline int32 b3World::GetBodyCount() const
{
	return m_bodyCount;
}

inline int32 b3World::GetJointCount() const
{
	return m_jointCount;
}

inline int32 b3World::GetContactCount() const
{
	return m_contactManager.m_contactCount;
}

inline void b3World::SetGravity(const b3Vec2& gravity)
{
	m_gravity = gravity;
}

inline b3Vec2 b3World::GetGravity() const
{
	return m_gravity;
}

inline bool b3World::IsLocked() const
{
	return (m_flags & e_locked) == e_locked;
}

inline void b3World::SetAutoClearForces(bool flag)
{
	if (flag)
	{
		m_flags |= e_clearForces;
	}
	else
	{
		m_flags &= ~e_clearForces;
	}
}

/// Get the flag that controls automatic clearing of forces after each time step.
inline bool b3World::GetAutoClearForces() const
{
	return (m_flags & e_clearForces) == e_clearForces;
}

inline const b3ContactManager& b3World::GetContactManager() const
{
	return m_contactManager;
}

inline const b3Profile& b3World::GetProfile() const
{
	return m_profile;
}

#endif

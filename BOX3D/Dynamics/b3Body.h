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

#ifndef b3_BODY_H
#define b3_BODY_H

#include "Box3D/Common/b3Math.h"
#include "Box3D/Collision/Shapes/b3Shape.h"
#include <memory>

class b3Fixture;
class b3Joint;
class b3Contact;
class b3Controller;
class b3World;
struct b3FixtureDef;
struct b3JointEdge;
struct b3ContactEdge;

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
enum b3BodyType
{
	b3_staticBody = 0,
	b3_kinematicBody,
	b3_dynamicBody

	// TODO_ERIN
	//b3_bulletBody,
};

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
struct b3BodyDef
{
	/// This constructor sets the body definition default values.
	b3BodyDef()
	{
		userData = nullptr;
		position.Set(0.0f, 0.0f);
		angle = 0.0f;
		linearVelocity.Set(0.0f, 0.0f);
		angularVelocity = 0.0f;
		linearDamping = 0.0f;
		angularDamping = 0.0f;
		allowSleep = true;
		awake = true;
		fixedRotation = false;
		bullet = false;
		type = b3_staticBody;
		active = true;
		gravityScale = 1.0f;
	}

	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	b3BodyType type;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	b3Vec2 position;

	/// The world angle of the body in radians.
	float32 angle;

	/// The linear velocity of the body's origin in world co-ordinates.
	b3Vec2 linearVelocity;

	/// The angular velocity of the body.
	float32 angularVelocity;

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Units are 1/time
	float32 linearDamping;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	/// Units are 1/time
	float32 angularDamping;

	/// Set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	bool allowSleep;

	/// Is this body initially awake or sleeping?
	bool awake;

	/// Should this body be prevented from rotating? Useful for characters.
	bool fixedRotation;

	/// Is this a fast moving body that should be prevented from tunneling through
	/// other moving bodies? Note that all bodies are prevented from tunneling through
	/// kinematic and static bodies. This setting is only considered on dynamic bodies.
	/// @warning You should use this flag sparingly since it increases processing time.
	bool bullet;

	/// Does this body start out active?
	bool active;

	/// Use this to store application specific body data.
	void* userData;

	/// Scale the gravity applied to this body.
	float32 gravityScale;
};

/// A rigid body. These are created via b3World::CreateBody.
class b3Body
{
public:
	/// Creates a fixture and attach it to this body. Use this function if you need
	/// to set some fixture parameters, like friction. Otherwise you can create the
	/// fixture directly from a shape.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// Contacts are not created until the next time step.
	/// @param def the fixture definition.
	/// @warning This function is locked during callbacks.
	b3Fixture* CreateFixture(const b3FixtureDef* def);

	/// Creates a fixture from a shape and attach it to this body.
	/// This is a convenience function. Use b3FixtureDef if you need to set parameters
	/// like friction, restitution, user data, or filtering.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// @param shape the shape to be cloned.
	/// @param density the shape density (set to zero for static bodies).
	/// @warning This function is locked during callbacks.
	b3Fixture* CreateFixture(const b3Shape* shape, float32 density);

	/// Destroy a fixture. This removes the fixture from the broad-phase and
	/// destroys all contacts associated with this fixture. This will
	/// automatically adjust the mass of the body if the body is dynamic and the
	/// fixture has positive density.
	/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	/// @param fixture the fixture to be removed.
	/// @warning This function is locked during callbacks.
	void DestroyFixture(b3Fixture* fixture);

	/// Set the position of the body's origin and rotation.
	/// Manipulating a body's transform may cause non-physical behavior.
	/// Note: contacts are updated on the next call to b3World::Step.
	/// @param position the world position of the body's local origin.
	/// @param angle the world rotation in radians.
	void SetTransform(const b3Vec2& position, float32 angle);

	/// Get the body transform for the body's origin.
	/// @return the world transform of the body's origin.
	const b3Transform& GetTransform() const;

	/// Get the world body origin position.
	/// @return the world position of the body's origin.
	const b3Vec2& GetPosition() const;

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	float32 GetAngle() const;

	/// Get the world position of the center of mass.
	const b3Vec2& GetWorldCenter() const;

	/// Get the local position of the center of mass.
	const b3Vec2& GetLocalCenter() const;

	/// Set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	void SetLinearVelocity(const b3Vec2& v);

	/// Get the linear velocity of the center of mass.
	/// @return the linear velocity of the center of mass.
	const b3Vec2& GetLinearVelocity() const;

	/// Set the angular velocity.
	/// @param omega the new angular velocity in radians/second.
	void SetAngularVelocity(float32 omega);

	/// Get the angular velocity.
	/// @return the angular velocity in radians/second.
	float32 GetAngularVelocity() const;

	/// Apply a force at a world point. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyForce(const b3Vec2& force, const b3Vec2& point, bool wake);

	/// Apply a force to the center of mass. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param wake also wake up the body
	void ApplyForceToCenter(const b3Vec2& force, bool wake);

	/// Apply a torque. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// @param torque about the z-axis (out of the screen), usually in N-m.
	/// @param wake also wake up the body
	void ApplyTorque(float32 torque, bool wake);

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyLinearImpulse(const b3Vec2& impulse, const b3Vec2& point, bool wake);

	/// Apply an impulse to the center of mass. This immediately modifies the velocity.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param wake also wake up the body
	void ApplyLinearImpulseToCenter(const b3Vec2& impulse, bool wake);

	/// Apply an angular impulse.
	/// @param impulse the angular impulse in units of kg*m*m/s
	/// @param wake also wake up the body
	void ApplyAngularImpulse(float32 impulse, bool wake);

	/// Get the total mass of the body.
	/// @return the mass, usually in kilograms (kg).
	float32 GetMass() const;

	/// Get the rotational inertia of the body about the local origin.
	/// @return the rotational inertia, usually in kg-m^2.
	float32 GetInertia() const;

	/// Get the mass data of the body.
	/// @return a struct containing the mass, inertia and center of the body.
	void GetMassData(b3MassData* data) const;

	/// Set the mass properties to override the mass properties of the fixtures.
	/// Note that this changes the center of mass position.
	/// Note that creating or destroying fixtures can also alter the mass.
	/// This function has no effect if the body isn't dynamic.
	/// @param massData the mass properties.
	void SetMassData(const b3MassData* data);

	/// This resets the mass properties to the sum of the mass properties of the fixtures.
	/// This normally does not need to be called unless you called SetMassData to override
	/// the mass and you later want to reset the mass.
	void ResetMassData();

	/// Get the world coordinates of a point given the local coordinates.
	/// @param localPoint a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.
	b3Vec2 GetWorldPoint(const b3Vec2& localPoint) const;

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	b3Vec2 GetWorldVector(const b3Vec2& localVector) const;

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	b3Vec2 GetLocalPoint(const b3Vec2& worldPoint) const;

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.
	b3Vec2 GetLocalVector(const b3Vec2& worldVector) const;

	/// Get the world linear velocity of a world point attached to this body.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	b3Vec2 GetLinearVelocityFromWorldPoint(const b3Vec2& worldPoint) const;

	/// Get the world velocity of a local point.
	/// @param a point in local coordinates.
	/// @return the world velocity of a point.
	b3Vec2 GetLinearVelocityFromLocalPoint(const b3Vec2& localPoint) const;

	/// Get the linear damping of the body.
	float32 GetLinearDamping() const;

	/// Set the linear damping of the body.
	void SetLinearDamping(float32 linearDamping);

	/// Get the angular damping of the body.
	float32 GetAngularDamping() const;

	/// Set the angular damping of the body.
	void SetAngularDamping(float32 angularDamping);

	/// Get the gravity scale of the body.
	float32 GetGravityScale() const;

	/// Set the gravity scale of the body.
	void SetGravityScale(float32 scale);

	/// Set the type of this body. This may alter the mass and velocity.
	void SetType(b3BodyType type);

	/// Get the type of this body.
	b3BodyType GetType() const;

	/// Should this body be treated like a bullet for continuous collision detection?
	void SetBullet(bool flag);

	/// Is this body treated like a bullet for continuous collision detection?
	bool IsBullet() const;

	/// You can disable sleeping on this body. If you disable sleeping, the
	/// body will be woken.
	void SetSleepingAllowed(bool flag);

	/// Is this body allowed to sleep
	bool IsSleepingAllowed() const;

	/// Set the sleep state of the body. A sleeping body has very
	/// low CPU cost.
	/// @param flag set to true to wake the body, false to put it to sleep.
	void SetAwake(bool flag);

	/// Get the sleeping state of this body.
	/// @return true if the body is awake.
	bool IsAwake() const;

	/// Set the active state of the body. An inactive body is not
	/// simulated and cannot be collided with or woken up.
	/// If you pass a flag of true, all fixtures will be added to the
	/// broad-phase.
	/// If you pass a flag of false, all fixtures will be removed from
	/// the broad-phase and all contacts will be destroyed.
	/// Fixtures and joints are otherwise unaffected. You may continue
	/// to create/destroy fixtures and joints on inactive bodies.
	/// Fixtures on an inactive body are implicitly inactive and will
	/// not participate in collisions, ray-casts, or queries.
	/// Joints connected to an inactive body are implicitly inactive.
	/// An inactive body is still owned by a b3World object and remains
	/// in the body list.
	void SetActive(bool flag);

	/// Get the active state of the body.
	bool IsActive() const;

	/// Set this body to have fixed rotation. This causes the mass
	/// to be reset.
	void SetFixedRotation(bool flag);

	/// Does this body have fixed rotation?
	bool IsFixedRotation() const;

	/// Get the list of all fixtures attached to this body.
	b3Fixture* GetFixtureList();
	const b3Fixture* GetFixtureList() const;

	/// Get the list of all joints attached to this body.
	b3JointEdge* GetJointList();
	const b3JointEdge* GetJointList() const;

	/// Get the list of all contacts attached to this body.
	/// @warning this list changes during the time step and you may
	/// miss some collisions if you don't use b3ContactListener.
	b3ContactEdge* GetContactList();
	const b3ContactEdge* GetContactList() const;

	/// Get the next body in the world's body list.
	b3Body* GetNext();
	const b3Body* GetNext() const;

	/// Get the user data pointer that was provided in the body definition.
	void* GetUserData() const;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data);

	/// Get the parent world of this body.
	b3World* GetWorld();
	const b3World* GetWorld() const;

	/// Dump this body to a log file
	void Dump();

private:

	friend class b3World;
	friend class b3Island;
	friend class b3ContactManager;
	friend class b3ContactSolver;
	friend class b3Contact;
	
	friend class b3DistanceJoint;
	friend class b3FrictionJoint;
	friend class b3GearJoint;
	friend class b3MotorJoint;
	friend class b3MouseJoint;
	friend class b3PrismaticJoint;
	friend class b3PulleyJoint;
	friend class b3RevoluteJoint;
	friend class b3RopeJoint;
	friend class b3WeldJoint;
	friend class b3WheelJoint;

	// m_flags
	enum
	{
		e_islandFlag		= 0x0001,
		e_awakeFlag			= 0x0002,
		e_autoSleepFlag		= 0x0004,
		e_bulletFlag		= 0x0008,
		e_fixedRotationFlag	= 0x0010,
		e_activeFlag		= 0x0020,
		e_toiFlag			= 0x0040
	};

	b3Body(const b3BodyDef* bd, b3World* world);
	~b3Body();

	void SynchronizeFixtures();
	void SynchronizeTransform();

	// This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collideConnected flag.
	bool ShouldCollide(const b3Body* other) const;

	void Advance(float32 t);

	b3BodyType m_type;

	uint16 m_flags;

	int32 m_islandIndex;

	b3Transform m_xf;		// the body origin transform
	b3Sweep m_sweep;		// the swept motion for CCD

	b3Vec2 m_linearVelocity;
	float32 m_angularVelocity;

	b3Vec2 m_force;
	float32 m_torque;

	b3World* m_world;
	b3Body* m_prev;
	b3Body* m_next;

	b3Fixture* m_fixtureList;
	int32 m_fixtureCount;

	b3JointEdge* m_jointList;
	b3ContactEdge* m_contactList;

	float32 m_mass, m_invMass;

	// Rotational inertia about the center of mass.
	float32 m_I, m_invI;

	float32 m_linearDamping;
	float32 m_angularDamping;
	float32 m_gravityScale;

	float32 m_sleepTime;

	void* m_userData;
};

inline b3BodyType b3Body::GetType() const
{
	return m_type;
}

inline const b3Transform& b3Body::GetTransform() const
{
	return m_xf;
}

inline const b3Vec2& b3Body::GetPosition() const
{
	return m_xf.p;
}

inline float32 b3Body::GetAngle() const
{
	return m_sweep.a;
}

inline const b3Vec2& b3Body::GetWorldCenter() const
{
	return m_sweep.c;
}

inline const b3Vec2& b3Body::GetLocalCenter() const
{
	return m_sweep.localCenter;
}

inline void b3Body::SetLinearVelocity(const b3Vec2& v)
{
	if (m_type == b3_staticBody)
	{
		return;
	}

	//if (b3Transform(v,v) > 0.0f)???
	{
		SetAwake(true);
	}

	m_linearVelocity = v;
}

inline const b3Vec2& b3Body::GetLinearVelocity() const
{
	return m_linearVelocity;
}

inline void b3Body::SetAngularVelocity(float32 w)
{
	if (m_type == b3_staticBody)
	{
		return;
	}

	if (w * w > 0.0f)
	{
		SetAwake(true);
	}

	m_angularVelocity = w;
}

inline float32 b3Body::GetAngularVelocity() const
{
	return m_angularVelocity;
}

inline float32 b3Body::GetMass() const
{
	return m_mass;
}

inline float32 b3Body::GetInertia() const
{
	return m_I + m_mass * b3Dot(m_sweep.localCenter, m_sweep.localCenter);
}

inline void b3Body::GetMassData(b3MassData* data) const
{
	data->mass = m_mass;
	data->I = m_I + m_mass * b3Dot(m_sweep.localCenter, m_sweep.localCenter);
	data->center = m_sweep.localCenter;
}

inline b3Vec2 b3Body::GetWorldPoint(const b3Vec2& localPoint) const
{
	return b3Mul(m_xf, localPoint);
}

inline b3Vec2 b3Body::GetWorldVector(const b3Vec2& localVector) const
{
	return b3Mul(m_xf.q, localVector);
}

inline b3Vec2 b3Body::GetLocalPoint(const b3Vec2& worldPoint) const
{
	return b3MulT(m_xf, worldPoint);
}

inline b3Vec2 b3Body::GetLocalVector(const b3Vec2& worldVector) const
{
	return b3MulT(m_xf.q, worldVector);
}

inline b3Vec2 b3Body::GetLinearVelocityFromWorldPoint(const b3Vec2& worldPoint) const
{
	return m_linearVelocity + b3Cross(m_angularVelocity, worldPoint - m_sweep.c);
}

inline b3Vec2 b3Body::GetLinearVelocityFromLocalPoint(const b3Vec2& localPoint) const
{
	return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
}

inline float32 b3Body::GetLinearDamping() const
{
	return m_linearDamping;
}

inline void b3Body::SetLinearDamping(float32 linearDamping)
{
	m_linearDamping = linearDamping;
}

inline float32 b3Body::GetAngularDamping() const
{
	return m_angularDamping;
}

inline void b3Body::SetAngularDamping(float32 angularDamping)
{
	m_angularDamping = angularDamping;
}

inline float32 b3Body::GetGravityScale() const
{
	return m_gravityScale;
}

inline void b3Body::SetGravityScale(float32 scale)
{
	m_gravityScale = scale;
}

inline void b3Body::SetBullet(bool flag)
{
	if (flag)
	{
		m_flags |= e_bulletFlag;
	}
	else
	{
		m_flags &= ~e_bulletFlag;
	}
}

inline bool b3Body::IsBullet() const
{
	return (m_flags & e_bulletFlag) == e_bulletFlag;
}

inline void b3Body::SetAwake(bool flag)
{
	if (flag)
	{
		m_flags |= e_awakeFlag;
		m_sleepTime = 0.0f;
	}
	else
	{
		m_flags &= ~e_awakeFlag;
		m_sleepTime = 0.0f;
		m_linearVelocity.SetZero();
		m_angularVelocity = 0.0f;
		m_force.SetZero();
		m_torque = 0.0f;
	}
}

inline bool b3Body::IsAwake() const
{
	return (m_flags & e_awakeFlag) == e_awakeFlag;
}

inline bool b3Body::IsActive() const
{
	return (m_flags & e_activeFlag) == e_activeFlag;
}

inline bool b3Body::IsFixedRotation() const
{
	return (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
}

inline void b3Body::SetSleepingAllowed(bool flag)
{
	if (flag)
	{
		m_flags |= e_autoSleepFlag;
	}
	else
	{
		m_flags &= ~e_autoSleepFlag;
		SetAwake(true);
	}
}

inline bool b3Body::IsSleepingAllowed() const
{
	return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
}

inline b3Fixture* b3Body::GetFixtureList()
{
	return m_fixtureList;
}

inline const b3Fixture* b3Body::GetFixtureList() const
{
	return m_fixtureList;
}

inline b3JointEdge* b3Body::GetJointList()
{
	return m_jointList;
}

inline const b3JointEdge* b3Body::GetJointList() const
{
	return m_jointList;
}

inline b3ContactEdge* b3Body::GetContactList()
{
	return m_contactList;
}

inline const b3ContactEdge* b3Body::GetContactList() const
{
	return m_contactList;
}

inline b3Body* b3Body::GetNext()
{
	return m_next;
}

inline const b3Body* b3Body::GetNext() const
{
	return m_next;
}

inline void b3Body::SetUserData(void* data)
{
	m_userData = data;
}

inline void* b3Body::GetUserData() const
{
	return m_userData;
}

inline void b3Body::ApplyForce(const b3Vec2& force, const b3Vec2& point, bool wake)
{
	if (m_type != b3_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate a force if the body is sleeping.
	if (m_flags & e_awakeFlag)
	{
		m_force += force;
		m_torque += b3Cross(point - m_sweep.c, force);
	}
}

inline void b3Body::ApplyForceToCenter(const b3Vec2& force, bool wake)
{
	if (m_type != b3_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate a force if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
		m_force += force;
	}
}

inline void b3Body::ApplyTorque(float32 torque, bool wake)
{
	if (m_type != b3_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate a force if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
		m_torque += torque;
	}
}

inline void b3Body::ApplyLinearImpulse(const b3Vec2& impulse, const b3Vec2& point, bool wake)
{
	if (m_type != b3_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate velocity if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
		m_linearVelocity += m_invMass * impulse;
		m_angularVelocity += m_invI * b3Cross(point - m_sweep.c, impulse);
	}
}

inline void b3Body::ApplyLinearImpulseToCenter(const b3Vec2& impulse, bool wake)
{
	if (m_type != b3_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate velocity if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
		m_linearVelocity += m_invMass * impulse;
	}
}

inline void b3Body::ApplyAngularImpulse(float32 impulse, bool wake)
{
	if (m_type != b3_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate velocity if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
		m_angularVelocity += m_invI * impulse;
	}
}

inline void b3Body::SynchronizeTransform()
{
	m_xf.q.Set(m_sweep.a);
	m_xf.p = m_sweep.c - b3Mul(m_xf.q, m_sweep.localCenter);
}

inline void b3Body::Advance(float32 alpha)
{
	// Advance to the new safe time. This doesn't sync the broad-phase.
	m_sweep.Advance(alpha);
	m_sweep.c = m_sweep.c0;
	m_sweep.a = m_sweep.a0;
	m_xf.q.Set(m_sweep.a);
	m_xf.p = m_sweep.c - b3Mul(m_xf.q, m_sweep.localCenter);
}

inline b3World* b3Body::GetWorld()
{
	return m_world;
}

inline const b3World* b3Body::GetWorld() const
{
	return m_world;
}

#endif

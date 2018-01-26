/*
* Copyright (c) 2006-2007 Erin Catto http://www.Box3D.org
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

#ifndef b3_JOINT_H
#define b3_JOINT_H

#include "Box3D/Common/b3Math.h"

class b3Body;
class b3Joint;
struct b3SolverData;
class b3BlockAllocator;

enum b3JointType
{
	e_unknownJoint,
	e_revoluteJoint,
	e_prismaticJoint,
	e_distanceJoint,
	e_pulleyJoint,
	e_mouseJoint,
	e_gearJoint,
	e_wheelJoint,
    e_weldJoint,
	e_frictionJoint,
	e_ropeJoint,
	e_motorJoint
};

enum b3LimitState
{
	e_inactiveLimit,
	e_atLowerLimit,
	e_atUpperLimit,
	e_equalLimits
};

struct b3Jacobian
{
	b3Vec2 linear;
	float32 angularA;
	float32 angularB;
};

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
struct b3JointEdge
{
	b3Body* other;			///< provides quick access to the other body attached.
	b3Joint* joint;			///< the joint
	b3JointEdge* prev;		///< the previous joint edge in the body's joint list
	b3JointEdge* next;		///< the next joint edge in the body's joint list
};

/// Joint definitions are used to construct joints.
struct b3JointDef
{
	b3JointDef()
	{
		type = e_unknownJoint;
		userData = nullptr;
		bodyA = nullptr;
		bodyB = nullptr;
		collideConnected = false;
	}

	/// The joint type is set automatically for concrete joint types.
	b3JointType type;

	/// Use this to attach application specific data to your joints.
	void* userData;

	/// The first attached body.
	b3Body* bodyA;

	/// The second attached body.
	b3Body* bodyB;

	/// Set this flag to true if the attached bodies should collide.
	bool collideConnected;
};

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
class b3Joint
{
public:

	/// Get the type of the concrete joint.
	b3JointType GetType() const;

	/// Get the first body attached to this joint.
	b3Body* GetBodyA();

	/// Get the second body attached to this joint.
	b3Body* GetBodyB();

	/// Get the anchor point on bodyA in world coordinates.
	virtual b3Vec2 GetAnchorA() const = 0;

	/// Get the anchor point on bodyB in world coordinates.
	virtual b3Vec2 GetAnchorB() const = 0;

	/// Get the reaction force on bodyB at the joint anchor in Newtons.
	virtual b3Vec2 GetReactionForce(float32 inv_dt) const = 0;

	/// Get the reaction torque on bodyB in N*m.
	virtual float32 GetReactionTorque(float32 inv_dt) const = 0;

	/// Get the next joint the world joint list.
	b3Joint* GetNext();
	const b3Joint* GetNext() const;

	/// Get the user data pointer.
	void* GetUserData() const;

	/// Set the user data pointer.
	void SetUserData(void* data);

	/// Short-cut function to determine if either body is inactive.
	bool IsActive() const;

	/// Get collide connected.
	/// Note: modifying the collide connect flag won't work correctly because
	/// the flag is only checked when fixture AABBs begin to overlap.
	bool GetCollideConnected() const;

	/// Dump this joint to the log file.
	virtual void Dump() { b3Log("// Dump is not supported for this joint type.\n"); }

	/// Shift the origin for any points stored in world coordinates.
	virtual void ShiftOrigin(const b3Vec2& newOrigin) { B3_NOT_USED(newOrigin);  }

protected:
	friend class b3World;
	friend class b3Body;
	friend class b3Island;
	friend class b3GearJoint;

	static b3Joint* Create(const b3JointDef* def, b3BlockAllocator* allocator);
	static void Destroy(b3Joint* joint, b3BlockAllocator* allocator);

	b3Joint(const b3JointDef* def);
	virtual ~b3Joint() {}

	virtual void InitVelocityConstraints(const b3SolverData& data) = 0;
	virtual void SolveVelocityConstraints(const b3SolverData& data) = 0;

	// This returns true if the position errors are within tolerance.
	virtual bool SolvePositionConstraints(const b3SolverData& data) = 0;

	b3JointType m_type;
	b3Joint* m_prev;
	b3Joint* m_next;
	b3JointEdge m_edgeA;
	b3JointEdge m_edgeB;
	b3Body* m_bodyA;
	b3Body* m_bodyB;

	int32 m_index;

	bool m_islandFlag;
	bool m_collideConnected;

	void* m_userData;
};

inline b3JointType b3Joint::GetType() const
{
	return m_type;
}

inline b3Body* b3Joint::GetBodyA()
{
	return m_bodyA;
}

inline b3Body* b3Joint::GetBodyB()
{
	return m_bodyB;
}

inline b3Joint* b3Joint::GetNext()
{
	return m_next;
}

inline const b3Joint* b3Joint::GetNext() const
{
	return m_next;
}

inline void* b3Joint::GetUserData() const
{
	return m_userData;
}

inline void b3Joint::SetUserData(void* data)
{
	m_userData = data;
}

inline bool b3Joint::GetCollideConnected() const
{
	return m_collideConnected;
}

#endif

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

#ifndef b3_CONTACT_SOLVER_H
#define b3_CONTACT_SOLVER_H

#include "Box3D/Common/b3Math.h"
#include "Box3D/Collision/b3Collision.h"
#include "Box3D/Dynamics/b3TimeStep.h"

class b3Contact;
class b3Body;
class b3StackAllocator;
struct b3ContactPositionConstraint;

struct b3VelocityConstraintPoint
{
	b3Vec2 rA;
	b3Vec2 rB;
	float32 normalImpulse;
	float32 tangentImpulse;
	float32 normalMass;
	float32 tangentMass;
	float32 velocityBias;
};

struct b3ContactVelocityConstraint
{
	b3VelocityConstraintPoint points[b3_maxManifoldPoints];
	b3Vec2 normal;
	b3Mat22 normalMass;
	b3Mat22 K;
	int32 indexA;
	int32 indexB;
	float32 invMassA, invMassB;
	float32 invIA, invIB;
	float32 friction;
	float32 restitution;
	float32 tangentSpeed;
	int32 pointCount;
	int32 contactIndex;
};

struct b3ContactSolverDef
{
	b3TimeStep step;
	b3Contact** contacts;
	int32 count;
	b3Position* positions;
	b3Velocity* velocities;
	b3StackAllocator* allocator;
};

class b3ContactSolver
{
public:
	b3ContactSolver(b3ContactSolverDef* def);
	~b3ContactSolver();

	void InitializeVelocityConstraints();

	void WarmStart();
	void SolveVelocityConstraints();
	void StoreImpulses();

	bool SolvePositionConstraints();
	bool SolveTOIPositionConstraints(int32 toiIndexA, int32 toiIndexB);

	b3TimeStep m_step;
	b3Position* m_positions;
	b3Velocity* m_velocities;
	b3StackAllocator* m_allocator;
	b3ContactPositionConstraint* m_positionConstraints;
	b3ContactVelocityConstraint* m_velocityConstraints;
	b3Contact** m_contacts;
	int m_count;
};

#endif


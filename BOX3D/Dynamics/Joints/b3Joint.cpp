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

#include "Box3D/Dynamics/Joints/b3Joint.h"
//#include "Box3D/Dynamics/Joints/b3DistanceJoint.h"
//#include "Box3D/Dynamics/Joints/b3WheelJoint.h"
//#include "Box3D/Dynamics/Joints/b3MouseJoint.h"
//#include "Box3D/Dynamics/Joints/b3RevoluteJoint.h"
//#include "Box3D/Dynamics/Joints/b3PrismaticJoint.h"
//#include "Box3D/Dynamics/Joints/b3PulleyJoint.h"
//#include "Box3D/Dynamics/Joints/b3GearJoint.h"
//#include "Box3D/Dynamics/Joints/b3WeldJoint.h"
//#include "Box3D/Dynamics/Joints/b3FrictionJoint.h"
//#include "Box3D/Dynamics/Joints/b3RopeJoint.h"
//#include "Box3D/Dynamics/Joints/b3MotorJoint.h"
#include "Box3D/Dynamics/b3Body.h"
#include "Box3D/Dynamics/b3World.h"
#include "Box3D/Common/b3BlockAllocator.h"

#include <new>

b3Joint* b3Joint::Create(const b3JointDef* def, b3BlockAllocator* allocator)
{
	b3Joint* joint = nullptr;

	switch (def->type)
	{
	case e_distanceJoint:
		{
			//!!!

			//void* mem = allocator->Allocate(sizeof(b3DistanceJoint));
			//joint = new (mem) b3DistanceJoint(static_cast<const b3DistanceJointDef*>(def));
		}
		break;

	case e_mouseJoint:
		{
			//void* mem = allocator->Allocate(sizeof(b3MouseJoint));
			//joint = new (mem) b3MouseJoint(static_cast<const b3MouseJointDef*>(def));
		}
		break;

	case e_prismaticJoint:
		{
			//void* mem = allocator->Allocate(sizeof(b3PrismaticJoint));
			//joint = new (mem) b3PrismaticJoint(static_cast<const b3PrismaticJointDef*>(def));
		}
		break;

	case e_revoluteJoint:
		{
			//void* mem = allocator->Allocate(sizeof(b3RevoluteJoint));
			//joint = new (mem) b3RevoluteJoint(static_cast<const b3RevoluteJointDef*>(def));
		}
		break;

	case e_pulleyJoint:
		{
			//void* mem = allocator->Allocate(sizeof(b3PulleyJoint));
			//joint = new (mem) b3PulleyJoint(static_cast<const b3PulleyJointDef*>(def));
		}
		break;

	case e_gearJoint:
		{
			//void* mem = allocator->Allocate(sizeof(b3GearJoint));
			//joint = new (mem) b3GearJoint(static_cast<const b3GearJointDef*>(def));
		}
		break;

	case e_wheelJoint:
		{
			//void* mem = allocator->Allocate(sizeof(b3WheelJoint));
			//joint = new (mem) b3WheelJoint(static_cast<const b3WheelJointDef*>(def));
		}
		break;

	case e_weldJoint:
		{
			//void* mem = allocator->Allocate(sizeof(b3WeldJoint));
			//joint = new (mem) b3WeldJoint(static_cast<const b3WeldJointDef*>(def));
		}
		break;
        
	case e_frictionJoint:
		{
			//void* mem = allocator->Allocate(sizeof(b3FrictionJoint));
			//joint = new (mem) b3FrictionJoint(static_cast<const b3FrictionJointDef*>(def));
		}
		break;

	case e_ropeJoint:
		{
			//void* mem = allocator->Allocate(sizeof(b3RopeJoint));
			//joint = new (mem) b3RopeJoint(static_cast<const b3RopeJointDef*>(def));
		}
		break;

	case e_motorJoint:
		{
			//void* mem = allocator->Allocate(sizeof(b3MotorJoint));
			//joint = new (mem) b3MotorJoint(static_cast<const b3MotorJointDef*>(def));
		}
		break;

	default:
		b3Assert(false);
		break;
	}

	return joint;
}

void b3Joint::Destroy(b3Joint* joint, b3BlockAllocator* allocator)
{
	joint->~b3Joint();
	switch (joint->m_type)
	{
		//!!!
	//case e_distanceJoint:
		//allocator->Free(joint, sizeof(b3DistanceJoint));
		//break;

	//case e_mouseJoint:
		//allocator->Free(joint, sizeof(b3MouseJoint));
		//break;

	//case e_prismaticJoint:
		//allocator->Free(joint, sizeof(b3PrismaticJoint));
		//break;

	//case e_revoluteJoint:
		//allocator->Free(joint, sizeof(b3RevoluteJoint));
		//break;

	//case e_pulleyJoint:
		//allocator->Free(joint, sizeof(b3PulleyJoint));
		//break;

	//case e_gearJoint:
		//allocator->Free(joint, sizeof(b3GearJoint));
		//break;

	//case e_wheelJoint:
		//allocator->Free(joint, sizeof(b3WheelJoint));
		//break;
    
	//case e_weldJoint:
		//allocator->Free(joint, sizeof(b3WeldJoint));
		//break;

	//case e_frictionJoint:
		//allocator->Free(joint, sizeof(b3FrictionJoint));
		//break;

	//case e_ropeJoint:
		//allocator->Free(joint, sizeof(b3RopeJoint));
		//break;

	//case e_motorJoint:
		//allocator->Free(joint, sizeof(b3MotorJoint));
		//break;

	default:
		b3Assert(false);
		break;
	}
}

b3Joint::b3Joint(const b3JointDef* def)
{
	b3Assert(def->bodyA != def->bodyB);

	m_type = def->type;
	m_prev = nullptr;
	m_next = nullptr;
	m_bodyA = def->bodyA;
	m_bodyB = def->bodyB;
	m_index = 0;
	m_collideConnected = def->collideConnected;
	m_islandFlag = false;
	m_userData = def->userData;

	m_edgeA.joint = nullptr;
	m_edgeA.other = nullptr;
	m_edgeA.prev = nullptr;
	m_edgeA.next = nullptr;

	m_edgeB.joint = nullptr;
	m_edgeB.other = nullptr;
	m_edgeB.prev = nullptr;
	m_edgeB.next = nullptr;
}

bool b3Joint::IsActive() const
{
	return m_bodyA->IsActive() && m_bodyB->IsActive();
}

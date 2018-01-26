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

//!!!
#pragma once
/**
\mainpage Box3D API Documentation

\section intro_sec Getting Started

For documentation please see http://Box3D.org/documentation.html

For discussion please visit http://Box3D.org/forum
*/

// These include files constitute the main Box3D API

#include "Box3D/Common/b3Settings.h"
#include "Box3D/Common/b3Draw.h"
//#include "Box3D/Common/b3Math.h"
#include "Box3D/Common/b3Timer.h"

//#include "Box3D/Collision/Shapes/b3CircleShape.h"
//#include "Box3D/Collision/Shapes/b3EdgeShape.h"
//#include "Box3D/Collision/Shapes/b3ChainShape.h"
//#include "Box3D/Collision/Shapes/b3PolygonShape.h"

#include "Box3D/Collision/b3BroadPhase.h"
#include "Box3D/Collision/b3Distance.h"
#include "Box3D/Collision/b3DynamicTree.h"
#include "Box3D/Collision/b3TimeOfImpact.h"

#include "Box3D/Dynamics/b3Body.h"
#include "Box3D/Dynamics/b3Fixture.h"
#include "Box3D/Dynamics/b3WorldCallbacks.h"
#include "Box3D/Dynamics/b3TimeStep.h"
#include "Box3D/Dynamics/b3World.h"

#include "Box3D/Dynamics/Contacts/b3Contact.h"

//#include "Box3D/Dynamics/Joints/b3DistanceJoint.h"
//#include "Box3D/Dynamics/Joints/b3FrictionJoint.h"
//#include "Box3D/Dynamics/Joints/b3GearJoint.h"
//#include "Box3D/Dynamics/Joints/b3MotorJoint.h"
//#include "Box3D/Dynamics/Joints/b3MouseJoint.h"
//#include "Box3D/Dynamics/Joints/b3PrismaticJoint.h"
//#include "Box3D/Dynamics/Joints/b3PulleyJoint.h"
//#include "Box3D/Dynamics/Joints/b3RevoluteJoint.h"
//#include "Box3D/Dynamics/Joints/b3RopeJoint.h"
//#include "Box3D/Dynamics/Joints/b3WeldJoint.h"
//#include "Box3D/Dynamics/Joints/b3WheelJoint.h"
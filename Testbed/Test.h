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

#ifndef TEST_H
#define TEST_H

#include "Box3D/Box3D.h"
#include "DebugDraw.h"

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include "glew/glew.h"
#endif
#include "glfw/glfw3.h"

#include <stdlib.h>

class Test;
struct Settings;

typedef Test* TestCreateFcn();

#define	RAND_LIMIT	32767
#define DRAW_STRING_NEW_LINE 16

/// Random number in range [-1,1]
inline float32 RandomFloat()
{
	float32 r = (float32)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = 2.0f * r - 1.0f;
	return r;
}

/// Random floating point number in range [lo, hi]
inline float32 RandomFloat(float32 lo, float32 hi)
{
	float32 r = (float32)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = (hi - lo) * r + lo;
	return r;
}

/// Test settings. Some can be controlled in the GUI.
struct Settings
{
	Settings()
	{
		hz = 60.0f;
		velocityIterations = 8;
		positionIterations = 3;
		drawShapes = true;
		drawJoints = true;
		drawAABBs = false;
		drawContactPoints = false;
		drawContactNormals = false;
		drawContactImpulse = false;
		drawFrictionImpulse = false;
		drawCOMs = false;
		drawStats = false;
		drawProfile = false;
		enableWarmStarting = true;
		enableContinuous = true;
		enableSubStepping = false;
		enableSleep = true;
		pause = false;
		singleStep = false;
	}

	float32 hz;
	int32 velocityIterations;
	int32 positionIterations;
	bool drawShapes;
	bool drawJoints;
	bool drawAABBs;
	bool drawContactPoints;
	bool drawContactNormals;
	bool drawContactImpulse;
	bool drawFrictionImpulse;
	bool drawCOMs;
	bool drawStats;
	bool drawProfile;
	bool enableWarmStarting;
	bool enableContinuous;
	bool enableSubStepping;
	bool enableSleep;
	bool pause;
	bool singleStep;
};

struct TestEntry
{
	const char *name;
	TestCreateFcn *createFcn;
};

extern TestEntry g_testEntries[];
// This is called when a joint in the world is implicitly destroyed
// because an attached body is destroyed. This gives us a chance to
// nullify the mouse joint.
class DestructionListener : public b3DestructionListener
{
public:
	void SayGoodbye(b3Fixture* fixture) override { B3_NOT_USED(fixture); }
	void SayGoodbye(b3Joint* joint) override;

	Test* test;
};

const int32 k_maxContactPoints = 2048;

struct ContactPoint
{
	b3Fixture* fixtureA;
	b3Fixture* fixtureB;
	b3Vec2 normal;
	b3Vec2 position;
	b3PointState state;
	float32 normalImpulse;
	float32 tangentImpulse;
	float32 separation;
};

class Test : public b3ContactListener
{
public:

	Test();
	virtual ~Test();

	void DrawTitle(const char *string);
	virtual void Step(Settings* settings);
	virtual void Keyboard(int key) { B3_NOT_USED(key); }
	virtual void KeyboardUp(int key) { B3_NOT_USED(key); }
	void ShiftMouseDown(const b3Vec2& p);
	virtual void MouseDown(const b3Vec2& p);
	virtual void MouseUp(const b3Vec2& p);
	void MouseMove(const b3Vec2& p);
	void LaunchBomb();
	void LaunchBomb(const b3Vec2& position, const b3Vec2& velocity);

	void SpawnBomb(const b3Vec2& worldPt);
	void CompleteBombSpawn(const b3Vec2& p);

	// Let derived tests know that a joint was destroyed.
	virtual void JointDestroyed(b3Joint* joint) { B3_NOT_USED(joint); }

	// Callbacks for derived classes.
	virtual void BeginContact(b3Contact* contact)  override { B3_NOT_USED(contact); }
	virtual void EndContact(b3Contact* contact)  override { B3_NOT_USED(contact); }
	virtual void PreSolve(b3Contact* contact, const b3Manifold* oldManifold) override;
	virtual void PostSolve(b3Contact* contact, const b3ContactImpulse* impulse) override
	{
		B3_NOT_USED(contact);
		B3_NOT_USED(impulse);
	}

	void ShiftOrigin(const b3Vec2& newOrigin);

protected:
	friend class DestructionListener;
	friend class BoundaryListener;
	friend class ContactListener;

	b3Body* m_groundBody;
	b3AABB	m_worldAABB;
	ContactPoint m_points[k_maxContactPoints];
	int32 m_pointCount;
	DestructionListener m_destructionListener;
	int32 m_textLine;
	b3World* m_world;
	b3Body* m_bomb;
	b3MouseJoint* m_mouseJoint;
	b3Vec2 m_bombSpawnPoint;
	bool m_bombSpawning;
	b3Vec2 m_mouseWorld;
	int32 m_stepCount;

	b3Profile m_maxProfile;
	b3Profile m_totalProfile;
};

#endif

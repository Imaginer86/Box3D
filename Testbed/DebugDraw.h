/*
* Copyright (c) 2006-2013 Erin Catto http://www.Box3D.org
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

#ifndef DEBUGDRAW_H
#define DEBUGDRAW_H

#include "Box3D/Box3D.h"

struct b2ABB;
struct GLRenderPoints;
struct GLRenderLines;
struct GLRenderTriangles;

//
struct Camera
{
	Camera()
	{
		m_center.Set(0.0f, 20.0f);
		m_zoom = 1.0f;
		m_width = 1280;
		m_height = 800;
	}

	b3Vec2 ConvertScreenToWorld(const b3Vec2& screenPoint);
	b3Vec2 ConvertWorldToScreen(const b3Vec2& worldPoint);
	void BuildProjectionMatrix(float32* m, float32 zBias);

	b3Vec2 m_center;
	float32 m_zoom;
	int32 m_width;
	int32 m_height;
};

// This class implements debug drawing callbacks that are invoked
// inside b3World::Step.
class DebugDraw : public b3Draw
{
public:
	DebugDraw();
	~DebugDraw();

	void Create();
	void Destroy();

	void DrawPolygon(const b3Vec2* vertices, int32 vertexCount, const b3Color& color) override;

	void DrawSolidPolygon(const b3Vec2* vertices, int32 vertexCount, const b3Color& color) override;

	void DrawCircle(const b3Vec2& center, float32 radius, const b3Color& color) override;

	void DrawSolidCircle(const b3Vec2& center, float32 radius, const b3Vec2& axis, const b3Color& color) override;

	void DrawSegment(const b3Vec2& p1, const b3Vec2& p2, const b3Color& color) override;

	void DrawTransform(const b3Transform& xf) override;

	void DrawPoint(const b3Vec2& p, float32 size, const b3Color& color) override;

	void DrawString(int x, int y, const char* string, ...); 

	void DrawString(const b3Vec2& p, const char* string, ...);

	void DrawAABB(b3AABB* aabb, const b3Color& color);

	void Flush();

private:
	GLRenderPoints* m_points;
	GLRenderLines* m_lines;
	GLRenderTriangles* m_triangles;
};

extern DebugDraw g_debugDraw;
extern Camera g_camera;

#endif

#pragma once

#include "BOX3D\Common\b3Math.h"

/// Color for debug drawing. Each value has the range [0,1].
struct b3Color
{
	b3Color() {}
	b3Color(float32 rIn, float32 gIn, float32 bIn, float32 aIn = 1.0f)
	{
		r = rIn; g = gIn; b = bIn; a = aIn;
	}

	void Set(float32 rIn, float32 gIn, float32 bIn, float32 aIn = 1.0f)
	{
		r = rIn; g = gIn; b = bIn; a = aIn;
	}

	float32 r, g, b, a;
};

// Implement and register this class with a b3World to provide debug drawing of physics
/// entities in your game.
class b3Draw
{
public:
	b3Draw();

	virtual ~b3Draw() {}

	enum
	{
		e_shapeBit = 0x0001,	///< draw shapes
		e_jointBit = 0x0002,	///< draw joint connections
		e_aabbBit = 0x0004,	///< draw axis aligned bounding boxes
		e_pairBit = 0x0008,	///< draw broad-phase pairs
		e_centerOfMassBit = 0x0010	///< draw center of mass frame
	};

	/// Set the drawing flags.
	void SetFlags(uint32 flags);

	/// Get the drawing flags.
	uint32 GetFlags() const;

	/// Append flags to the current flags.
	void AppendFlags(uint32 flags);

	/// Clear flags from the current flags.
	void ClearFlags(uint32 flags);

	/// Draw a closed polygon provided in CCW order.
	virtual void DrawPolygon(const b3Vec2* vertices, int32 vertexCount, const b3Color& color) = 0;

	/// Draw a solid closed polygon provided in CCW order.
	virtual void DrawSolidPolygon(const b3Vec2* vertices, int32 vertexCount, const b3Color& color) = 0;

	/// Draw a circle.
	virtual void DrawCircle(const b3Vec2& center, float32 radius, const b3Color& color) = 0;

	/// Draw a solid circle.
	virtual void DrawSolidCircle(const b3Vec2& center, float32 radius, const b3Vec2& axis, const b3Color& color) = 0;

	/// Draw a line segment.
	virtual void DrawSegment(const b3Vec2& p1, const b3Vec2& p2, const b3Color& color) = 0;

	/// Draw a transform. Choose your own length scale.
	/// @param xf a transform.
	virtual void DrawTransform(const b3Transform& xf) = 0;

	/// Draw a point.
	virtual void DrawPoint(const b3Vec2& p, float32 size, const b3Color& color) = 0;

protected:
	uint32 m_drawFlags;
};
#pragma once
#include "BOX3D\Common\b3Settings.h"
#include <cmath>



/// This function is used to ensure that a floating point number is not a NaN or infinity.
inline bool b3IsValid(float32 x)
{
	int32 ix = *reinterpret_cast<int32*>(&x);
	return (ix & 0x7f800000) != 0x7f800000;
}

/// This is a approximate yet fast inverse square-root.
inline float32 b3InvSqrt(float32 x)
{
	union
	{
		float32 x;
		int32 i;
	} convert;

	convert.x = x;
	float32 xhalf = 0.5f * x;
	convert.i = 0x5f3759df - (convert.i >> 1);
	x = convert.x;
	x = x * (1.5f - xhalf * x * x);
	return x;
}

#define	b3Sqrt(x)	sqrtf(x)
#define	b3Atan2(y, x)	atan2f(y, x)

/// A 2D column vector.
struct b3Vec2
{
	/// Default constructor does nothing (for performance).
	b3Vec2() {}

	/// Construct using coordinates.
	b3Vec2(float32 xIn, float32 yIn) : x(xIn), y(yIn) {}

	/// Set this vector to all zeros.
	void SetZero() { x = 0.0f; y = 0.0f; }

	/// Set this vector to some specified coordinates.
	void Set(float32 x_, float32 y_) { x = x_; y = y_; }

	/// Negate this vector.
	b3Vec2 operator -() const { b3Vec2 v; v.Set(-x, -y); return v; }

	/// Read from and indexed element.
	float32 operator () (int32 i) const
	{
		return (&x)[i];
	}

	/// Write to an indexed element.
	float32& operator () (int32 i)
	{
		return (&x)[i];
	}

	/// Add a vector to this vector.
	void operator += (const b3Vec2& v)
	{
		x += v.x; y += v.y;
	}

	/// Subtract a vector from this vector.
	void operator -= (const b3Vec2& v)
	{
		x -= v.x; y -= v.y;
	}

	/// Multiply this vector by a scalar.
	void operator *= (float32 a)
	{
		x *= a; y *= a;
	}

	/// Get the length of this vector (the norm).
	float32 Length() const
	{
		return b3Sqrt(x * x + y * y);
	}

	/// Get the length squared. For performance, use this instead of
	/// b3Vec2::Length (if possible).
	float32 LengthSquared() const
	{
		return x * x + y * y;
	}

	/// Convert this vector into a unit vector. Returns the length.
	float32 Normalize()
	{
		float32 length = Length();
		if (length < b3_epsilon)
		{
			return 0.0f;
		}
		float32 invLength = 1.0f / length;
		x *= invLength;
		y *= invLength;

		return length;
	}

	/// Does this vector contain finite coordinates?
	bool IsValid() const
	{
		return b3IsValid(x) && b3IsValid(y);
	}

	/// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
	b3Vec2 Skew() const
	{
		return b3Vec2(-y, x);
	}

	float32 x, y;
};

/// A 2D column vector with 3 elements.
struct b3Vec3
{
	/// Default constructor does nothing (for performance).
	b3Vec3() {}

	/// Construct using coordinates.
	b3Vec3(float32 xIn, float32 yIn, float32 zIn) : x(xIn), y(yIn), z(zIn) {}

	/// Set this vector to all zeros.
	void SetZero() { x = 0.0f; y = 0.0f; z = 0.0f; }

	/// Set this vector to some specified coordinates.
	void Set(float32 x_, float32 y_, float32 z_) { x = x_; y = y_; z = z_; }

	/// Negate this vector.
	b3Vec3 operator -() const { b3Vec3 v; v.Set(-x, -y, -z); return v; }

	/// Add a vector to this vector.
	void operator += (const b3Vec3& v)
	{
		x += v.x; y += v.y; z += v.z;
	}

	/// Subtract a vector from this vector.
	void operator -= (const b3Vec3& v)
	{
		x -= v.x; y -= v.y; z -= v.z;
	}

	/// Multiply this vector by a scalar.
	void operator *= (float32 s)
	{
		x *= s; y *= s; z *= s;
	}

	float32 x, y, z;
};

/// A 2-by-2 matrix. Stored in column-major order.
struct b3Mat22
{
	/// The default constructor does nothing (for performance).
	b3Mat22() {}

	/// Construct this matrix using columns.
	b3Mat22(const b3Vec2& c1, const b3Vec2& c2)
	{
		ex = c1;
		ey = c2;
	}

	/// Construct this matrix using scalars.
	b3Mat22(float32 a11, float32 a12, float32 a21, float32 a22)
	{
		ex.x = a11; ex.y = a21;
		ey.x = a12; ey.y = a22;
	}

	/// Initialize this matrix using columns.
	void Set(const b3Vec2& c1, const b3Vec2& c2)
	{
		ex = c1;
		ey = c2;
	}

	/// Set this to the identity matrix.
	void SetIdentity()
	{
		ex.x = 1.0f; ey.x = 0.0f;
		ex.y = 0.0f; ey.y = 1.0f;
	}

	/// Set this matrix to all zeros.
	void SetZero()
	{
		ex.x = 0.0f; ey.x = 0.0f;
		ex.y = 0.0f; ey.y = 0.0f;
	}

	b3Mat22 GetInverse() const
	{
		float32 a = ex.x, b = ey.x, c = ex.y, d = ey.y;
		b3Mat22 B;
		float32 det = a * d - b * c;
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}
		B.ex.x = det * d;	B.ey.x = -det * b;
		B.ex.y = -det * c;	B.ey.y = det * a;
		return B;
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	b3Vec2 Solve(const b3Vec2& b) const
	{
		float32 a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
		float32 det = a11 * a22 - a12 * a21;
		if (det != 0.0f)
		{
			det = 1.0f / det;
		}
		b3Vec2 x;
		x.x = det * (a22 * b.x - a12 * b.y);
		x.y = det * (a11 * b.y - a21 * b.x);
		return x;
	}

	b3Vec2 ex, ey;
};

/// A 3-by-3 matrix. Stored in column-major order.
struct b3Mat33
{
	/// The default constructor does nothing (for performance).
	b3Mat33() {}

	/// Construct this matrix using columns.
	b3Mat33(const b3Vec3& c1, const b3Vec3& c2, const b3Vec3& c3)
	{
		ex = c1;
		ey = c2;
		ez = c3;
	}

	/// Set this matrix to all zeros.
	void SetZero()
	{
		ex.SetZero();
		ey.SetZero();
		ez.SetZero();
	}

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases.
	b3Vec3 Solve33(const b3Vec3& b) const;

	/// Solve A * x = b, where b is a column vector. This is more efficient
	/// than computing the inverse in one-shot cases. Solve only the upper
	/// 2-by-2 matrix equation.
	b3Vec2 Solve22(const b3Vec2& b) const;

	/// Get the inverse of this matrix as a 2-by-2.
	/// Returns the zero matrix if singular.
	void GetInverse22(b3Mat33* M) const;

	/// Get the symmetric inverse of this matrix as a 3-by-3.
	/// Returns the zero matrix if singular.
	void GetSymInverse33(b3Mat33* M) const;

	b3Vec3 ex, ey, ez;
};

/// Rotation
struct b3Rot
{
	b3Rot() {}

	/// Initialize from an angle in radians
	//explicit b3Rot(b3Vec2 v)
	//{
		/// TODO_ERIN optimize
		//s = sinf(v.x);
		//c = cosf(v.y);
	//}

	/// Initialize from an angle in radians
	explicit b3Rot(float32 angle)
	{
		/// TODO_ERIN optimize
		s = sinf(angle);
		c = cosf(angle);
	}

	/// Set using an angle in radians.
	void Set(float32 angle)
	{
		/// TODO_ERIN optimize
		s = sinf(angle);
		c = cosf(angle);
	}

	/// Set to the identity rotation
	void SetIdentity()
	{
		s = 0.0f;
		c = 1.0f;
	}

	/// Get the angle in radians
	float32 GetAngle() const
	{
		return b3Atan2(s, c);
	}

	/// Get the x-axis
	b3Vec2 GetXAxis() const
	{
		return b3Vec2(c, s);
	}

	/// Get the u-axis
	b3Vec2 GetYAxis() const
	{
		return b3Vec2(-s, c);
	}

	/// Sine and cosine
	float32 s, c;
};

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
struct b3Transform
{
	/// The default constructor does nothing.
	b3Transform() {}

	/// Initialize using a position vector and a rotation.
	b3Transform(const b3Vec2& position, const b3Rot& rotation) : p(position), q(rotation) {}

	b3Transform(const b3Vec2& position, const b3Vec2& rotation) : p(position), q(rotation.Length()) {}

	/// Set this to the identity transform.
	void SetIdentity()
	{
		p.SetZero();
		q.SetIdentity();
	}

	/// Set this based on the position and angle.
	void Set(const b3Vec2& position, float32 angle)
	{
		p = position;
		q.Set(angle);
	}

	b3Vec2 p;
	b3Rot q;
};

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
struct b3Sweep
{
	/// Get the interpolated transform at a specific time.
	/// @param beta is a factor in [0,1], where 0 indicates alpha0.
	void GetTransform(b3Transform* xfb, float32 beta) const;

	/// Advance the sweep forward, yielding a new initial state.
	/// @param alpha the new initial time.
	void Advance(float32 alpha);

	/// Normalize the angles.
	void Normalize();

	b3Vec2 localCenter;	///< local center of mass position
	b3Vec2 c0, c;		///< center world positions
	float32 a0, a;		///< world angles

						/// Fraction of the current time step in the range [0,1]
						/// c0 and a0 are the positions at alpha0.
	float32 alpha0;
};

/// Useful constant
extern const b3Vec2 b3Vec2_zero;

/// Perform the dot product on two vectors.
inline float32 b3Dot(const b3Vec2& a, const b3Vec2& b)
{
	return a.x * b.x + a.y * b.y;
}

/// Perform the cross product on two vectors. In 2D this produces a scalar.
inline float32 b3Cross(const b3Vec2& a, const b3Vec2& b)
{
	return a.x * b.y - a.y * b.x;
}

/// Perform the cross product on a vector and a scalar. In 2D this produces
/// a vector.
inline b3Vec2 b3Cross(const b3Vec2& a, float32 s)
{
	return b3Vec2(s * a.y, -s * a.x);
}

/// Perform the cross product on a scalar and a vector. In 2D this produces
/// a vector.
inline b3Vec2 b3Cross(float32 s, const b3Vec2& a)
{
	return b3Vec2(-s * a.y, s * a.x);
}

/// Multiply a matrix times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another.
inline b3Vec2 b3Mul(const b3Mat22& A, const b3Vec2& v)
{
	return b3Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply a matrix transpose times a vector. If a rotation matrix is provided,
/// then this transforms the vector from one frame to another (inverse transform).
inline b3Vec2 b3MulT(const b3Mat22& A, const b3Vec2& v)
{
	return b3Vec2(b3Dot(v, A.ex), b3Dot(v, A.ey));
}

/// Add two vectors component-wise.
inline b3Vec2 operator + (const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(a.x + b.x, a.y + b.y);
}

/// Subtract two vectors component-wise.
inline b3Vec2 operator - (const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(a.x - b.x, a.y - b.y);
}

inline b3Vec2 operator * (float32 s, const b3Vec2& a)
{
	return b3Vec2(s * a.x, s * a.y);
}

inline bool operator == (const b3Vec2& a, const b3Vec2& b)
{
	return a.x == b.x && a.y == b.y;
}

inline bool operator != (const b3Vec2& a, const b3Vec2& b)
{
	return a.x != b.x || a.y != b.y;
}

inline float32 b3Distance(const b3Vec2& a, const b3Vec2& b)
{
	b3Vec2 c = a - b;
	return c.Length();
}

inline float32 b3DistanceSquared(const b3Vec2& a, const b3Vec2& b)
{
	b3Vec2 c = a - b;
	return b3Dot(c, c);
}

inline b3Vec3 operator * (float32 s, const b3Vec3& a)
{
	return b3Vec3(s * a.x, s * a.y, s * a.z);
}

/// Add two vectors component-wise.
inline b3Vec3 operator + (const b3Vec3& a, const b3Vec3& b)
{
	return b3Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

/// Subtract two vectors component-wise.
inline b3Vec3 operator - (const b3Vec3& a, const b3Vec3& b)
{
	return b3Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

/// Perform the dot product on two vectors.
inline float32 b3Dot(const b3Vec3& a, const b3Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Perform the cross product on two vectors.
inline b3Vec3 b3Cross(const b3Vec3& a, const b3Vec3& b)
{
	return b3Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

inline b3Mat22 operator + (const b3Mat22& A, const b3Mat22& B)
{
	return b3Mat22(A.ex + B.ex, A.ey + B.ey);
}

// A * B
inline b3Mat22 b3Mul(const b3Mat22& A, const b3Mat22& B)
{
	return b3Mat22(b3Mul(A, B.ex), b3Mul(A, B.ey));
}

// A^T * B
inline b3Mat22 b3MulT(const b3Mat22& A, const b3Mat22& B)
{
	b3Vec2 c1(b3Dot(A.ex, B.ex), b3Dot(A.ey, B.ex));
	b3Vec2 c2(b3Dot(A.ex, B.ey), b3Dot(A.ey, B.ey));
	return b3Mat22(c1, c2);
}

/// Multiply a matrix times a vector.
inline b3Vec3 b3Mul(const b3Mat33& A, const b3Vec3& v)
{
	return v.x * A.ex + v.y * A.ey + v.z * A.ez;
}

/// Multiply a matrix times a vector.
inline b3Vec2 b3Mul22(const b3Mat33& A, const b3Vec2& v)
{
	return b3Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y);
}

/// Multiply two rotations: q * r
inline b3Rot b3Mul(const b3Rot& q, const b3Rot& r)
{
	// [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
	// [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
	// s = qs * rc + qc * rs
	// c = qc * rc - qs * rs
	b3Rot qr;
	qr.s = q.s * r.c + q.c * r.s;
	qr.c = q.c * r.c - q.s * r.s;
	return qr;
}

/// Transpose multiply two rotations: qT * r
inline b3Rot b3MulT(const b3Rot& q, const b3Rot& r)
{
	// [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
	// [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
	// s = qc * rs - qs * rc
	// c = qc * rc + qs * rs
	b3Rot qr;
	qr.s = q.c * r.s - q.s * r.c;
	qr.c = q.c * r.c + q.s * r.s;
	return qr;
}

/// Rotate a vector
inline b3Vec2 b3Mul(const b3Rot& q, const b3Vec2& v)
{
	return b3Vec2(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y);
}

/// Inverse rotate a vector
inline b3Vec2 b3MulT(const b3Rot& q, const b3Vec2& v)
{
	return b3Vec2(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y);
}

inline b3Vec2 b3Mul(const b3Transform& T, const b3Vec2& v)
{
	float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
	float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;

	return b3Vec2(x, y);
}

inline b3Vec2 b3MulT(const b3Transform& T, const b3Vec2& v)
{
	float32 px = v.x - T.p.x;
	float32 py = v.y - T.p.y;
	float32 x = (T.q.c * px + T.q.s * py);
	float32 y = (-T.q.s * px + T.q.c * py);

	return b3Vec2(x, y);
}

// v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
//    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
inline b3Transform b3Mul(const b3Transform& A, const b3Transform& B)
{
	b3Transform C;
	C.q = b3Mul(A.q, B.q);
	C.p = b3Mul(A.q, B.p) + A.p;
	return C;
}

// v2 = A.q' * (B.q * v1 + B.p - A.p)
//    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
inline b3Transform b3MulT(const b3Transform& A, const b3Transform& B)
{
	b3Transform C;
	C.q = b3MulT(A.q, B.q);
	C.p = b3MulT(A.q, B.p - A.p);
	return C;
}

template <typename T>
inline T b3Abs(T a)
{
	return a > T(0) ? a : -a;
}

inline b3Vec2 b3Abs(const b3Vec2& a)
{
	return b3Vec2(b3Abs(a.x), b3Abs(a.y));
}

inline b3Mat22 b3Abs(const b3Mat22& A)
{
	return b3Mat22(b3Abs(A.ex), b3Abs(A.ey));
}

template <typename T>
inline T b3Min(T a, T b)
{
	return a < b ? a : b;
}

inline b3Vec2 b3Min(const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(b3Min(a.x, b.x), b3Min(a.y, b.y));
}

template <typename T>
inline T b3Max(T a, T b)
{
	return a > b ? a : b;
}

inline b3Vec2 b3Max(const b3Vec2& a, const b3Vec2& b)
{
	return b3Vec2(b3Max(a.x, b.x), b3Max(a.y, b.y));
}

template <typename T>
inline T b3Clamp(T a, T low, T high)
{
	return b3Max(low, b3Min(a, high));
}

inline b3Vec2 b3Clamp(const b3Vec2& a, const b3Vec2& low, const b3Vec2& high)
{
	return b3Max(low, b3Min(a, high));
}

template<typename T> inline void b3Swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

/// "Next Largest Power of 2
/// Given a binary integer value x, the next largest power of 2 can be computed by a SWAR algorithm
/// that recursively "folds" the upper bits into the lower bits. This process yields a bit vector with
/// the same most significant 1 as x, but all 1's below it. Adding 1 to that value yields the next
/// largest power of 2. For a 32-bit value:"
inline uint32 b3NextPowerOfTwo(uint32 x)
{
	x |= (x >> 1);
	x |= (x >> 2);
	x |= (x >> 4);
	x |= (x >> 8);
	x |= (x >> 16);
	return x + 1;
}

inline bool b3IsPowerOfTwo(uint32 x)
{
	bool result = x > 0 && (x & (x - 1)) == 0;
	return result;
}

inline void b3Sweep::GetTransform(b3Transform* xf, float32 beta) const
{
	xf->p = (1.0f - beta) * c0 + beta * c;
	float32 angle = (1.0f - beta) * a0 + beta * a;
	xf->q.Set(angle);

	// Shift to origin
	xf->p -= b3Mul(xf->q, localCenter);
}

inline void b3Sweep::Advance(float32 alpha)
{
	b3Assert(alpha0 < 1.0f);
	float32 beta = (alpha - alpha0) / (1.0f - alpha0);
	c0 += beta * (c - c0);
	a0 += beta * (a - a0);
	alpha0 = alpha;
}

/// Normalize an angle in radians to be between -pi and pi
inline void b3Sweep::Normalize()
{
	float32 twoPi = 2.0f * b3_pi;
	float32 d = twoPi * floorf(a0 / twoPi);
	a0 -= d;
	a -= d;
}

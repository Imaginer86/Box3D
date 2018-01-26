/*
* Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include "Box3D/Collision/b3Collision.h"
//#include "Box3D/Collision/b3Distance.h"
#include "Box3D/Collision/b3TimeOfImpact.h"
//#include "Box3D/Collision/Shapes/b3CircleShape.h"
//#include "Box3D/Collision/Shapes/b3PolygonShape.h"
#include "Box3D/Common/b3Timer.h"

#include <stdio.h>

float32 b3_toiTime, b3_toiMaxTime;
int32 b3_toiCalls, b3_toiIters, b3_toiMaxIters;
int32 b3_toiRootIters, b3_toiMaxRootIters;

//
struct b3SeparationFunction
{
	enum Type
	{
		e_points,
		e_faceA,
		e_faceB
	};

	// TODO_ERIN might not need to return the separation

	float32 Initialize(const b3SimplexCache* cache,
		const b3DistanceProxy* proxyA, const b3Sweep& sweepA,
		const b3DistanceProxy* proxyB, const b3Sweep& sweepB,
		float32 t1)
	{
		m_proxyA = proxyA;
		m_proxyB = proxyB;
		int32 count = cache->count;
		b3Assert(0 < count && count < 3);

		m_sweepA = sweepA;
		m_sweepB = sweepB;

		b3Transform xfA, xfB;
		m_sweepA.GetTransform(&xfA, t1);
		m_sweepB.GetTransform(&xfB, t1);

		if (count == 1)
		{
			m_type = e_points;
			b3Vec2 localPointA = m_proxyA->GetVertex(cache->indexA[0]);
			b3Vec2 localPointB = m_proxyB->GetVertex(cache->indexB[0]);
			b3Vec2 pointA = b3Mul(xfA, localPointA);
			b3Vec2 pointB = b3Mul(xfB, localPointB);
			m_axis = pointB - pointA;
			float32 s = m_axis.Normalize();
			return s;
		}
		else if (cache->indexA[0] == cache->indexA[1])
		{
			// Two points on B and one on A.
			m_type = e_faceB;
			b3Vec2 localPointB1 = proxyB->GetVertex(cache->indexB[0]);
			b3Vec2 localPointb3 = proxyB->GetVertex(cache->indexB[1]);

			m_axis = b3Cross(localPointb3 - localPointB1, 1.0f);
			m_axis.Normalize();
			b3Vec2 normal = b3Mul(xfB.q, m_axis);

			m_localPoint = 0.5f * (localPointB1 + localPointb3);
			b3Vec2 pointB = b3Mul(xfB, m_localPoint);

			b3Vec2 localPointA = proxyA->GetVertex(cache->indexA[0]);
			b3Vec2 pointA = b3Mul(xfA, localPointA);

			float32 s = b3Dot(pointA - pointB, normal);
			if (s < 0.0f)
			{
				m_axis = -m_axis;
				s = -s;
			}
			return s;
		}
		else
		{
			// Two points on A and one or two points on B.
			m_type = e_faceA;
			b3Vec2 localPointA1 = m_proxyA->GetVertex(cache->indexA[0]);
			b3Vec2 localPointA2 = m_proxyA->GetVertex(cache->indexA[1]);
			
			m_axis = b3Cross(localPointA2 - localPointA1, 1.0f);
			m_axis.Normalize();
			b3Vec2 normal = b3Mul(xfA.q, m_axis);

			m_localPoint = 0.5f * (localPointA1 + localPointA2);
			b3Vec2 pointA = b3Mul(xfA, m_localPoint);

			b3Vec2 localPointB = m_proxyB->GetVertex(cache->indexB[0]);
			b3Vec2 pointB = b3Mul(xfB, localPointB);

			float32 s = b3Dot(pointB - pointA, normal);
			if (s < 0.0f)
			{
				m_axis = -m_axis;
				s = -s;
			}
			return s;
		}
	}

	//
	float32 FindMinSeparation(int32* indexA, int32* indexB, float32 t) const
	{
		b3Transform xfA, xfB;
		m_sweepA.GetTransform(&xfA, t);
		m_sweepB.GetTransform(&xfB, t);

		switch (m_type)
		{
		case e_points:
			{
				b3Vec2 axisA = b3MulT(xfA.q,  m_axis);
				b3Vec2 axisB = b3MulT(xfB.q, -m_axis);

				*indexA = m_proxyA->GetSupport(axisA);
				*indexB = m_proxyB->GetSupport(axisB);

				b3Vec2 localPointA = m_proxyA->GetVertex(*indexA);
				b3Vec2 localPointB = m_proxyB->GetVertex(*indexB);
				
				b3Vec2 pointA = b3Mul(xfA, localPointA);
				b3Vec2 pointB = b3Mul(xfB, localPointB);

				float32 separation = b3Dot(pointB - pointA, m_axis);
				return separation;
			}

		case e_faceA:
			{
				b3Vec2 normal = b3Mul(xfA.q, m_axis);
				b3Vec2 pointA = b3Mul(xfA, m_localPoint);

				b3Vec2 axisB = b3MulT(xfB.q, -normal);
				
				*indexA = -1;
				*indexB = m_proxyB->GetSupport(axisB);

				b3Vec2 localPointB = m_proxyB->GetVertex(*indexB);
				b3Vec2 pointB = b3Mul(xfB, localPointB);

				float32 separation = b3Dot(pointB - pointA, normal);
				return separation;
			}

		case e_faceB:
			{
				b3Vec2 normal = b3Mul(xfB.q, m_axis);
				b3Vec2 pointB = b3Mul(xfB, m_localPoint);

				b3Vec2 axisA = b3MulT(xfA.q, -normal);

				*indexB = -1;
				*indexA = m_proxyA->GetSupport(axisA);

				b3Vec2 localPointA = m_proxyA->GetVertex(*indexA);
				b3Vec2 pointA = b3Mul(xfA, localPointA);

				float32 separation = b3Dot(pointA - pointB, normal);
				return separation;
			}

		default:
			b3Assert(false);
			*indexA = -1;
			*indexB = -1;
			return 0.0f;
		}
	}

	//
	float32 Evaluate(int32 indexA, int32 indexB, float32 t) const
	{
		b3Transform xfA, xfB;
		m_sweepA.GetTransform(&xfA, t);
		m_sweepB.GetTransform(&xfB, t);

		switch (m_type)
		{
		case e_points:
			{
				b3Vec2 localPointA = m_proxyA->GetVertex(indexA);
				b3Vec2 localPointB = m_proxyB->GetVertex(indexB);

				b3Vec2 pointA = b3Mul(xfA, localPointA);
				b3Vec2 pointB = b3Mul(xfB, localPointB);
				float32 separation = b3Dot(pointB - pointA, m_axis);

				return separation;
			}

		case e_faceA:
			{
				b3Vec2 normal = b3Mul(xfA.q, m_axis);
				b3Vec2 pointA = b3Mul(xfA, m_localPoint);

				b3Vec2 localPointB = m_proxyB->GetVertex(indexB);
				b3Vec2 pointB = b3Mul(xfB, localPointB);

				float32 separation = b3Dot(pointB - pointA, normal);
				return separation;
			}

		case e_faceB:
			{
				b3Vec2 normal = b3Mul(xfB.q, m_axis);
				b3Vec2 pointB = b3Mul(xfB, m_localPoint);

				b3Vec2 localPointA = m_proxyA->GetVertex(indexA);
				b3Vec2 pointA = b3Mul(xfA, localPointA);

				float32 separation = b3Dot(pointA - pointB, normal);
				return separation;
			}

		default:
			b3Assert(false);
			return 0.0f;
		}
	}

	const b3DistanceProxy* m_proxyA;
	const b3DistanceProxy* m_proxyB;
	b3Sweep m_sweepA, m_sweepB;
	Type m_type;
	b3Vec2 m_localPoint;
	b3Vec2 m_axis;
};

// CCD via the local separating axis method. This seeks progression
// by computing the largest time at which separation is maintained.
void b3TimeOfImpact(b3TOIOutput* output, const b3TOIInput* input)
{
	b3Timer timer;

	++b3_toiCalls;

	output->state = b3TOIOutput::e_unknown;
	output->t = input->tMax;

	const b3DistanceProxy* proxyA = &input->proxyA;
	const b3DistanceProxy* proxyB = &input->proxyB;

	b3Sweep sweepA = input->sweepA;
	b3Sweep sweepB = input->sweepB;

	// Large rotations can make the root finder fail, so we normalize the
	// sweep angles.
	sweepA.Normalize();
	sweepB.Normalize();

	float32 tMax = input->tMax;

	float32 totalRadius = proxyA->m_radius + proxyB->m_radius;
	float32 target = b3Max(b3_linearSlop, totalRadius - 3.0f * b3_linearSlop);
	float32 tolerance = 0.25f * b3_linearSlop;
	b3Assert(target > tolerance);

	float32 t1 = 0.0f;
	const int32 k_maxIterations = 20;	// TODO_ERIN b3Settings
	int32 iter = 0;

	// Prepare input for distance query.
	b3SimplexCache cache;
	cache.count = 0;
	b3DistanceInput distanceInput;
	distanceInput.proxyA = input->proxyA;
	distanceInput.proxyB = input->proxyB;
	distanceInput.useRadii = false;

	// The outer loop progressively attempts to compute new separating axes.
	// This loop terminates when an axis is repeated (no progress is made).
	for(;;)
	{
		b3Transform xfA, xfB;
		sweepA.GetTransform(&xfA, t1);
		sweepB.GetTransform(&xfB, t1);

		// Get the distance between shapes. We can also use the results
		// to get a separating axis.
		distanceInput.transformA = xfA;
		distanceInput.transformB = xfB;
		b3DistanceOutput distanceOutput;
		b3Distance(&distanceOutput, &cache, &distanceInput);

		// If the shapes are overlapped, we give up on continuous collision.
		if (distanceOutput.distance <= 0.0f)
		{
			// Failure!
			output->state = b3TOIOutput::e_overlapped;
			output->t = 0.0f;
			break;
		}

		if (distanceOutput.distance < target + tolerance)
		{
			// Victory!
			output->state = b3TOIOutput::e_touching;
			output->t = t1;
			break;
		}

		// Initialize the separating axis.
		b3SeparationFunction fcn;
		fcn.Initialize(&cache, proxyA, sweepA, proxyB, sweepB, t1);
#if 0
		// Dump the curve seen by the root finder
		{
			const int32 N = 100;
			float32 dx = 1.0f / N;
			float32 xs[N+1];
			float32 fs[N+1];

			float32 x = 0.0f;

			for (int32 i = 0; i <= N; ++i)
			{
				sweepA.GetTransform(&xfA, x);
				sweepB.GetTransform(&xfB, x);
				float32 f = fcn.Evaluate(xfA, xfB) - target;

				printf("%g %g\n", x, f);

				xs[i] = x;
				fs[i] = f;

				x += dx;
			}
		}
#endif

		// Compute the TOI on the separating axis. We do this by successively
		// resolving the deepest point. This loop is bounded by the number of vertices.
		bool done = false;
		float32 t2 = tMax;
		int32 pushBackIter = 0;
		for (;;)
		{
			// Find the deepest point at t2. Store the witness point indices.
			int32 indexA, indexB;
			float32 s2 = fcn.FindMinSeparation(&indexA, &indexB, t2);

			// Is the final configuration separated?
			if (s2 > target + tolerance)
			{
				// Victory!
				output->state = b3TOIOutput::e_separated;
				output->t = tMax;
				done = true;
				break;
			}

			// Has the separation reached tolerance?
			if (s2 > target - tolerance)
			{
				// Advance the sweeps
				t1 = t2;
				break;
			}

			// Compute the initial separation of the witness points.
			float32 s1 = fcn.Evaluate(indexA, indexB, t1);

			// Check for initial overlap. This might happen if the root finder
			// runs out of iterations.
			if (s1 < target - tolerance)
			{
				output->state = b3TOIOutput::e_failed;
				output->t = t1;
				done = true;
				break;
			}

			// Check for touching
			if (s1 <= target + tolerance)
			{
				// Victory! t1 should hold the TOI (could be 0.0).
				output->state = b3TOIOutput::e_touching;
				output->t = t1;
				done = true;
				break;
			}

			// Compute 1D root of: f(x) - target = 0
			int32 rootIterCount = 0;
			float32 a1 = t1, a2 = t2;
			for (;;)
			{
				// Use a mix of the secant rule and bisection.
				float32 t;
				if (rootIterCount & 1)
				{
					// Secant rule to improve convergence.
					t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
				}
				else
				{
					// Bisection to guarantee progress.
					t = 0.5f * (a1 + a2);
				}

				++rootIterCount;
				++b3_toiRootIters;

				float32 s = fcn.Evaluate(indexA, indexB, t);

				if (b3Abs(s - target) < tolerance)
				{
					// t2 holds a tentative value for t1
					t2 = t;
					break;
				}

				// Ensure we continue to bracket the root.
				if (s > target)
				{
					a1 = t;
					s1 = s;
				}
				else
				{
					a2 = t;
					s2 = s;
				}
				
				if (rootIterCount == 50)
				{
					break;
				}
			}

			b3_toiMaxRootIters = b3Max(b3_toiMaxRootIters, rootIterCount);

			++pushBackIter;

			if (pushBackIter == b3_maxPolygonVertices)
			{
				break;
			}
		}

		++iter;
		++b3_toiIters;

		if (done)
		{
			break;
		}

		if (iter == k_maxIterations)
		{
			// Root finder got stuck. Semi-victory.
			output->state = b3TOIOutput::e_failed;
			output->t = t1;
			break;
		}
	}

	b3_toiMaxIters = b3Max(b3_toiMaxIters, iter);

	float32 time = timer.GetMilliseconds();
	b3_toiMaxTime = b3Max(b3_toiMaxTime, time);
	b3_toiTime += time;
}

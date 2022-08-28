#include "btHeightfieldTerrainShape.h"

// Equivalent to std::minmax({a, b, c}).
// Performs at most 3 comparisons.
static btHeightfieldTerrainShape::Range minmaxRange(btScalar a, btScalar b, btScalar c)
{
	if (a > b)
	{
		if (b > c)
			return btHeightfieldTerrainShape::Range(c, a);
		else if (a > c)
			return btHeightfieldTerrainShape::Range(b, a);
		else
			return btHeightfieldTerrainShape::Range(b, c);
	}
	else
	{
		if (a > c)
			return btHeightfieldTerrainShape::Range(c, b);
		else if (b > c)
			return btHeightfieldTerrainShape::Range(a, b);
		else
			return btHeightfieldTerrainShape::Range(a, c);
	}
}

/// given input vector, return quantized version
/**
  This routine is basically determining the gridpoint indices for a given
  input vector, answering the question: "which gridpoint is closest to the
  provided point?".

  "with clamp" means that we restrict the point to be in the heightfield's
  axis-aligned bounding box.
 */
void btHeightfieldTerrainShape::quantizeWithClamp(int* out, const btVector3& point, int /*isMax*/) const
{
	btVector3 clampedPoint(point);
	clampedPoint.setMax(m_localAabbMin);
	clampedPoint.setMin(m_localAabbMax);

	out[0] = getQuantized(clampedPoint.getX());
	out[1] = getQuantized(clampedPoint.getY());
	out[2] = getQuantized(clampedPoint.getZ());
}

/// process all triangles within the provided axis-aligned bounding box
/**
  basic algorithm:
    - convert input aabb to local coordinates (scale down and shift for local origin)
    - convert input aabb to a range of heightfield grid points (quantize)
    - iterate over all triangles in that subset of the grid
 */
void btHeightfieldTerrainShape::processAllTriangles(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax) const
{
	// scale down the input aabb's so they are in local (non-scaled) coordinates
	btVector3 localAabbMin = aabbMin * btVector3(1.f / m_localScaling[0], 1.f / m_localScaling[1], 1.f / m_localScaling[2]);
	btVector3 localAabbMax = aabbMax * btVector3(1.f / m_localScaling[0], 1.f / m_localScaling[1], 1.f / m_localScaling[2]);

	// account for local origin
	localAabbMin += m_localOrigin;
	localAabbMax += m_localOrigin;

	//quantize the aabbMin and aabbMax, and adjust the start/end ranges
	int quantizedAabbMin[3];
	int quantizedAabbMax[3];
	quantizeWithClamp(quantizedAabbMin, localAabbMin, 0);
	quantizeWithClamp(quantizedAabbMax, localAabbMax, 1);

	// expand the min/max quantized values
	// this is to catch the case where the input aabb falls between grid points!
	for (int i = 0; i < 3; ++i)
	{
		quantizedAabbMin[i]--;
		quantizedAabbMax[i]++;
	}

	int startX = 0;
	int endX = m_heightStickWidth - 1;
	int startJ = 0;
	int endJ = m_heightStickLength - 1;

	switch (m_upAxis)
	{
		case 0:
		{
			if (quantizedAabbMin[1] > startX)
				startX = quantizedAabbMin[1];
			if (quantizedAabbMax[1] < endX)
				endX = quantizedAabbMax[1];
			if (quantizedAabbMin[2] > startJ)
				startJ = quantizedAabbMin[2];
			if (quantizedAabbMax[2] < endJ)
				endJ = quantizedAabbMax[2];
			break;
		}
		case 1:
		{
			if (quantizedAabbMin[0] > startX)
				startX = quantizedAabbMin[0];
			if (quantizedAabbMax[0] < endX)
				endX = quantizedAabbMax[0];
			if (quantizedAabbMin[2] > startJ)
				startJ = quantizedAabbMin[2];
			if (quantizedAabbMax[2] < endJ)
				endJ = quantizedAabbMax[2];
			break;
		};
		case 2:
		{
			if (quantizedAabbMin[0] > startX)
				startX = quantizedAabbMin[0];
			if (quantizedAabbMax[0] < endX)
				endX = quantizedAabbMax[0];
			if (quantizedAabbMin[1] > startJ)
				startJ = quantizedAabbMin[1];
			if (quantizedAabbMax[1] < endJ)
				endJ = quantizedAabbMax[1];
			break;
		}
		default:
		{
			//need to get valid m_upAxis
			btAssert(0);
		}
	}

	// TODO If m_vboundsGrid is available, use it to determine if we really need to process this area
	
	const Range aabbUpRange(aabbMin[m_upAxis], aabbMax[m_upAxis]);
	for (int j = startJ; j < endJ; j++)
	{
		for (int x = startX; x < endX; x++)
		{
			btVector3 vertices[3];
			int indices[3] = { 0, 1, 2 };
			if (m_flipTriangleWinding)
			{
				indices[0] = 2;
				indices[2] = 0;
			}

			if (m_flipQuadEdges || (m_useDiamondSubdivision && !((j + x) & 1)) || (m_useZigzagSubdivision && !(j & 1)))
			{
				getVertex(x, j, vertices[indices[0]]);
				getVertex(x, j + 1, vertices[indices[1]]);
				getVertex(x + 1, j + 1, vertices[indices[2]]);

				// Skip triangle processing if the triangle is out-of-AABB.
				Range upRange = minmaxRange(vertices[0][m_upAxis], vertices[1][m_upAxis], vertices[2][m_upAxis]);

				if (upRange.overlaps(aabbUpRange))
					callback->processTriangle(vertices, 2 * x, j);
			
				// already set: getVertex(x, j, vertices[indices[0]])

				// equivalent to: getVertex(x + 1, j + 1, vertices[indices[1]]);
				vertices[indices[1]] = vertices[indices[2]];

				getVertex(x + 1, j, vertices[indices[2]]);
				upRange.min = btMin(upRange.min, vertices[indices[2]][m_upAxis]);
				upRange.max = btMax(upRange.max, vertices[indices[2]][m_upAxis]);

				if (upRange.overlaps(aabbUpRange))
					callback->processTriangle(vertices, 2 * x + 1, j);
			}
			else
			{
				getVertex(x, j, vertices[indices[0]]);
				getVertex(x, j + 1, vertices[indices[1]]);
				getVertex(x + 1, j, vertices[indices[2]]);

				// Skip triangle processing if the triangle is out-of-AABB.
				Range upRange = minmaxRange(vertices[0][m_upAxis], vertices[1][m_upAxis], vertices[2][m_upAxis]);

				if (upRange.overlaps(aabbUpRange))
					callback->processTriangle(vertices, 2 * x, j);

				// already set: getVertex(x, j + 1, vertices[indices[1]]);

				// equivalent to: getVertex(x + 1, j, vertices[indices[0]]);
				vertices[indices[0]] = vertices[indices[2]];

				getVertex(x + 1, j + 1, vertices[indices[2]]);
				upRange.min = btMin(upRange.min, vertices[indices[2]][m_upAxis]);
				upRange.max = btMax(upRange.max, vertices[indices[2]][m_upAxis]);

				if (upRange.overlaps(aabbUpRange))
					callback->processTriangle(vertices, 2 * x + 1, j);
			}
		}
	}
}

namespace
{
	struct GridRaycastState
	{
		int x;  // Next quad coords
		int z;
		int prev_x;  // Previous quad coords
		int prev_z;
		btScalar param;      // Exit param for previous quad
		btScalar prevParam;  // Enter param for previous quad
		btScalar maxDistanceFlat;
		btScalar maxDistance3d;
	};
}

// TODO Does it really need to take 3D vectors?
/// Iterates through a virtual 2D grid of unit-sized square cells,
/// and executes an action on each cell intersecting the given segment, ordered from begin to end.
/// Initially inspired by http://www.cse.yorku.ca/~amana/research/grid.pdf
template <typename Action_T>
void gridRaycast(Action_T& quadAction, const btVector3& beginPos, const btVector3& endPos, int indices[3])
{
	GridRaycastState rs;
	rs.maxDistance3d = beginPos.distance(endPos);
	if (rs.maxDistance3d < 0.0001)
	{
		// Consider the ray is too small to hit anything
		return;
	}
	

	btScalar rayDirectionFlatX = endPos[indices[0]] - beginPos[indices[0]];
	btScalar rayDirectionFlatZ = endPos[indices[2]] - beginPos[indices[2]];
	rs.maxDistanceFlat = btSqrt(rayDirectionFlatX * rayDirectionFlatX + rayDirectionFlatZ * rayDirectionFlatZ);

	if (rs.maxDistanceFlat < 0.0001)
	{
		// Consider the ray vertical
		rayDirectionFlatX = 0;
		rayDirectionFlatZ = 0;
	}
	else
	{
		rayDirectionFlatX /= rs.maxDistanceFlat;
		rayDirectionFlatZ /= rs.maxDistanceFlat;
	}

	const int xiStep = rayDirectionFlatX > 0 ? 1 : rayDirectionFlatX < 0 ? -1 : 0;
	const int ziStep = rayDirectionFlatZ > 0 ? 1 : rayDirectionFlatZ < 0 ? -1 : 0;

	const float infinite = 9999999;
	const btScalar paramDeltaX = xiStep != 0 ? 1.f / btFabs(rayDirectionFlatX) : infinite;
	const btScalar paramDeltaZ = ziStep != 0 ? 1.f / btFabs(rayDirectionFlatZ) : infinite;

	// pos = param * dir
	btScalar paramCrossX;  // At which value of `param` we will cross a x-axis lane?
	btScalar paramCrossZ;  // At which value of `param` we will cross a z-axis lane?

	// paramCrossX and paramCrossZ are initialized as being the first cross
	// X initialization
	if (xiStep != 0)
	{
		if (xiStep == 1)
		{
			paramCrossX = (ceil(beginPos[indices[0]]) - beginPos[indices[0]]) * paramDeltaX;
		}
		else
		{
			paramCrossX = (beginPos[indices[0]] - floor(beginPos[indices[0]])) * paramDeltaX;
		}
	}
	else
	{
		paramCrossX = infinite;  // Will never cross on X
	}

	// Z initialization
	if (ziStep != 0)
	{
		if (ziStep == 1)
		{
			paramCrossZ = (ceil(beginPos[indices[2]]) - beginPos[indices[2]]) * paramDeltaZ;
		}
		else
		{
			paramCrossZ = (beginPos[indices[2]] - floor(beginPos[indices[2]])) * paramDeltaZ;
		}
	}
	else
	{
		paramCrossZ = infinite;  // Will never cross on Z
	}

	rs.x = static_cast<int>(floor(beginPos[indices[0]]));
	rs.z = static_cast<int>(floor(beginPos[indices[2]]));

	// Workaround cases where the ray starts at an integer position
	if (paramCrossX == 0.0)
	{
		paramCrossX += paramDeltaX;
		// If going backwards, we should ignore the position we would get by the above flooring,
		// because the ray is not heading in that direction
		if (xiStep == -1)
		{
			rs.x -= 1;
		}
	}

	if (paramCrossZ == 0.0)
	{
		paramCrossZ += paramDeltaZ;
		if (ziStep == -1)
			rs.z -= 1;
	}

	rs.prev_x = rs.x;
	rs.prev_z = rs.z;
	rs.param = 0;

	while (true)
	{
		rs.prev_x = rs.x;
		rs.prev_z = rs.z;
		rs.prevParam = rs.param;

		if (paramCrossX < paramCrossZ)
		{
			// X lane
			rs.x += xiStep;
			// Assign before advancing the param,
			// to be in sync with the initialization step
			rs.param = paramCrossX;
			paramCrossX += paramDeltaX;
		}
		else
		{
			// Z lane
			rs.z += ziStep;
			rs.param = paramCrossZ;
			paramCrossZ += paramDeltaZ;
		}

		if (rs.param > rs.maxDistanceFlat)
		{
			rs.param = rs.maxDistanceFlat;
			quadAction(rs);
			break;
		}
		else
		{
			quadAction(rs);
		}
	}
}

struct ProcessTrianglesAction
{
	const btHeightfieldTerrainShape* shape;
	bool flipQuadEdges;
	bool useDiamondSubdivision;
	int width;
	int length;
	btTriangleCallback* callback;

	void exec(int x, int z) const
	{
		if (x < 0 || z < 0 || x >= width || z >= length)
		{
			return;
		}

		btVector3 vertices[3];

		// TODO Since this is for raycasts, we could greatly benefit from an early exit on the first hit

		// Check quad
		if (flipQuadEdges || (useDiamondSubdivision && (((z + x) & 1) > 0)))
		{
			// First triangle
			shape->getVertex(x, z, vertices[0]);
			shape->getVertex(x + 1, z, vertices[1]);
			shape->getVertex(x + 1, z + 1, vertices[2]);
			callback->processTriangle(vertices, x, z);

			// Second triangle
			shape->getVertex(x, z, vertices[0]);
			shape->getVertex(x + 1, z + 1, vertices[1]);
			shape->getVertex(x, z + 1, vertices[2]);
			callback->processTriangle(vertices, x, z);
		}
		else
		{
			// First triangle
			shape->getVertex(x, z, vertices[0]);
			shape->getVertex(x, z + 1, vertices[1]);
			shape->getVertex(x + 1, z, vertices[2]);
			callback->processTriangle(vertices, x, z);

			// Second triangle
			shape->getVertex(x + 1, z, vertices[0]);
			shape->getVertex(x, z + 1, vertices[1]);
			shape->getVertex(x + 1, z + 1, vertices[2]);
			callback->processTriangle(vertices, x, z);
		}
	}

	void operator()(const GridRaycastState& bs) const
	{
		exec(bs.prev_x, bs.prev_z);
	}
};

struct ProcessVBoundsAction
{
	const btAlignedObjectArray<btHeightfieldTerrainShape::Range>& vbounds;
	int width;
	int length;
	int chunkSize;

	btVector3 rayBegin;
	btVector3 rayEnd;
	btVector3 rayDir;

	int* m_indices;
	ProcessTrianglesAction processTriangles;

	ProcessVBoundsAction(const btAlignedObjectArray<btHeightfieldTerrainShape::Range>& bnd, int* indices)
		: vbounds(bnd),
		m_indices(indices)
	{
	}
	void operator()(const GridRaycastState& rs) const
	{
		int x = rs.prev_x;
		int z = rs.prev_z;

		if (x < 0 || z < 0 || x >= width || z >= length)
		{
			return;
		}

		const btHeightfieldTerrainShape::Range chunk = vbounds[x + z * width];

		btVector3 enterPos;
		btVector3 exitPos;

		if (rs.maxDistanceFlat > 0.0001)
		{
			btScalar flatTo3d = chunkSize * rs.maxDistance3d / rs.maxDistanceFlat;
			btScalar enterParam3d = rs.prevParam * flatTo3d;
			btScalar exitParam3d = rs.param * flatTo3d;
			enterPos = rayBegin + rayDir * enterParam3d;
			exitPos = rayBegin + rayDir * exitParam3d;

			// We did enter the flat projection of the AABB,
			// but we have to check if we intersect it on the vertical axis
			if (enterPos[1] > chunk.max && exitPos[m_indices[1]] > chunk.max)
			{
				return;
			}
			if (enterPos[1] < chunk.min && exitPos[m_indices[1]] < chunk.min)
			{
				return;
			}
		}
		else
		{
			// Consider the ray vertical
			// (though we shouldn't reach this often because there is an early check up-front)
			enterPos = rayBegin;
			exitPos = rayEnd;
		}

		gridRaycast(processTriangles, enterPos, exitPos, m_indices);
		// Note: it could be possible to have more than one grid at different levels,
		// to do this there would be a branch using a pointer to another ProcessVBoundsAction
	}
};

// TODO How do I interrupt the ray when there is a hit? `callback` does not return any result
/// Performs a raycast using a hierarchical Bresenham algorithm.
/// Does not allocate any memory by itself.
void btHeightfieldTerrainShape::performRaycast(btTriangleCallback* callback, const btVector3& raySource, const btVector3& rayTarget) const
{
	// Transform to cell-local
	btVector3 beginPos = raySource / m_localScaling;
	btVector3 endPos = rayTarget / m_localScaling;
	beginPos += m_localOrigin;
	endPos += m_localOrigin;

	ProcessTrianglesAction processTriangles;
	processTriangles.shape = this;
	processTriangles.flipQuadEdges = m_flipQuadEdges;
	processTriangles.useDiamondSubdivision = m_useDiamondSubdivision;
	processTriangles.callback = callback;
	processTriangles.width = m_heightStickWidth - 1;
	processTriangles.length = m_heightStickLength - 1;

	// TODO Transform vectors to account for m_upAxis
	int indices[3] = { 0, 1, 2 };
	if (m_upAxis == 2)
	{
		indices[1] = 2;
		indices[2] = 1;
	}
	int iBeginX = static_cast<int>(floor(beginPos[indices[0]]));
	int iBeginZ = static_cast<int>(floor(beginPos[indices[2]]));
	int iEndX = static_cast<int>(floor(endPos[indices[0]]));
	int iEndZ = static_cast<int>(floor(endPos[indices[2]]));

	if (iBeginX == iEndX && iBeginZ == iEndZ)
	{
		// The ray will never cross quads within the plane,
		// so directly process triangles within one quad
		// (typically, vertical rays should end up here)
		processTriangles.exec(iBeginX, iEndZ);
		return;
	}

	if (m_vboundsGrid.size()==0)
	{
		// Process all quads intersecting the flat projection of the ray
		gridRaycast(processTriangles, beginPos, endPos, &indices[0]);
	}
	else
	{
		btVector3 rayDiff = endPos - beginPos;
		btScalar flatDistance2 = rayDiff[indices[0]] * rayDiff[indices[0]] + rayDiff[indices[2]] * rayDiff[indices[2]];
		if (flatDistance2 < m_vboundsChunkSize * m_vboundsChunkSize)
		{
			// Don't use chunks, the ray is too short in the plane
			gridRaycast(processTriangles, beginPos, endPos, &indices[0]);
			return;
		}

		ProcessVBoundsAction processVBounds(m_vboundsGrid, &indices[0]);
		processVBounds.width = m_vboundsGridWidth;
		processVBounds.length = m_vboundsGridLength;
		processVBounds.rayBegin = beginPos;
		processVBounds.rayEnd = endPos;
		processVBounds.rayDir = rayDiff.normalized();
		processVBounds.processTriangles = processTriangles;
		processVBounds.chunkSize = m_vboundsChunkSize;
		// The ray is long, run raycast on a higher-level grid
		gridRaycast(processVBounds, beginPos / m_vboundsChunkSize, endPos / m_vboundsChunkSize, indices);
	}
}
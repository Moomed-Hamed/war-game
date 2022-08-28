#ifndef BT_HEIGHTFIELD_TERRAIN_SHAPE_H
#define BT_HEIGHTFIELD_TERRAIN_SHAPE_H

#include "btConcaveShape.h"

///btHeightfieldTerrainShape simulates a 2D heightfield terrain
/**
  The caller is responsible for maintaining the heightfield array; this
  class does not make a copy.

  The heightfield can be dynamic so long as the min/max height values
  capture the extremes (heights must always be in that range).

  The local origin of the heightfield is assumed to be the exact
  center (as determined by width and length and height, with each
  axis multiplied by the localScaling).

  \b NOTE: be careful with coordinates.  If you have a heightfield with a local
  min height of -100m, and a max height of +500m, you may be tempted to place it
  at the origin (0,0) and expect the heights in world coordinates to be
  -100 to +500 meters.
  Actually, the heights will be -300 to +300m, because bullet will re-center
  the heightfield based on its AABB (which is determined by the min/max
  heights).  So keep in mind that once you create a btHeightfieldTerrainShape
  object, the heights will be adjusted relative to the center of the AABB.  This
  is different to the behavior of many rendering engines, but is useful for
  physics engines.

  Most (but not all) rendering and heightfield libraries assume upAxis = 1
  (that is, the y-axis is "up").  This class allows any of the 3 coordinates
  to be "up".  Make sure your choice of axis is consistent with your rendering
  system.

  The heightfield heights are determined from the data type used for the
  heightfieldData array.  

   - unsigned char: height at a point is the uchar value at the
       grid point, multipled by heightScale.  uchar isn't recommended
       because of its inability to deal with negative values, and
       low resolution (8-bit).

   - short: height at a point is the short int value at that grid
       point, multipled by heightScale.

   - float or dobule: height at a point is the value at that grid point.

  Whatever the caller specifies as minHeight and maxHeight will be honored.
  The class will not inspect the heightfield to discover the actual minimum
  or maximum heights.  These values are used to determine the heightfield's
  axis-aligned bounding box, multiplied by localScaling.

  For usage and testing see the TerrainDemo.
 */

static inline int getQuantized(btScalar x)
{
	if (x < 0.0)
	{
		return (int)(x - 0.5);
	}
	return (int)(x + 0.5);
}

ATTRIBUTE_ALIGNED16(class)
btHeightfieldTerrainShape : public btConcaveShape
{
public:
	struct Range
	{
		Range() {}
		Range(btScalar min, btScalar max) : min(min), max(max) {}

		bool overlaps(const Range& other) const
		{
			return !(min > other.max || max < other.min);
		}

		btScalar min;
		btScalar max;
	};

protected:
	btVector3 m_localAabbMin;
	btVector3 m_localAabbMax;
	btVector3 m_localOrigin;

	///terrain data
	int m_heightStickWidth;
	int m_heightStickLength;
	btScalar m_minHeight;
	btScalar m_maxHeight;
	btScalar m_width;
	btScalar m_length;
	btScalar m_heightScale;
	union {
		const unsigned char* m_heightfieldDataUnsignedChar;
		const short* m_heightfieldDataShort;
		const float* m_heightfieldDataFloat;
		const double* m_heightfieldDataDouble;
		const void* m_heightfieldDataUnknown;
	};

	PHY_ScalarType m_heightDataType;
	bool m_flipQuadEdges;
	bool m_useDiamondSubdivision;
	bool m_useZigzagSubdivision;
	bool m_flipTriangleWinding;
	int m_upAxis;

	btVector3 m_localScaling;

	// Accelerator
	btAlignedObjectArray<Range> m_vboundsGrid;
	int m_vboundsGridWidth;
	int m_vboundsGridLength;
	int m_vboundsChunkSize;

	
	btScalar m_userValue3;

	struct btTriangleInfoMap* m_triangleInfoMap;

	/// This returns the "raw" (user's initial) height, not the actual height.
	/// The actual height needs to be adjusted to be relative to the center
	/// of the heightfield's AABB
	virtual btScalar getRawHeightFieldValue(int x, int y) const
	{
		btScalar val = 0.f;
		switch (m_heightDataType)
		{
			case PHY_FLOAT:
			{
				val = m_heightfieldDataFloat[(y * m_heightStickWidth) + x];
				break;
			}

			case PHY_DOUBLE:
			{
				val = m_heightfieldDataDouble[(y * m_heightStickWidth) + x];
				break;
			}

			case PHY_UCHAR:
			{
				unsigned char heightFieldValue = m_heightfieldDataUnsignedChar[(y * m_heightStickWidth) + x];
				val = heightFieldValue * m_heightScale;
				break;
			}

			case PHY_SHORT:
			{
				short hfValue = m_heightfieldDataShort[(y * m_heightStickWidth) + x];
				val = hfValue * m_heightScale;
				break;
			}

			default:
			{
				btAssert(!"Bad m_heightDataType");
			}
		}

		return val;
	}
	void quantizeWithClamp(int* out, const btVector3& point, int isMax) const;

	/// protected initialization
	/**
	  Handles the work of constructors so that public constructors can be
	  backwards-compatible without a lot of copy/paste.
	 */
	void initialize(
		int heightStickWidth, int heightStickLength, const void* heightfieldData,
		btScalar heightScale, btScalar minHeight, btScalar maxHeight, int upAxis,
		PHY_ScalarType hdt, bool flipQuadEdges)
	{
		// validation
		btAssert(heightStickWidth > 1);   // && "bad width");
		btAssert(heightStickLength > 1);  // && "bad length");
		btAssert(heightfieldData);        // && "null heightfield data");
		// btAssert(heightScale) -- do we care?  Trust caller here
		btAssert(minHeight <= maxHeight);                                                         // && "bad min/max height");
		btAssert(upAxis >= 0 && upAxis < 3);                                                      // && "bad upAxis--should be in range [0,2]");
		btAssert(hdt != PHY_UCHAR || hdt != PHY_FLOAT || hdt != PHY_DOUBLE || hdt != PHY_SHORT);  // && "Bad height data type enum");

		// initialize member variables
		m_shapeType = TERRAIN_SHAPE_PROXYTYPE;
		m_heightStickWidth = heightStickWidth;
		m_heightStickLength = heightStickLength;
		m_minHeight = minHeight;
		m_maxHeight = maxHeight;
		m_width = (btScalar)(heightStickWidth - 1);
		m_length = (btScalar)(heightStickLength - 1);
		m_heightScale = heightScale;
		m_heightfieldDataUnknown = heightfieldData;
		m_heightDataType = hdt;
		m_flipQuadEdges = flipQuadEdges;
		m_useDiamondSubdivision = false;
		m_useZigzagSubdivision = false;
		m_flipTriangleWinding = false;
		m_upAxis = upAxis;
		m_localScaling.setValue(btScalar(1.), btScalar(1.), btScalar(1.));

		m_vboundsChunkSize = 0;
		m_vboundsGridWidth = 0;
		m_vboundsGridLength = 0;

		// determine min/max axis-aligned bounding box (aabb) values
		switch (m_upAxis)
		{
			case 0:
			{
				m_localAabbMin.setValue(m_minHeight, 0, 0);
				m_localAabbMax.setValue(m_maxHeight, m_width, m_length);
				break;
			}
			case 1:
			{
				m_localAabbMin.setValue(0, m_minHeight, 0);
				m_localAabbMax.setValue(m_width, m_maxHeight, m_length);
				break;
			};
			case 2:
			{
				m_localAabbMin.setValue(0, 0, m_minHeight);
				m_localAabbMax.setValue(m_width, m_length, m_maxHeight);
				break;
			}
			default:
			{
				//need to get valid m_upAxis
				btAssert(0);  // && "Bad m_upAxis");
			}
		}

		// remember origin (defined as exact middle of aabb)
		m_localOrigin = btScalar(0.5) * (m_localAabbMin + m_localAabbMax);
	}

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	/// preferred constructors
	btHeightfieldTerrainShape(
		int heightStickWidth, int heightStickLength,
		const float* heightfieldData, btScalar minHeight, btScalar maxHeight,
		int upAxis, bool flipQuadEdges)
		: m_userValue3(0), m_triangleInfoMap(0)
	{
		initialize(heightStickWidth, heightStickLength, heightfieldData,
				   /*heightScale=*/1, minHeight, maxHeight, upAxis, PHY_FLOAT,
				   flipQuadEdges);
	}
	btHeightfieldTerrainShape(
		int heightStickWidth, int heightStickLength, const double* heightfieldData,
		btScalar minHeight, btScalar maxHeight, int upAxis, bool flipQuadEdges)
		: m_userValue3(0), m_triangleInfoMap(0)
	{
		initialize(heightStickWidth, heightStickLength, heightfieldData,
				   /*heightScale=*/1, minHeight, maxHeight, upAxis, PHY_DOUBLE,
				   flipQuadEdges);
	}
	btHeightfieldTerrainShape(
		int heightStickWidth, int heightStickLength, const short* heightfieldData, btScalar heightScale,
		btScalar minHeight, btScalar maxHeight, int upAxis, bool flipQuadEdges)
		: m_userValue3(0), m_triangleInfoMap(0)
	{
		initialize(heightStickWidth, heightStickLength, heightfieldData,
				   heightScale, minHeight, maxHeight, upAxis, PHY_SHORT,
				   flipQuadEdges);
	}
	btHeightfieldTerrainShape(
		int heightStickWidth, int heightStickLength, const unsigned char* heightfieldData, btScalar heightScale,
		btScalar minHeight, btScalar maxHeight, int upAxis, bool flipQuadEdges)
		: m_userValue3(0), m_triangleInfoMap(0)
	{
		initialize(heightStickWidth, heightStickLength, heightfieldData,
				   heightScale, minHeight, maxHeight, upAxis, PHY_UCHAR,
				   flipQuadEdges);
	}

	/// legacy constructor
	/**
	  This constructor supports a range of heightfield
	  data types, and allows for a non-zero minimum height value.
	  heightScale is needed for any integer-based heightfield data types.

	  This legacy constructor considers `PHY_FLOAT` to mean `btScalar`.
	  With `BT_USE_DOUBLE_PRECISION`, it will expect `heightfieldData`
	  to be double-precision.
	 */
	btHeightfieldTerrainShape(
		int heightStickWidth, int heightStickLength, const void* heightfieldData,
		btScalar heightScale, btScalar minHeight, btScalar maxHeight, int upAxis,
		PHY_ScalarType hdt, bool flipQuadEdges)
		: m_userValue3(0),
		  m_triangleInfoMap(0)
	{
		// legacy constructor: Assumes PHY_FLOAT means btScalar.
		initialize(heightStickWidth, heightStickLength, heightfieldData,
				   heightScale, minHeight, maxHeight, upAxis, hdt,
				   flipQuadEdges);
	}

	/// legacy constructor
	/**
	  The legacy constructor assumes the heightfield has a minimum height
	  of zero.  Only unsigned char or btScalar data are supported.  For legacy
	  compatibility reasons, heightScale is calculated as maxHeight / 65535 
	  (and is only used when useFloatData = false).
 	 */
	btHeightfieldTerrainShape(int heightStickWidth, int heightStickLength, const void* heightfieldData, btScalar maxHeight, int upAxis, bool useFloatData, bool flipQuadEdges)
		: m_userValue3(0),
		  m_triangleInfoMap(0)
	{
		// legacy constructor: support only btScalar or unsigned char data,
		// and min height is zero.
		PHY_ScalarType hdt = (useFloatData) ? PHY_FLOAT : PHY_UCHAR;
		btScalar minHeight = 0.0f;

		// previously, height = uchar * maxHeight / 65535.
		// So to preserve legacy behavior, heightScale = maxHeight / 65535
		btScalar heightScale = maxHeight / 65535;

		initialize(heightStickWidth, heightStickLength, heightfieldData,
				   heightScale, minHeight, maxHeight, upAxis, hdt,
				   flipQuadEdges);
	}

	virtual ~btHeightfieldTerrainShape()
	{
		clearAccelerator();
	}

	void setUseDiamondSubdivision(bool useDiamondSubdivision = true) { m_useDiamondSubdivision = useDiamondSubdivision; }

	///could help compatibility with Ogre heightfields. See https://code.google.com/p/bullet/issues/detail?id=625
	void setUseZigzagSubdivision(bool useZigzagSubdivision = true) { m_useZigzagSubdivision = useZigzagSubdivision; }

	void setFlipTriangleWinding(bool flipTriangleWinding)
	{
		m_flipTriangleWinding = flipTriangleWinding;
	}
	virtual void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
	{
		btVector3 halfExtents = (m_localAabbMax - m_localAabbMin) * m_localScaling * btScalar(0.5);

		btVector3 localOrigin(0, 0, 0);
		localOrigin[m_upAxis] = (m_minHeight + m_maxHeight) * btScalar(0.5);
		localOrigin *= m_localScaling;

		btMatrix3x3 abs_b = t.getBasis().absolute();
		btVector3 center = t.getOrigin();
		btVector3 extent = halfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);
		extent += btVector3(getMargin(), getMargin(), getMargin());

		aabbMin = center - extent;
		aabbMax = center + extent;
	}

	virtual void processAllTriangles(btTriangleCallback * callback, const btVector3& aabbMin, const btVector3& aabbMax) const;

	virtual void calculateLocalInertia(btScalar mass, btVector3 & inertia) const
	{
		//moving concave objects not supported
		inertia.setValue(btScalar(0.), btScalar(0.), btScalar(0.));
	}

	virtual void setLocalScaling(const btVector3& scaling)
	{
		m_localScaling = scaling;
	}

	virtual const btVector3& getLocalScaling() const
	{
		return m_localScaling;
	}

	/// this returns the vertex in bullet-local coordinates
	void getVertex(int x, int y, btVector3& vertex) const
	{
		btAssert(x >= 0);
		btAssert(y >= 0);
		btAssert(x < m_heightStickWidth);
		btAssert(y < m_heightStickLength);

		btScalar height = getRawHeightFieldValue(x, y);

		switch (m_upAxis)
		{
			case 0:
			{
				vertex.setValue(
					height - m_localOrigin.getX(),
					(-m_width / btScalar(2.0)) + x,
					(-m_length / btScalar(2.0)) + y);
				break;
			}
			case 1:
			{
				vertex.setValue(
					(-m_width / btScalar(2.0)) + x,
					height - m_localOrigin.getY(),
					(-m_length / btScalar(2.0)) + y);
				break;
			};
			case 2:
			{
				vertex.setValue(
					(-m_width / btScalar(2.0)) + x,
					(-m_length / btScalar(2.0)) + y,
					height - m_localOrigin.getZ());
				break;
			}
			default:
			{
				//need to get valid m_upAxis
				btAssert(0);
			}
		}

		vertex *= m_localScaling;
	}

	void performRaycast(btTriangleCallback * callback, const btVector3& raySource, const btVector3& rayTarget) const;

	/// Builds a grid data structure storing the min and max heights of the terrain in chunks.
	/// if chunkSize is zero, that accelerator is removed.
	/// If you modify the heights, you need to rebuild this accelerator.
	void buildAccelerator(int chunkSize = 16)
	{
		if (chunkSize <= 0)
		{
			clearAccelerator();
			return;
		}

		m_vboundsChunkSize = chunkSize;
		int nChunksX = m_heightStickWidth / chunkSize;
		int nChunksZ = m_heightStickLength / chunkSize;

		if (m_heightStickWidth % chunkSize > 0)
		{
			++nChunksX;  // In case terrain size isn't dividable by chunk size
		}
		if (m_heightStickLength % chunkSize > 0)
		{
			++nChunksZ;
		}

		if (m_vboundsGridWidth != nChunksX || m_vboundsGridLength != nChunksZ)
		{
			clearAccelerator();
			m_vboundsGridWidth = nChunksX;
			m_vboundsGridLength = nChunksZ;
		}

		if (nChunksX == 0 || nChunksZ == 0)
		{
			return;
		}

		// This data structure is only reallocated if the required size changed
		m_vboundsGrid.resize(nChunksX * nChunksZ);

		// Compute min and max height for all chunks
		for (int cz = 0; cz < nChunksZ; ++cz)
		{
			int z0 = cz * chunkSize;

			for (int cx = 0; cx < nChunksX; ++cx)
			{
				int x0 = cx * chunkSize;

				Range r;

				r.min = getRawHeightFieldValue(x0, z0);
				r.max = r.min;

				// Compute min and max height for this chunk.
				// We have to include one extra cell to account for neighbors.
				// Here is why:
				// Say we have a flat terrain, and a plateau that fits a chunk perfectly.
				//
				//   Left        Right
				// 0---0---0---1---1---1
				// |   |   |   |   |   |
				// 0---0---0---1---1---1
				// |   |   |   |   |   |
				// 0---0---0---1---1---1
				//           x
				//
				// If the AABB for the Left chunk did not share vertices with the Right,
				// then we would fail collision tests at x due to a gap.
				//
				for (int z = z0; z < z0 + chunkSize + 1; ++z)
				{
					if (z >= m_heightStickLength)
					{
						continue;
					}

					for (int x = x0; x < x0 + chunkSize + 1; ++x)
					{
						if (x >= m_heightStickWidth)
						{
							continue;
						}

						btScalar height = getRawHeightFieldValue(x, z);

						if (height < r.min)
						{
							r.min = height;
						}
						else if (height > r.max)
						{
							r.max = height;
						}
					}
				}

				m_vboundsGrid[cx + cz * nChunksX] = r;
			}
		}
	}
	void clearAccelerator()
	{
		m_vboundsGrid.clear();
	}

	int getUpAxis() const
	{
		return m_upAxis;
	}
	//debugging
	virtual const char* getName() const { return "HEIGHTFIELD"; }

	
	void setUserValue3(btScalar value)
	{
		m_userValue3 = value;
	}
	btScalar getUserValue3() const
	{
		return m_userValue3;
	}
	const struct btTriangleInfoMap* getTriangleInfoMap() const
	{
		return m_triangleInfoMap;
	}
	struct btTriangleInfoMap* getTriangleInfoMap()
	{
		return m_triangleInfoMap;
	}
	void setTriangleInfoMap(btTriangleInfoMap* map)
	{
		m_triangleInfoMap = map;
	}
	const unsigned char* getHeightfieldRawData() const
	{
		return m_heightfieldDataUnsignedChar;
	}
};

#endif  //BT_HEIGHTFIELD_TERRAIN_SHAPE_H

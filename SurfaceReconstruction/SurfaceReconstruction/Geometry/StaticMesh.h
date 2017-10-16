/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _STATIC_MESH_H_
#define _STATIC_MESH_H_

//#include "Math/Polybezier.h"
#include "Platform/DataTypes.h"
#include "SurfaceReconstruction/Geometry/Mesh.h"

// todo comments

namespace SurfaceReconstruction
{
	/// Represents object surfaces by means of triangles.
	class StaticMesh : public Mesh
	{
	public:
		StaticMesh(const Storage::Path &fileName);
		StaticMesh(const StaticMesh &other);
		StaticMesh(const std::vector<Math::Vector3> &colorBuffer, const std::vector<Math::Vector3> &normalBuffer, const std::vector<Math::Vector3> &positionBuffer,
			const std::vector<Real> &scaleBuffer, const std::vector<uint32> &indexBuffer);
		StaticMesh(const Math::Vector3 *colorBuffer, const Math::Vector3 *normalBuffer, const Math::Vector3 *positionBuffer,
			const Real *scaleBuffer, const uint32 *indexBuffer, const uint32 vertexCount, const uint32 indexCount);
		StaticMesh(const uint32 vertexCount, const uint32 indexCount);
		//StaticMesh(const Math::Polybezier<Math::Vector2> *surfaces, const uint32 surfaceCount, const Real height, bool vertexPadding);

		virtual ~StaticMesh();

		virtual void clear();
		
		virtual inline Math::Vector3 &getColor(const uint32 vertexIdx);
		virtual inline const Math::Vector3 &getColor(const uint32 vertexIdx) const;

		virtual inline Math::Vector3 *getColors();
		virtual inline const Math::Vector3 *getColors() const;
		
		virtual inline uint32 getIndexCount() const;
		virtual inline const uint32 *getIndices() const;
		
		virtual inline Math::Vector3 &getNormal(const uint32 vertexIdx);
		virtual inline const Math::Vector3 &getNormal(const uint32 vertexIdx) const;
		
		virtual inline Math::Vector3 *getNormals();
		virtual inline const Math::Vector3 *getNormals() const;
		
		virtual inline Math::Vector3 &getPosition(const uint32 vertexIdx);
		virtual inline const Math::Vector3 &getPosition(const uint32 vertexIdx) const;
		
		virtual inline Math::Vector3 *getPositions();
		virtual inline const Math::Vector3 *getPositions() const;
		
		virtual inline Real &getScale(const uint32 vertexIdx);
		virtual inline const Real &getScale(const uint32 vertexIdx) const;
		
		virtual inline Real *getScales();
		virtual inline const Real *getScales() const;

		virtual inline uint32 getVertexCount() const;		

	protected:
		virtual void allocateMemory(const uint32 vertexCount, const uint32 indexCount);

		virtual void setIndices(const uint32 *indices, const uint32 indexCount);

	private:
		StaticMesh();

		/** Assignment operator is forbidden. Don't use it.*/
		inline StaticMesh &operator =(const Mesh &rhs);

	private:
		Math::Vector3 *mColors;
		Math::Vector3 *mNormals;
		Math::Vector3 *mPositions;
		Real *mScales;
		uint32 *mIndices;

		uint32 mIndexCount;
		uint32 mVertexCount;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline & template function definitions   /////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline uint32 StaticMesh::getIndexCount() const
	{
		return mIndexCount;
	}

	inline const uint32 *StaticMesh::getIndices() const
	{
		return mIndices;
	}
		
	inline Math::Vector3 &StaticMesh::getColor(const uint32 vertexIdx)
	{
		return mColors[vertexIdx];
	}

	inline const Math::Vector3 &StaticMesh::getColor(const uint32 vertexIdx) const
	{
		return mColors[vertexIdx];
	}
	
	inline Math::Vector3 *StaticMesh::getColors()
	{
		return mColors;
	}

	inline const Math::Vector3 *StaticMesh::getColors() const
	{
		return mColors;
	}
		
	//inline uint32 *StaticMesh::getIndices()
	//{
	//	return mIndices;
	//}

	inline Math::Vector3 &StaticMesh::getNormal(const uint32 vertexIdx)
	{
		return mNormals[vertexIdx];
	}

	inline const Math::Vector3 &StaticMesh::getNormal(const uint32 vertexIdx) const
	{
		return mNormals[vertexIdx];
	}
		
	inline Math::Vector3 *StaticMesh::getNormals()
	{
		return mNormals;
	}

	inline const Math::Vector3 *StaticMesh::getNormals() const
	{
		return mNormals;
	}

	inline Math::Vector3 &StaticMesh::getPosition(const uint32 vertexIdx)
	{
		return mPositions[vertexIdx];
	}

	inline const Math::Vector3 &StaticMesh::getPosition(const uint32 vertexIdx) const
	{
		return mPositions[vertexIdx];
	}

	inline Math::Vector3 *StaticMesh::getPositions()
	{
		return mPositions;
	}

	inline const Math::Vector3 *StaticMesh::getPositions() const
	{
		return mPositions;
	}
		
	inline Real &StaticMesh::getScale(const uint32 vertexIdx)
	{
		return mScales[vertexIdx];
	}

	inline const Real &StaticMesh::getScale(const uint32 vertexIdx) const
	{
		return mScales[vertexIdx];
	}

	inline Real *StaticMesh::getScales()
	{
		return mScales;
	}

	inline const Real *StaticMesh::getScales() const
	{
		return mScales;
	}

	inline uint32 StaticMesh::getVertexCount() const
	{
		return mVertexCount;
	}
	
	inline StaticMesh &StaticMesh::operator =(const Mesh &rhs)
	{
		assert(false);
		return *this;
	}
}

#endif // _STATIC_MESH_H_

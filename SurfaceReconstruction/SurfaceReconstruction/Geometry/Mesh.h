/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _MESH_H_
#define _MESH_H_

#include <vector>
#include "Math/Vector2.h"
#include "Math/Vector3.h"
#include "Platform/Utilities/PlyFile.h"
#include "SurfaceReconstruction/Geometry/Vertex.h"

// todo comments

namespace SurfaceReconstruction
{
	/// Represents object surfaces by means of triangles.
	class Mesh
	{
	public:
		static void computeAABB(Math::Vector3 &min, Math::Vector3 &max, const Math::Vector3 *positions, const uint32 vertexCount);
		
		static Real computeAverageDistance(const Math::Vector3 &center, const Math::Vector3 *positions, const uint32 *neighborIndices, const uint32 indexCount,
			const uint32 skipIdx = Vertex::INVALID_IDX);

		static void computeNormalsOfTriangles(Math::Vector3 *normals, 
			const Math::Vector3 *positions, const uint32 *indices, const uint32 indexCount);

		static void computeNormalsWeightedByAngles(Math::Vector3 *normals,
			const Math::Vector3 *positions, const uint32 vertexCount, const uint32 *indices, const uint32 indexCount);
		
		static void computeNormalsWeightedByArea(Math::Vector3 *normals,
			const Math::Vector3 *positions, const uint32 vertexCount, const uint32 *indices, const uint32 indexCount);

		static inline Real computeLaplacianWeight(const Math::Vector3 &p0, const Math::Vector3 &p1);

		static void computeVertexScales(Real *scales, const std::vector<uint32> *vertexNeighbors, const Math::Vector3 *positions, const uint32 vertexCount);
		static Real computeVertexScale(const uint32 *neighbors, const uint32 *neighborOffsets, const Math::Vector3 *positions, const uint32 vertexIdx);
		static void computeVertexScales(Real *scales, const uint32 *neighbors, const uint32 *neighborOffsets, const Math::Vector3 *positions, const uint32 vertexCount);

		static void zeroNormals(Math::Vector3 *normals, const uint32 count);
		static void normalizeNormals(Math::Vector3 *normals, const uint32 count);

	public:
		virtual ~Mesh();

		void applyMovementField(const Math::Vector3 *vertexMovements);

		virtual void clear() = 0;
		
		inline void computeAABB(Math::Vector3 &min, Math::Vector3 &max) const;

		inline void computeNormalsOfTriangles(Math::Vector3 *normals) const;		

		virtual const Math::Vector3 &getColor(const uint32 vertexIdx) const = 0;
		virtual const Math::Vector3 *getColors() const = 0;
		
		virtual uint32 getIndexCount() const = 0;
		virtual const uint32 *getIndices() const = 0;
		
		virtual const Math::Vector3 &getNormal(const uint32 vertexIdx) const = 0;
		virtual const Math::Vector3 *getNormals() const = 0;

		virtual const Math::Vector3 &getPosition(const uint32 vertexIdx) const = 0;
		virtual const Math::Vector3 *getPositions() const = 0;
		
		virtual const Real &getScale(const uint32 vertexIdx) const = 0;
		virtual const Real *getScales() const = 0;

		inline const uint32 *getTriangle(const uint32 triangleIndex) const;

		inline uint32 getTriangleCount() const;

		virtual uint32 getVertexCount() const = 0;

		void saveToFile(const Storage::Path &fileName, const bool saveAsPly, const bool saveAsMesh) const;

		/** Non-shrinking smoothing operator based on the paper:\n
			A Signal Processing Approach To Fair Surface Design
			Authors: Gabriel Taubin,
			The paper is about a low-pass filter for meshes that has the transfer function: f(k) = (1 - smoothingMy * k)(1 - smoothingLambda * k)
			with smoothingLambda \in [0, 1] and smoothingMy < -smothingLambda.
			It should quickly decrease within the eigenvalue frequency period k \in [0, 2]. (Low frequencies k should be kept and high k \in [0, 2] attenuated.
			The transfer function can be defined by means of two parameters: \n
			- the pass-band eigenvalue k_bp (for which f(k_pb) = 1 holds since f(0) = 1 ^ smoothingMy < -smoothingLambda): it defines which low frequencies k_l \in [0, k_bp] are not attenuated \n
			- the smoothingLambda factor: it defines how where (at k_0) the transfer function reaches zero along the positive direction of k, k_0 = 1 / smoothingLambda
			See the paper for details.
		@param movementField For each vertex a movement is computed that is added to the old vertex position. This array temporarily stores all these movements and must have space for all vertices.
		@param weightField Is temporarily needed to store sums of vertex neighbor weights. Must have space for all vertices.
		@param passBandEigenvalue Defines the eigenvalue frequencies k \in [0, passBandEigenvalue] that are not attenuated. Values within [0.01, 0.1] produce good results. 
		@param smoothingLambda Defines the low pass filter transfer function f(k) for mesh smoothing. Must be positive. See the above description and paper for details.*/
		void smoothByTaubinOp(Math::Vector3 *movementField, Real *weightField, const Real passBandEigenvalue = 0.01f, const Real smoothingLambda = 0.6307f);
		void smoothByUmbrellaOp(Math::Vector3 *movementField, Real *weightField, const Real smoothingLambda);

	protected:
		virtual void allocateMemory(const uint32 vertexCount, const uint32 indexCount) = 0;

		void computeLaplacianVectorField(Math::Vector3 *movementField, Real *weightField);

		virtual Math::Vector3 &getColor(const uint32 vertexIdx)= 0;
		virtual Math::Vector3 *getColors() = 0;
		
		//virtual inline uint32 *getIndices() = 0;
		
		virtual Math::Vector3 &getNormal(const uint32 vertexIdx) = 0;
		virtual Math::Vector3 *getNormals() = 0;

		virtual Math::Vector3 &getPosition(const uint32 vertexIdx) = 0;
		virtual Math::Vector3 *getPositions() = 0;
		
		virtual Real &getScale(const uint32 vertexIdx) = 0;
		virtual Real *getScales() = 0;

		/** todo */
		virtual void loadFromFile(const Storage::Path &fileName);
		void loadFromMesh(const Storage::Path &fileName);
		void loadFromPly(const Storage::Path &fileName);
		void loadVertices(Utilities::PlyFile &plyFile, const Graphics::VerticesDescription &verticesFormat);
		
		void prepareLaplacianSmoothing(Math::Vector3 *movementField, Real *weightField, const uint32 vertexIdx0, const uint32 vertexIdx1) const;
		virtual void setIndices(const uint32 *indices, const uint32 indexCount) = 0;

	private:
		/** Assignment operator is forbidden. Don't use it.*/
		inline Mesh &operator =(const Mesh &rhs);

	public:
		static const uint32 FILE_VERSION;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline & template function definitions   /////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void Mesh::computeNormalsOfTriangles(Math::Vector3 *normals) const
	{
		Mesh::computeNormalsOfTriangles(normals, getPositions(), getIndices(), getIndexCount());
	}

	inline void Mesh::computeAABB(Math::Vector3 &min, Math::Vector3 &max) const
	{
		Mesh::computeAABB(min, max, getPositions(), getVertexCount());
	}

	inline Real Mesh::computeLaplacianWeight(const Math::Vector3 &p0, const Math::Vector3 &p1)
	{
		const Real distance = (p0 - p1).getLength();
		const Real weight = 1.0f / (1.0f + distance);

		return weight;
	}
	
	inline const uint32 *Mesh::getTriangle(const uint32 triangleIndex) const
	{
		return getIndices() + 3 * triangleIndex;
	}

	inline uint32 Mesh::getTriangleCount() const
	{
		return getIndexCount() / 3;
	}
	
	inline Mesh &Mesh::operator =(const Mesh &rhs)
	{
		assert(false);
		return *this;
	}
}

#endif // _MESH_H_

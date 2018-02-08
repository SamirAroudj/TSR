/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _FLEXIBLE_MESH_
#define _FLEXIBLE_MESH_

// todo comments

#include <cassert>
#include <map>
#include "Math/Vector3.h"
#include "Patterns/Subject.h"
#include "SurfaceReconstruction/Geometry/Edge.h"
#include "SurfaceReconstruction/Geometry/IFlexibleMeshObserver.h"
#include "SurfaceReconstruction/Geometry/IVertexChecker.h"
#include "SurfaceReconstruction/Geometry/Mesh.h"

namespace SurfaceReconstruction
{
	// forward declarations
	class StaticMesh;

	class EdgeConflict
	{
	public:
		EdgeConflict(const Edge &edge, const uint32 edgeIdx);

	public:
		std::vector<uint32> mTriangles;
		uint32 mEdgeIdx;
	};

	class FlexibleMesh : public Mesh, public Patterns::Subject<IFlexibleMeshObserver>
	{
	public:		
		static void addToVertexNeighbors(std::vector<uint32> &vertexNeighbood, const uint32 vertexIdx);

		static void computeOffsetsForFiltering(uint32 *vertexOffsets, uint32 *edgeOffsets, uint32 *triangleOffsets,
			const uint32 vertexCount, const uint32 edgeCount, const uint32 triangleCount,
			const std::vector<uint32> *verticesToEdges, const Edge *edges, const uint32 *indices,
			const IVertexChecker &vertexChecker);

		static void computeTriangleOffsets(uint32 *triangleOffsets, const uint32 *vertexOffsets, const uint32 *indices, const uint32 indexCount, const bool *additionalSkipTriangles);

		static void findBorderRing(std::vector<uint32> &remainingEdges, std::vector<uint32> &borderEdges);
		static void findBorderRings(std::vector<std::vector<uint32>> &holeBorderEdges, const uint32 *vertexOffsets = NULL);

		template <class T>
		static void filterData(std::vector<T> &buffer, const uint32 *offsets);

		template <class T>
		static void filterData(T *targetbuffer, const T *sourceBuffer,
			const uint32 *offsets, const uint32 sourceCount);

		template <class T>
		static void filterData(T *targetbuffer, const T *sourceBuffer,
			const uint32 *offsets, const uint32 sourceCount, const uint32 elementsPerBlock);

		static void filterTriangles(uint32 *targetIndices, const uint32 *sourceIndices, const uint32 *triangleOffsets, const uint32 sourceIndexCount, const uint32 *vertexOffsets);		

		static void findVertexNeighbors(std::vector<uint32> *vertexNeighbors, const uint32 *indices, const uint32 indexCount);
		
		static void prefixSum(uint32 *vertexOffsets, uint32 *edgeOffsets, uint32 *triangleOffsets,
			const uint32 vertexCount, const uint32 edgeCount, const uint32 triangleCount);

		static void updateVertexIndices(std::vector<std::vector<uint32>> &vertexSets, const uint32 *vertexOffsets);

	public:
		FlexibleMesh();
		FlexibleMesh(const Storage::Path &fileName);
		FlexibleMesh(const uint32 vertexCount, const uint32 indexCount);
		FlexibleMesh(const Mesh &mesh, IFlexibleMeshObserver *observer = NULL);
		FlexibleMesh(const FlexibleMesh &copy);
		virtual ~FlexibleMesh();
		
		/** todo */
		void addTriangle(const uint32 vertexIndices[3]);
		inline void addTriangle(const uint32 v0, const uint32 v1, const uint32 v2);

		/** todo 
		Only adds vertex data but no vertex index or other connectivity data. */
		void addVertex(const Math::Vector3 &color, const Math::Vector3 &normal, const Math::Vector3 &position, const Real scale);
		
		void checkEdges() const;
		virtual void clear();

		void clearAdjacencies();
		void clearOffsets(const uint32 vertexCount, const uint32 edgeCount, const uint32 triangleCount);

		inline void computeNormalsWeightedByAngles();
		inline void computeNormalsWeightedByArea();

		void computeScalesViaEdgeDistances();

		Math::Vector3 computeUmbrellaSmoothingMovement(const uint32 vertexIdx, const Real lambda) const;

		StaticMesh *createStaticMesh() const;

		void deleteGeometry(const uint32 *vertexOffsets, const uint32 *edgeOffsets, const uint32 *triangleOffsets);
		void deleteIsolatedGeometry(const uint32 triangleIslesMinSize);
		void deleteUnsupportedGeometry(const IVertexChecker &vertexChecker);

		void doSelfCheck() const;

		void fillHoles();
		void fillHoles(const std::vector<std::vector<uint32>> &holeBorderRings);

		/** Finds links of vertices to edges, edges &
		their links to triangles & triangle neighbors / all used connectivity information. */
		void findAdjacencies();

		bool getAdjacentTriangleNormals(Math::Vector3 &n0, Math::Vector3 &n1, const uint32 edgeVertexIdx0, const uint32 edgeVertexIdx1) const;
		
		Math::Vector3 getCenterOfNeighbors(const uint32 vertexIdx) const;

		virtual inline Math::Vector3 &getColor(const uint32 vertexIdx);
		virtual inline const Math::Vector3 &getColor(const uint32 vertexIdx) const;
		inline Math::Vector3 *getColors();
		virtual inline const Math::Vector3 *getColors() const;

		inline const std::vector<EdgeConflict> &getEdgeConflicts() const;

		inline const Edge &getEdge(const uint32 edgeIdx) const;	
		inline const Edge *getEdge(const uint32 edgeVertexIdx0, const uint32 edgeVertexIdx1) const;	
		inline uint32 getEdgeCapacity() const;
		inline uint32 getEdgeCount() const;

		/** Returns the mesh glboal index of the edge between vertexIdx0 and vertexIdx1 if it exists or -1 if there is no edge between them.  
		@param vertexIdx0 Set this to the index of one vertex of the edge for which you want to know its index.
		@param vertexIdx1 Set this to the index of the other vertex of the edge for which you want to know its index.
		@return Returns the mesh glboal index of the edge between vertexIdx0 and vertexIdx1 if it exists or -1 if there is no edge between them.  */
		uint32 getEdgeIndex(const uint32 vertexIdx0, const uint32 vertexIdx1) const;
		void getEdgeIndices(uint32 edgeIndices[3], const uint32 triangleIdx) const;
		inline const Edge *getEdges() const;
		void getEdges(uint32 edgeIndices[3], const uint32 triangleIdx) const;
		
		virtual inline uint32 getIndexCount() const;
		virtual inline const uint32 *getIndices() const;
		
		virtual inline Math::Vector3 &getNormal(const uint32 vertexIdx);
		virtual inline const Math::Vector3 &getNormal(const uint32 vertexIdx) const;
		inline Math::Vector3 *getNormals();
		virtual inline const Math::Vector3 *getNormals() const;
		
		virtual inline Math::Vector3 &getPosition(const uint32 vertexIdx);
		virtual inline const Math::Vector3 &getPosition(const uint32 vertexIdx) const;
		inline Math::Vector3 *getPositions();
		virtual inline const Math::Vector3 *getPositions() const;
		
		virtual inline Real &getScale(const uint32 vertexIdx);
		virtual inline const Real &getScale(const uint32 vertexIdx) const;
		inline Real *getScales();
		virtual inline const Real *getScales() const;

		void getTriangleNeighbors(uint32 neighbors[3], const uint32 triangleIdx) const;

		inline uint32 getVertexCapacity() const;
		virtual inline uint32 getVertexCount() const;

		inline const std::vector<uint32> *getVerticesToEdges() const;
		
		void getVertexNeighbors(std::vector<uint32> &neighbors, std::vector<uint32> &offsets) const;

		virtual void loadFromFile(const Storage::Path &fileName);

		void markDoomedElements(uint32 *globalDoomedVertices, uint32 *globalDoomedEdges, uint32 *globalDoomedTriangles,
			const uint32 *doomedVertices, const uint32 doomedVertexCount,
			const uint32 *doomedEdges, const uint32 doomedEdgeCount,
			const uint32 *doomedTriangles, const uint32 doomedTriangleCount);

		void mergeEdges(std::vector<uint32> &edgesWithNewIndices, const std::vector<uint32> &edges);
		void mergeEdge(uint32 *globalDoomedVertices, uint32 *globalDoomedEdges, uint32 *globalDoomedTriangles,
			const uint32 doomedEdgeIdx);
		void mergeEdgesWithoutFiltering(uint32 *globalDoomedVertices, uint32 *globalDoomedEdges, uint32 *globalDoomedTriangles,
			const std::vector<uint32> &mergingEdges);

		FlexibleMesh &operator =(const FlexibleMesh &rhs);

		void reserve(const uint32 vertexCount, const uint32 edgeCount, const uint32 indexCount);	

		void set(const Math::Vector3 &color, const Math::Vector3 &normal, const Math::Vector3 &position, const Real scale,
			const uint32 vertexIdx);

		inline void setColor(const Math::Vector3 &color, const uint32 vertexIdx);
		virtual void setIndices(const uint32 *newIndices, const uint32 indexCount);
		inline void setNormal(const Math::Vector3 &normal, const uint32 vertexIdx);
		inline void setPosition(const Math::Vector3 &position, const uint32 vertexIdx);
		inline void setScale(const Real scale, const uint32 vertexIdx);

		void subdivideEdges(const std::vector<uint32> &edges);
		void subdivideTriangles(std::vector<uint32> &doomedTriangles, std::vector<uint32> *possiblyDoomedTriangles = NULL);
		
		
		inline void smoothByUmbrellaOp(Math::Vector3 *movementField, Real *weightField, const Real smoothingLambda);
		void smoothByUmbrellaOp(Math::Vector3 *vectorField, const std::vector<uint32> &vertices, const Real lambda);

		inline void zeroColors();

		inline void zeroScales();

	protected:
		static void extendBorderRing(uint32 &size, std::vector<uint32> &border,
			const uint32 edge[2], const uint32 v0Idx, const uint32 v1Idx);
		
		static void removeDuplicatesInRings(std::vector<std::vector<uint32>> &holeBorders, const uint32 *vertexOffsets = NULL);

	protected:
		/** Used by findAdjacencies.
		@param triangleIdx todo
		@param edgeIdx Set it to 0, 1 or 2 to identify the local edge of triangle triangleIdx to be added.
		@see MeshRefiner::findAdjacencies */
		void addEdge(const uint32 triangleIdx, const uint32 edgeIdx);

		/** todo Is used by addEdge to resolve conflicts of edges regarding having more than two triangles per edge. */
		void addEdgeConflict(const uint32 globalEdgeIdx, const uint32 newTriangleIdx);
		
		void addNewEdgeSplitTriangles(uint32 oldTriangleIndices[3], const uint32 newIdx, const uint32 replacedIdx);

		void allocateMemory(const uint32 vertexCount, const uint32 indexCount);		

		//void checkConnectivity(const std::string &info,
		//	const uint32 *doomedVertices = NULL, const uint32 *doomedEdges = NULL, const uint32 *doomedTriangles = NULL) const;
		
		void connectHoleFillingRings(const uint32 *borderVertices, const uint32 borderEdgeCount,
			const uint32 oldVertexCount, const uint32 additionalVertexCount);

		uint32 createEdge(const uint32 v0, const uint32 v1, const uint32 triangleIdx);

		void createHoleFillingRingVertices(const uint32 *borderVertices, const uint32 borderEdgeCount,
			const uint32 newVertexCount, const Math::Vector3 &center);

		void createNewVerticesForSubdivision(const uint32 oldIndices[6], const uint32 newIndices[6]);
		
		void createSplitTriangles(const uint32 newVertexIndices[6], const uint32 oldVertexIndices[6],
			const uint32 triangleIdx, const uint32 oldNeighborTriangles[3]);

		void fillHole(const uint32 *borderVertices, const uint32 borderEdgeCount, const Math::Vector3 &center);
		void fillHole(const uint32 *borderVertices, const uint32 borderEdgeCount);

		void gatherIndicesForSplit(uint32 newVertexIndices[6], uint32 oldVertexIndices[6], uint32 oldNeighborTriangles[3],
			const uint32 triangleIdx) const;
		void gatherSplitTriangles(std::vector<uint32> *splitTriangles,
			const uint32 triangleIdx, const uint32 oldNeighborTriangles[3], const uint32 oldTriangleCount) const;
		
		void interpolateVertex(const uint32 targetIdx, const uint32 v0, const uint32 v1, const Real f0 = 0.5f);
		void interpolateEdgeSplitVertex(const uint32 targetIdx, const uint32 v0, const uint32 v1, const Real f0 = 0.5f);

		bool isInvalidEdgeMerge(const uint32 keptVertex, const uint32 keptEdges[2],
			const uint32 replacedVertex, const uint32 removedTriangles[2]) const;

		void moveEdges(const uint32 targetVertex, const uint32 sourceVertex);

		void onNewElements(const uint32 oldVertexCount, const uint32 oldEdgeCount, const uint32 oldTriangleCount);

		void reassignOldEdges(const uint32 oldIndices[3], const uint32 newIndices[3], const uint32 triangleIdx);	
		void removeVertexToEdgeLink(const uint32 vertexIdx, const uint32 globalEdgeIdx);

		void replaceCentralTriangle(const uint32 newIndices[3], const uint32 oldIndices[3], const uint32 triangleIdx);
		void replaceNeighborTriangle(const uint32 triangleIdx,
			const uint32 splitEdgeV0, const uint32 splitEdgeCenterVertex, const uint32 splitEdgeV1,
			const uint32 newCenterVertexIdx, const uint32 oldOppositeSplitVertex);

		void reserveFor4Splits(const uint32 splitTriangleCount);
		void reserveForEdgeSplits(const uint32 splitEdgeCount);
		void reserveForHoleFilling(const std::vector<std::vector<uint32>> &holeBorders);
		void resize(const uint32 newVertexCount, const uint32 newTriangleCount);

		void shrinkOldEdgeSplitTriangles(const uint32 newVertexIdx, const uint32 oldEdgeIdx,
			const uint32 oldEV[2], const uint32 oldOppoV, const uint32 oldTriangleIdx);

		void subdivideEdge(const uint32 edgeIdx);
		void subdivideTriangle(const uint32 triangleIdx, uint32 oldNeighborTriangles[3], std::vector<uint32> *splitResults = NULL);

		void updateEdgeData(const uint32 *edgeOffsets);
		void updateEdgeLinks(const uint32 *vertexOffsets, const uint32 *triangleOffsets);
		void updateEdges(const uint32 *vertexOffsets, const uint32 *edgeOffsets, const uint32 *triangleOffsets);
		
		void updateVertexData(const uint32 *vertexOffsets);
		void updateVertexToEdgeLinks(const uint32 *edgeOffsets);
		void updateVertices(const uint32 *vertexOffsets, const uint32 *edgeOffsets);
		
		void updateTriangleData(const uint32 *vertexOffsets, const uint32 *triangleOffsets);
		void updateTriangles(const uint32 *vertexOffsets, const uint32 *triangleOffsets);
		
		
		void updateLinksForEdgeMerge(uint32 doomedV[3], uint32 &doomedVCount, uint32 doomedE[3],
			const uint32 keptE[2], const uint32 keptV[2]);
		void updateLinksForEdgeMerge(const uint32 keptV[3], const uint32 keptEdges[2],
			const uint32 doomedV, const uint32 doomedT[2], const uint32 newDoomedEdges[3]);
		void updateTriangleIndicesForEdgeMerge(const uint32 keptVertex, const uint32 doomedVertex, const uint32 doomedT[2]);

	private:
		// vertices
		std::vector<std::vector<uint32>> mVerticesToEdges;	/// Stores the edge indices of all incoming / outgoing edges for each vertex.		
		std::vector<Math::Vector3> mColors;
		std::vector<Math::Vector3> mNormals;
		std::vector<Math::Vector3> mPositions;
		std::vector<Real> mScales;

		// edges
		std::vector<Edge> mEdges;

		// triangles
		std::vector<uint32> mIndices;

		// conflict data
		std::vector<EdgeConflict> mEdgeConflicts; /// maps global edge indices to the indices of the triangles which lead to a conflict for that edge

		// temporarily used offsets
		std::vector<uint32> mVertexOffsets;
		std::vector<uint32> mEdgeOffsets;
		std::vector<uint32> mTriangleOffsets;
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline & template function definitions   /////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	inline EdgeConflict::EdgeConflict(const Edge &edge, const uint32 edgeIdx) :
		mEdgeIdx(edgeIdx)
	{
		mTriangles.push_back(edge.getTriangleIndices()[0]);
		mTriangles.push_back(edge.getTriangleIndices()[1]);
	}

	inline void FlexibleMesh::addTriangle(const uint32 v0, const uint32 v1, const uint32 v2)
	{
		const uint32 indices[3] = { v0, v1, v2 };
		addTriangle(indices);	
	}
	
	inline void FlexibleMesh::computeNormalsWeightedByAngles()
	{
		Mesh::computeNormalsWeightedByAngles(getNormals(), getPositions(), getVertexCount(), getIndices(), getIndexCount());
	}

	inline void FlexibleMesh::computeNormalsWeightedByArea()
	{
		Mesh::computeNormalsWeightedByArea(getNormals(), getPositions(), getVertexCount(), getIndices(), getIndexCount());
	}

	template <class T>
	void FlexibleMesh::filterData(std::vector<T> &buffer, const uint32 *offsets)
	{
		const uint32 oldCount = (uint32) buffer.size();
		const uint32 newCount = oldCount - offsets[oldCount];
		
		std::vector<T> newBuffer(newCount);
		FlexibleMesh::filterData<T>(newBuffer.data(), buffer.data(), offsets, oldCount);
		buffer.swap(newBuffer);
	}

	template <class T>
	void FlexibleMesh::filterData(T *targetBuffer, const T *sourceBuffer, const uint32 *offsets, const uint32 sourceCount)
	{	
		#pragma omp parallel for
		for (int64 i = 0; i < sourceCount; ++i)
		{
			// discard this one?
			const uint32 oldIdx = (uint32) i;
			if (offsets[oldIdx] != offsets[oldIdx + 1])
				continue;

			// keep & copy it
			const uint32 newIdx = oldIdx - offsets[oldIdx];
			targetBuffer[newIdx] = sourceBuffer[oldIdx];
		}
	}

	template <class T>
	void FlexibleMesh::filterData(T *targetBuffer, const T *sourceBuffer, const uint32 *offsets, const uint32 sourceCount, const uint32 elementsPerBlock)
	{	
		#pragma omp parallel for
		for (int64 i = 0; i < sourceCount; ++i)
		{
			// discard this one?
			const uint32 oldIdx = (uint32) i;
			if (offsets[oldIdx] != offsets[oldIdx + 1])
				continue;

			// copy  data for a complete memory block of elementsPerBlock elements
			const uint32 newIdx = oldIdx - offsets[oldIdx];
			const T *const sourceStart = sourceBuffer + oldIdx * elementsPerBlock;
			T *const targetStart = targetBuffer + newIdx * elementsPerBlock;

			for (uint32 relativeIdx = 0; relativeIdx < elementsPerBlock; ++relativeIdx)
				targetStart[relativeIdx] = sourceStart[relativeIdx];
		}
	}

	inline Math::Vector3 &FlexibleMesh::getColor(const uint32 vertexIdx)
	{
		return mColors[vertexIdx];
	}

	inline const Math::Vector3 &FlexibleMesh::getColor(const uint32 vertexIdx) const
	{
		return mColors[vertexIdx];
	}

	inline Math::Vector3 *FlexibleMesh::getColors()
	{
		return mColors.data();	
	}

	inline const Math::Vector3 *FlexibleMesh::getColors() const
	{
		return mColors.data();	
	}
	
	inline const std::vector<EdgeConflict> &FlexibleMesh::getEdgeConflicts() const
	{
		return mEdgeConflicts;
	}
	
	inline const Edge *FlexibleMesh::getEdge(const uint32 edgeVertexIdx0, const uint32 edgeVertexIdx1) const
	{
		const uint32 edgeIdx = getEdgeIndex(edgeVertexIdx0, edgeVertexIdx1);
		if (Edge::INVALID_IDX == edgeIdx)
			return NULL;
		return mEdges.data() + edgeIdx;
	}
	
	inline const Edge &FlexibleMesh::getEdge(const uint32 edgeIdx) const
	{
		return mEdges[edgeIdx];
	}

	inline uint32 FlexibleMesh::getEdgeCapacity() const
	{
		return (uint32) mEdges.capacity();
	}

	inline uint32 FlexibleMesh::getEdgeCount() const
	{
		return (uint32) mEdges.size();
	}

	inline const Edge *FlexibleMesh::getEdges() const
	{
		return mEdges.data();
	}

	inline uint32 FlexibleMesh::getIndexCount() const
	{
		return (uint32) mIndices.size();
	}
	
	//inline uint32 *FlexibleMesh::getIndices()
	//{
	//	return mIndices.data();
	//}

	inline const uint32 *FlexibleMesh::getIndices() const
	{
		return mIndices.data();
	}
	
	inline Math::Vector3 &FlexibleMesh::getNormal(const uint32 vertexIdx)
	{
		return mNormals[vertexIdx];
	}
	
	inline const Math::Vector3 &FlexibleMesh::getNormal(const uint32 vertexIdx) const
	{
		return mNormals[vertexIdx];
	}

	inline Math::Vector3 *FlexibleMesh::getNormals()
	{
		return mNormals.data();
	}

	inline const Math::Vector3 *FlexibleMesh::getNormals() const
	{
		return mNormals.data();
	}
	
	inline Math::Vector3 &FlexibleMesh::getPosition(const uint32 vertexIdx)
	{
		return mPositions[vertexIdx];
	}
	
	inline const Math::Vector3 &FlexibleMesh::getPosition(const uint32 vertexIdx) const
	{
		return mPositions[vertexIdx];
	}

	inline Math::Vector3 *FlexibleMesh::getPositions()
	{
		return mPositions.data();
	}

	inline const Math::Vector3 *FlexibleMesh::getPositions() const
	{
		return mPositions.data();
	}

	inline Real &FlexibleMesh::getScale(const uint32 vertexIdx)
	{
		return mScales[vertexIdx];
	}

	inline const Real &FlexibleMesh::getScale(const uint32 vertexIdx) const
	{
		return mScales[vertexIdx];
	}

	inline Real *FlexibleMesh::getScales()
	{
		return mScales.data();
	}

	inline const Real *FlexibleMesh::getScales() const
	{
		return mScales.data();
	}
	
	inline uint32 FlexibleMesh::getVertexCapacity() const
	{
		return (uint32) mPositions.capacity();
	}

	inline uint32 FlexibleMesh::getVertexCount() const
	{
		return (uint32) mPositions.size();
	}
	
	inline const std::vector<uint32> *FlexibleMesh::getVerticesToEdges() const
	{
		return mVerticesToEdges.data();
	}

	inline void FlexibleMesh::setColor(const Math::Vector3 &color, const uint32 vertexIdx)
	{
		mColors[vertexIdx] = color;
	}

	inline void FlexibleMesh::setNormal(const Math::Vector3 &normal, const uint32 vertexIdx)
	{
		mNormals[vertexIdx] = normal;
	}

	inline void FlexibleMesh::setPosition(const Math::Vector3 &position, const uint32 vertexIdx)
	{
		mPositions[vertexIdx] = position;
	}

	inline void FlexibleMesh::setScale(const Real scale, const uint32 vertexIdx)
	{
		mScales[vertexIdx] = scale;
	}

	inline void FlexibleMesh::smoothByUmbrellaOp(Math::Vector3 *movementField, Real *weightField, const Real smoothingLambda)
	{
		Mesh::smoothByUmbrellaOp(movementField, weightField, smoothingLambda);
	}

	inline void FlexibleMesh::zeroColors()
	{	
		const int64 vertexCount = mColors.size();

		// set vertex colors to black / zero
		#pragma omp parallel for
		for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
			mColors[vertexIdx].set(0.0f, 0.0f, 0.0f);
	}

	inline void FlexibleMesh::zeroScales()
	{	
		const int64 vertexCount = mScales.size();

		// set vertex colors to black / zero
		#pragma omp parallel for
		for (int64 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
			mScales[vertexIdx] = 0.0f;
	}
}

#endif // _FLEXIBLE_MESH_
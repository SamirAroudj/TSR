/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#ifndef _MESH_RENDERER_H_
#define _MESH_RENDERER_H_

#include "Platform/Storage/Path.h"

namespace SurfaceReconstruction
{
	// forward declarations
	class Mesh;

	struct MeshOnGPU
	{
	public:
		enum IndexType
		{
			INDEX_VERTEX_ARRAY_OBJECT = 0,	/// index for OpenGL vertex array object identification
			INDEX_VERTEX_BUFFER = 1,		/// index for OpenGL GL_ARRAY_BUFFER identification
			INDEX_INDEX_BUFFER = 2,			/// index for OpenGL GL_ELEMENT_ARRAY_BUFFER identification
			INDEX_TYPE_COUNT = 3			/// number of available different index types
		};

	public:
		const Mesh *mMesh;
		uint32 mIDs[INDEX_TYPE_COUNT]; // identifiers for OpenGL data, see IndexType enumberation
	};

	struct PNCVertex
	{
	public:
		enum VertexAttributeIndices
		{
			VERTEX_ATTRIBUTE_INDEX_POSITION,
			VERTEX_ATTRIBUTE_INDEX_NORMAL,
			VERTEX_ATTRIBUTE_INDEX_COLOR,
			VERTEX_ATTRIBUTE_INDEX_COUNT
		};

	public:
		float mPosition[3];
		float mNormal[3];
		float mColor[3];
	};

	class MeshRenderer
	{
	public:
		enum Index
		{
			INDEX_PROGRAM,
			INDEX_VERTEX_SHADER,
			INDEX_FRAGMENT_SHADER,
			INDEX_TYPE_COUNT
		};

		enum Location
		{
			LOCATION_VP,
			LOCATION_TYPE_COUNT
		};

	public:
		MeshRenderer();
		~MeshRenderer();

		void clear();

		void deleteUploadedMesh(const Mesh &mesh);

		void renderUploadedMeshes() const;

		void uploadData(const Mesh &mesh);

	private:
		void checkProgramAndShaders() const;

		void createShader(const Storage::Path &fileName, const Index shaderType);
		PNCVertex *createVertexBuffer(const Mesh &mesh);
		void defineVertexFormat();
		void deleteUploadedMesh(const uint32 meshIdx);
		void setupShaders();

	public:
		static const Storage::Path SHADER_DIRECTORY;
		static const uint32 VERTEX_BUFFER_BINDING_INDEX;

	private:
		std::vector<MeshOnGPU> mMeshes;
		uint32 mPNCProgramIDs[INDEX_TYPE_COUNT];		/// program, vertex and fragment shader IDs
		uint32 mUniformLocations[LOCATION_TYPE_COUNT];	/// locations of uniform variables to be fed into vertex shaders
	};
}

#endif // _MESH_RENDERER_H_
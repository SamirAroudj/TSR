/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

#include <GL/glew.h>
#include <GL/gL.h>
#include "Graphics/Camera3D.h"
#include "Graphics/GraphicsManager.h"
#include "Math/Vector3.h"
#include "Platform/FailureHandling/GraphicsException.h"
#include "Platform/Storage/File.h"
#include "SurfaceReconstruction/Geometry/Mesh.h"
#include "SurfaceReconstruction/Rendering/MeshRenderer.h"

using namespace FailureHandling;
using namespace Graphics;
using namespace Math;
using namespace std;
using namespace Storage;
using namespace SurfaceReconstruction;

const Path MeshRenderer::SHADER_DIRECTORY = "Data/Shaders/";
const uint32 MeshRenderer::VERTEX_BUFFER_BINDING_INDEX = 0;

MeshRenderer::MeshRenderer()
{
	// initialize IDs with invalid values
	for (uint32 i = 0; i < LOCATION_TYPE_COUNT; ++i)
		mUniformLocations[i] = (uint32) - 1;
	for (uint32 i = 0; i < INDEX_TYPE_COUNT; ++i)
		mPNCProgramIDs[i] = (uint32) -1;

	setupShaders();
}

void MeshRenderer::setupShaders()
{
	// create program
	mPNCProgramIDs[INDEX_PROGRAM] = glCreateProgram();

	// load, compile & attach vertex & fragment shader
	const Path vertexShaderFileName = Path::appendChild(SHADER_DIRECTORY, "PNCVertexShader.glsl");
	createShader(vertexShaderFileName, INDEX_VERTEX_SHADER);
	
	const Path fragmentShaderFileName = Path::appendChild(SHADER_DIRECTORY, "PNCFragmentShader.glsl");
	createShader(fragmentShaderFileName, INDEX_FRAGMENT_SHADER);

	// specify mapping of vertex attributes to shader variables
	glBindAttribLocation(mPNCProgramIDs[INDEX_PROGRAM], PNCVertex::VERTEX_ATTRIBUTE_INDEX_POSITION, "inPosition");
	glBindAttribLocation(mPNCProgramIDs[INDEX_PROGRAM], PNCVertex::VERTEX_ATTRIBUTE_INDEX_NORMAL, "inNormal");
	glBindAttribLocation(mPNCProgramIDs[INDEX_PROGRAM], PNCVertex::VERTEX_ATTRIBUTE_INDEX_COLOR, "inColor");
	
	// link & use the program
	glLinkProgram(mPNCProgramIDs[INDEX_PROGRAM]);
	checkProgramAndShaders();

	// get uniform locations
	mUniformLocations[LOCATION_VP] = glGetUniformLocation(mPNCProgramIDs[INDEX_PROGRAM], "VP");
	glUseProgram(mPNCProgramIDs[INDEX_PROGRAM]);
}

void MeshRenderer::createShader(const Path &fileName, const Index shaderIdx)
{
	// load shader text from file
	string shader;
	File::loadTextFile(shader, fileName);
	
	const uint32 sourceCount = 1;
	const char *sources[sourceCount] = { shader.c_str() };
	const int32 sourceLengths[sourceCount] = { (int32) shader.length() };
	
	uint32 shaderType = (uint32) -1;
	switch (shaderIdx)
	{
		case INDEX_VERTEX_SHADER: shaderType = GL_VERTEX_SHADER; break;
		case INDEX_FRAGMENT_SHADER: shaderType = GL_FRAGMENT_SHADER; break;
		default: assert(false); throw Exception("Unimplemented case: createShader. Unsupported shader type.");
	}
	mPNCProgramIDs[shaderIdx] = glCreateShader(shaderType);

	glShaderSource(mPNCProgramIDs[shaderIdx], sourceCount, sources, sourceLengths);
	glCompileShader(mPNCProgramIDs[shaderIdx]);
	glAttachShader(mPNCProgramIDs[INDEX_PROGRAM], mPNCProgramIDs[shaderIdx]);
}

MeshRenderer::~MeshRenderer()
{
	clear();

	// free shaders etc.
	glDeleteShader(mPNCProgramIDs[INDEX_FRAGMENT_SHADER]);
	glDeleteShader(mPNCProgramIDs[INDEX_VERTEX_SHADER]);
	glDeleteProgram(mPNCProgramIDs[INDEX_PROGRAM]);
}

void MeshRenderer::clear()
{
	// free each mesh
	while (!mMeshes.empty())
		deleteUploadedMesh(0);

	// free access data structures for meshes on the GPU
	mMeshes.clear();
	mMeshes.shrink_to_fit();
}

void MeshRenderer::deleteUploadedMesh(const Mesh &mesh)
{
	// uploaded?
	const uint32 meshIndex = getIndex(&mesh);
	if ((uint32) -1 == meshIndex)
		throw Exception("Mesh on the GPU to be deleted was not uploaded to the GPU or freed already.");

	deleteUploadedMesh(meshIndex);
}

uint32 MeshRenderer::getIndex(const Mesh *mesh) const
{
	// try to find the mesh
	const uint32 meshCount = (uint32) mMeshes.size();
	for (uint32 meshIdx = 0; meshIdx < meshCount; ++meshIdx)
		if (mesh == mMeshes[meshIdx].mMesh)
			return meshIdx;
	
	// not found
	return -1;
}

void MeshRenderer::deleteUploadedMesh(const uint32 targetMeshIdx)
{
	// free the mesh on the GPU
	MeshOnGPU &meshOnGPU = mMeshes[targetMeshIdx];

	glDeleteVertexArrays(1, meshOnGPU.mIDs + MeshOnGPU::INDEX_VERTEX_ARRAY_OBJECT);
	glDeleteBuffers(2, meshOnGPU.mIDs + MeshOnGPU::INDEX_VERTEX_BUFFER);

	mMeshes[targetMeshIdx] = mMeshes.back();
	mMeshes.pop_back();
}

void MeshRenderer::renderUploadedMeshes() const
{
	// get camera
	const Camera3D *camera = Camera3D::getActiveCamera();
	if (!camera)
		return;
	
	// compute & upload matrix mapping from world space to clipping coordinates
	const Matrix4x4 &projection = camera->getProjectionMatrix();
	const Matrix4x4 &viewMatrix = camera->getViewMatrix();
	const Matrix4x4 VP = viewMatrix * projection;
	glUniformMatrix4rv(mUniformLocations[LOCATION_VP], 1, GL_FALSE, VP.getData());

	// render each mesh
	const uint32 meshCount = (uint32) mMeshes.size();
	for (uint32 meshIdx = 0; meshIdx < meshCount; ++meshIdx)
	{
		// draw mesh using the uploaded vertex & index buffers
		const MeshOnGPU &mesh = mMeshes[meshIdx];
		const uint32 indexCount = mesh.mMesh->getIndexCount();
		
		// define what data is drawn
		glBindVertexArray(mesh.mIDs[MeshOnGPU::INDEX_VERTEX_ARRAY_OBJECT]);
		glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
	}
}

void MeshRenderer::uploadData(const Mesh &mesh)
{
	// meta data for uploading
	const uint32 vertexCount = mesh.getVertexCount();
	const uint32 indexCount = mesh.getIndexCount();
	const uint64 vertexBytes = vertexCount * sizeof(PNCVertex);
	const uint64 indexBytes = indexCount * sizeof(uint32);
	const uint32 baseOffset = 0;
	const GLbitfield flags = 0;
	
	// MeshOnGPU = access point to data on the GPU
	mMeshes.resize(mMeshes.size() + 1);
	MeshOnGPU &meshOnGPU = mMeshes.back();
	meshOnGPU.mMesh = &mesh;

	// create data for vertex buffer in a format OpenGL can use
	PNCVertex *vertexBuffer = createVertexBuffer(mesh);

	// create and bind parent object (vertex array object: VAO)
	glCreateVertexArrays(1, meshOnGPU.mIDs + MeshOnGPU::INDEX_VERTEX_ARRAY_OBJECT);
	glBindVertexArray(meshOnGPU.mIDs[MeshOnGPU::INDEX_VERTEX_ARRAY_OBJECT]);
	defineVertexFormat();

	// create, bind and fill vertex & index buffer
	glGenBuffers(2, meshOnGPU.mIDs + MeshOnGPU::INDEX_VERTEX_BUFFER);

	// create & upload vertex bufferGLuint baseOffset = 0;
	glBindVertexBuffer(VERTEX_BUFFER_BINDING_INDEX, meshOnGPU.mIDs[MeshOnGPU::INDEX_VERTEX_BUFFER], baseOffset, sizeof(PNCVertex));
	glBindBuffer(GL_ARRAY_BUFFER, meshOnGPU.mIDs[MeshOnGPU::INDEX_VERTEX_BUFFER]);
	//glBufferData(GL_ARRAY_BUFFER, vertexBytes, vertexBuffer, GL_STATIC_DRAW);
	glBufferStorage(GL_ARRAY_BUFFER, vertexBytes, vertexBuffer, flags);

	// create & upload index buffer
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshOnGPU.mIDs[MeshOnGPU::INDEX_INDEX_BUFFER]);
	glBufferStorage(GL_ELEMENT_ARRAY_BUFFER, indexBytes, mesh.getIndices(), flags);
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexBytes, mesh.getIndices(), GL_STATIC_DRAW);

	// free resources
	delete [] vertexBuffer;
	vertexBuffer = NULL;
}

void MeshRenderer::defineVertexFormat()
{
	// define vertex format according to PNCVertex vertices
	// attribute 0: 3 x float for positions
	glEnableVertexAttribArray(PNCVertex::VERTEX_ATTRIBUTE_INDEX_POSITION);
	glVertexAttribFormat(PNCVertex::VERTEX_ATTRIBUTE_INDEX_POSITION, 3, GL_FLOAT, GL_FALSE, offsetof(PNCVertex, mPosition));
	glVertexAttribBinding(PNCVertex::VERTEX_ATTRIBUTE_INDEX_POSITION, VERTEX_BUFFER_BINDING_INDEX);
	
	// attribute 1: 3 x float for normals
	glEnableVertexAttribArray(PNCVertex::VERTEX_ATTRIBUTE_INDEX_NORMAL);
	glVertexAttribFormat(PNCVertex::VERTEX_ATTRIBUTE_INDEX_NORMAL, 3, GL_FLOAT, GL_FALSE, offsetof(PNCVertex, mNormal));
	glVertexAttribBinding(PNCVertex::VERTEX_ATTRIBUTE_INDEX_NORMAL, VERTEX_BUFFER_BINDING_INDEX);

	// attribute 2: 3 x float for colors
	glEnableVertexAttribArray(PNCVertex::VERTEX_ATTRIBUTE_INDEX_COLOR);
	glVertexAttribFormat(PNCVertex::VERTEX_ATTRIBUTE_INDEX_COLOR, 3, GL_FLOAT, GL_FALSE, offsetof(PNCVertex, mColor));
	glVertexAttribBinding(PNCVertex::VERTEX_ATTRIBUTE_INDEX_COLOR, VERTEX_BUFFER_BINDING_INDEX);
}

void MeshRenderer::checkProgramAndShaders() const
{
	const GraphicsManager &manager = GraphicsManager::getSingleton();
	
	// check shaders
	for (Index index = INDEX_VERTEX_SHADER; index < INDEX_TYPE_COUNT; index = (Index) (index + 1))
		manager.checkShader(mPNCProgramIDs[index]);

	// check program
	manager.checkProgram(mPNCProgramIDs[INDEX_PROGRAM]);
}

PNCVertex *MeshRenderer::createVertexBuffer(const Mesh &mesh)
{
	// allocate memory
	const uint32 vertexCount = mesh.getVertexCount();
	PNCVertex *vertexBuffer = new PNCVertex[vertexCount];

	// fill the vertex buffer in a format proper for OpenGL (float, interleaved vertex attributes)
	for (uint32 vertexIdx = 0; vertexIdx < vertexCount; ++vertexIdx)
	{
		// get data
		const Vector3 &sourcePos = mesh.getPosition(vertexIdx);
		const Vector3 &sourceNormal = mesh.getNormal(vertexIdx);
		const Vector3 &sourceColor = mesh.getColor(vertexIdx);

		// fill vertex for OpenGL
		PNCVertex &v = vertexBuffer[vertexIdx];

		// copy position
		v.mPosition[0] = static_cast<float>(sourcePos.x);
		v.mPosition[1] = static_cast<float>(sourcePos.y);
		v.mPosition[2] = static_cast<float>(sourcePos.z);

		// copy normal
		v.mNormal[0] = static_cast<float>(sourceNormal.x);
		v.mNormal[1] = static_cast<float>(sourceNormal.y);
		v.mNormal[2] = static_cast<float>(sourceNormal.z);

		// copy color
		v.mColor[0] = static_cast<float>(sourceColor.x);
		v.mColor[1] = static_cast<float>(sourceColor.y);
		v.mColor[2] = static_cast<float>(sourceColor.z);
	}

	return vertexBuffer;
}
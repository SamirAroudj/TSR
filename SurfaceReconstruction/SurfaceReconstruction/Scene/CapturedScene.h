/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _CAPTURED_SCENE_H_
#define _CAPTURED_SCENE_H_

#include <map>
#include <vector>
#include "Graphics/VerticesDescription.h"
#include "SurfaceReconstruction/Scene/Scene.h"
#include "Utilities/PlyFile.h"

namespace SurfaceReconstruction
{
	/// Represents a Scene object which is created from captured real world data.
	class CapturedScene : public Scene
	{
	public:
		/** Creates a Scene by loading data from given sources.
		@param metaFileName todo */
		CapturedScene(const Storage::Path &metaFileName, const std::vector<IReconstructorObserver *> &observers);

		/** Frees all scene data. */
		virtual ~CapturedScene();

	protected:
		/** Gets synthetic scene description from a parameters file.
		@param fileName Describes where to create the scene, what data to load, how to create the scene, etc.*/
		virtual bool getParameters(const Storage::Path &fileName);

	private:
		/** Copy constructor is forbidden. Don't use it. */
		inline CapturedScene(const CapturedScene &other);

		/** Assignment operator is forbidden. Don't use it.*/
		inline CapturedScene &operator =(const CapturedScene &rhs);

		void loadCameras(std::map<uint32, uint32> &oldToNewViewIDs);

		void loadImages(const std::vector<uint32> &imagesScales);

		/** todo */
		void loadMetaData(std::vector<Storage::Path> &plyCloudFileNames, std::vector<uint32> &imageScales,
			const Storage::Path &fileName);

		/** todo */
		void loadSampleCloud(const Storage::Path &plyCloudFileName);

		/** todo */
		void loadSampleClouds(const std::vector<Storage::Path> &plyCloudFileNames, const std::map<uint32, uint32> &oldToNewViewIDs);

		/** todo
		@return Returns the number of loaded samples. */
		uint32 loadSamples(Utilities::PlyFile &file, const Storage::Path &fileName, const Graphics::VerticesDescription &verticesFormat,
			const uint32 maxNewSampleCount);

		/** todo */
		void readSampleProperty(Utilities::PlyFile &file, const uint32 sampleIdx,
			const Graphics::ElementsDescription::TYPES type, const Graphics::VerticesDescription::SEMANTICS semantic);

	public:
		static const char *PARAMETER_NAME_IMAGE_SCALE;
		static const char *PARAMETER_NAME_INPUT_ORIENTATION;
		static const char *PARAMETER_NAME_INPUT_ORIGIN;
		static const char *PARAMETER_NAME_PLY_FILE;

	protected:
		Math::Matrix3x3 mInputOrientation;	/// all points are transformed into the coordinate system defined by mInputOrientation and mInputOrigin via x = R^t * (inputX - origin)
		Math::Vector3 mInputOrigin;			/// all points are transformed into the coordinate system defined by mInputOrientation and mInputOrigin via x = R^t * (inputX - origin)
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	inline CapturedScene::CapturedScene(const CapturedScene &other) :
		Scene(other.mRefinerObservers)
	{
		assert(false);
	}

	inline CapturedScene &CapturedScene::operator =(const CapturedScene &rhs)
	{
		assert(false);
		return *this;
	}
}

#endif // _CAPTURED_SCENE_H_



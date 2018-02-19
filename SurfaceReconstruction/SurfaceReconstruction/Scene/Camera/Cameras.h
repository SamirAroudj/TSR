/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _CAMERAS_H_
#define _CAMERAS_H_

#include <vector>
#include "Graphics/PinholeCamera.h"
#include "Platform/Storage/File.h"
#include "SurfaceReconstruction/Scene/Camera/CameraData.h"
#include "SurfaceReconstruction/Scene/Samples.h"

namespace SurfaceReconstruction
{
	// forward declarations
	class ColorImage;
	class DepthImage;

	/// Represents how the scene was captured by means of cameras. (Can also be used for laser scans and similar). Each has a position, rotation, focal length, etc.
	class Cameras
	{
	public:
		/** todo
		@param x
		@param y
		@param pixelToRayDir Transforms normalized homogenous pixel coordinates to  non-normalized ray directions. */
		static Math::Vector3 getRay(const uint32 x, const uint32 y, const Math::Matrix3x3 &pixelToRayDir);

	public:
		inline Cameras();
		inline Cameras(const Storage::Path &fileName);
		~Cameras();

		inline void addCamera(const CameraData &data);

		/** todo
		@param viewID Set this to a unique number to find / identify the view associated with this camera. See getViewID for more info.
		@param aspectRatio This is the ratio of width to height of the camera's image plane. Must be positive.
		todo */
		void addCamera(const uint32 viewID, const Math::Quaternion &orientation, const Math::Vector3 &position,
			 const Real focalLength, const Math::Vector2 &principalPoint, const Real imageAspectRatio, const Real distortion[2]);

		/** Creates a projective capture somewhere inside sceneAABB but outside the objects described by surfaces.
		@param viewID Set this to a unique number to find / identify the view associated with this camera. See getViewID for more info.
		@param sceneAABB Set this to an axis aligned bounding box (first min, then max) to restrict the camera placement area.
		@param minSampleDistance This is the minimum distance from the ground truth surfaces to the camera position which a Sample must have during its creation otherwise it is discarded.
		@param maxSampleDistance This is the maximum distance from the ground truth surfaces to the camera position which a Sample must have during its creation otherwise it is discarded.
		@param meanFOV Defines the mean field of view (radiance measure) which is used for random FOV creation.
		@param maxFOVDeviation Defines the maximally possible difference between the field of view of this object and meanFOV. (radiance measure)
		@param aspectRatio This is the ratio of width to height of the camera's image plane. Must be positive. */
		void addCamera(const uint32 viewID, const Math::Vector3 sceneAABB[2], const Real minSampleDistance, const Real maxSampleDistance,
			const Real meanFOV, const Real maxFOVDeviation, const Real aspectRatio);

		/** Removes all cameras. */
		void clear();

		/** Computes and returns the matrix which transforms coordinates relative to the world space coordinate system into the pixel coordinate system of this view and its current image. 
		The pixel coordinates are not normalized! You must perform the perspective division after the matrix has been applied.
		@param WSToPS Is set to a matrix which transforms world space coordiantes into non normalized pixel coordinates relative to the entered resolution.
		@param resolution Defines the width and height of the target pixel space (image size) in pixels.
		@param considerPixelCenterOffset Set this to true if you want to get coordinates refering to pixel centers instead of lower left corners.
			E.g., a point at the lower left camera frustum edge is mapped to the pixel coordinates (0.5, 0.5) if this is set to true (instead of (0, 0)). */
		void computeHWSToNNPS(Math::Matrix4x4 &WSToPS, const Utilities::ImgSize &resolution, const bool considerPixelCenterOffset, const uint32 cameraIdx) const;

		/** Returns true if there are no cameras otherwise false is returned. 
		@return Returns true if there are no cameras otherwise false is returned.*/
		inline bool empty() const;

		/** todo */
		inline Graphics::PinholeCamera &getCamera(const uint32 cameraIdx);

		/** todo */
		inline const Graphics::PinholeCamera &getCamera(const uint32 cameraIdx) const;

		/** Returns the color image which is associated to this view or NULL if there is none.
		@return Returns the color image which was taken from this view or NULL if there is none.*/
		const ColorImage *getColorImage(const std::string &tag, const uint32 &scale, const uint32 &cameraIdx) const;
		
		inline uint32 getCount() const;

		const DepthImage *getDepthImage(const std::string &tag, const uint32 &scale, const uint32 &cameraIdx) const;

		Math::Vector3 getPositionWS(const uint32 cameraIdx) const;

		/** Returns the central view direction the projective capture device had when this View object was created.
		@return Returns the central view direction the projective capture device had when this View object was created. */
		const Math::Vector3 getViewDirection(const uint32 cameraIdx) const;

		/** Returns this camera's view identifier.
			There is one view for each captured image of the current scene and a camera for each registered view / image.
			Images might not be registered which is why there can be fewer cameras than views.
		@param cameraIdx Identifies the camera. (Cameras are compactly stored and linked to views (thus also images) via their view IDs.
		@return Returns the view identifier for the camera cameraIdx. */
		inline uint32 getViewID(const uint32 cameraIdx) const;

		inline bool isValid(const uint32 cameraIdx) const;

		void loadFromFile(const Storage::Path &fileName);

		inline void popBack();

		inline void reserve(const uint32 cameraCount);
		void saveToFile(const Storage::Path &fileName) const;

		void shrinkToFit();

	public:
		static const uint32 FILE_VERSION;	/// Identifies the version of implementation of saving and loading of views for persistent storage.
		static const uint32 INVALID_ID;

	private:
		std::vector<Graphics::PinholeCamera> mCameras;	/// Defines where the camera is, its field of view and so on.
		std::vector<uint32> mViewIDs;					/// Unique identifiers to link view folders with cameras. (Cameras are stored compactly. A view folder can contain an unregistered view.)
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline Cameras::Cameras()
	{

	}

	inline Cameras::Cameras(const Storage::Path &fileName)
	{
		loadFromFile(fileName);
	}

	inline void Cameras::addCamera(const CameraData &data)
	{
		addCamera(data.mViewID,	data.mOrientation, data.mPosition,
			data.mFocalLength, data.mPrincipalPoint, data.mImageAspectRatio, data.mDistortion);
	}

	inline bool Cameras::empty() const
	{
		return mCameras.empty();
	}

	inline const Graphics::PinholeCamera &Cameras::getCamera(const uint32 cameraIdx) const
	{
		return mCameras[cameraIdx];
	}

	inline Graphics::PinholeCamera &Cameras::getCamera(const uint32 cameraIdx)
	{
		return mCameras[cameraIdx];
	}

	inline uint32 Cameras::getCount() const
	{
		return (uint32) mCameras.size();
	}

	inline Math::Vector3 Cameras::getPositionWS(const uint32 cameraIdx) const
	{
		const Graphics::PinholeCamera &camera = mCameras[cameraIdx];
		const Math::Vector4 &pHWS = camera.getPosition();
		return Math::Vector3(pHWS.x, pHWS.y, pHWS.z);
	}
	
	inline uint32 Cameras::getViewID(const uint32 cameraIdx) const
	{
		return mViewIDs[cameraIdx];
	}

	inline bool Cameras::isValid(const uint32 cameraIdx) const
	{
		return (cameraIdx < (uint32) mCameras.size());
	}
	
	inline void Cameras::popBack()
	{
		mCameras.pop_back();
		mViewIDs.pop_back();
	}

	inline void Cameras::reserve(const uint32 cameraCount)
	{
		mCameras.reserve(cameraCount);
		mViewIDs.reserve(cameraCount);
	}

}

#endif // _CAMERAS_H_

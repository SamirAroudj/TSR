/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _VIEW_H_
#define _VIEW_H_

#include <vector>
#include "Graphics/PinholeCamera.h"
#include "Platform/Storage/File.h"
#include "SurfaceReconstruction/Scene/Samples.h"

namespace SurfaceReconstruction
{
	// forward declarations
	class ColorImage;

	/// Represents a surface capture which is a set of samples which were measured by a single projective device at some view point.
	class View
	{
	public:
		/** todo */
		static std::string getIDString(const uint32 viewID);
		
		/** Returns the color image which is associated to the entered view viewID or NULL if there is none.
		@param viewID Identifies the view for which its color image is returned.
		@return returns the image which is associated to the entered view viewID or NULL if there is none.*/
		static const ColorImage *getColorImage(const uint32 viewID);

		/** todo
		@param x
		@param y
		@param pixelToRayDir Transforms normalized homogenous pixel coordinates to  non-normalized ray directions. */
		static Math::Vector3 getRay(const uint32 x, const uint32 y, const Math::Matrix3x3 &pixelToRayDir);

	public:
		/** Creates a projective view somewhere inside sceneAABB but outside the objects described by surfaces.
		@param ID Set this to a unique number to find / identify this particular view.
		@param sceneAABB Set this to an axis aligned bounding box (first min, then max) to restrict the camera placement area.
		@param minSampleDistance This is the minimum distance from the ground truth surfaces to the camera position which a Sample must have during its creation otherwise it is discarded.
		@param maxSampleDistance This is the maximum distance from the ground truth surfaces to the camera position which a Sample must have during its creation otherwise it is discarded.
		@param meanFOV Defines the mean field of view (radiance measure) which is used for random FOV creation.
		@param maxFOVDeviation Defines the maximally possible difference between the field of view of this object and meanFOV. (radiance measure)
		@param aspectRatio This is the ratio of width to height of the camera's image plane. Must be positive. */
		View(const uint32 ID, const Math::Vector3 sceneAABB[2], const Real minSampleDistance, const Real maxSampleDistance,
			const Real meanFOV, const Real maxFOVDeviation, const Real aspectRatio);

		/** todo
		@param ID Set this to a unique number to find / identify this particular view.
		@param aspectRatio This is the ratio of width to height of the camera's image plane. Must be positive.
		todo */
		View(const uint32 ID, const Math::Quaternion &orientation, const Math::Vector4 &position,
			 const Real focalLength, const Math::Vector2 &principlePoint, const Real aspectRatio);
		
		/** todo
		@param ID Set this to a unique number to find / identify this particular view.
		todo */
		View(const uint32 ID, Storage::File &file, const Storage::Path &fileName);

		/** Computes and returns the matrix which transforms coordinates relative to the world space coordinate system into the pixel coordinate system of this view and its current image. 
		The pixel coordinates are not normalized! You must perform the perspective division after the matrix has been applied.
		@param WSToPS Is set to a matrix which transforms world space coordiantes into non normalized pixel coordinates relative to the color image this->getColorImage().
		@param considerPixelCenterOffset Set this to true if you want to get coordinates refering to pixel centers instead of lower left corners.
			E.g., a point at the lower left camera frustum edge is mapped to the pixel coordinates (0.5, 0.5) if this is set to true (instead of (0, 0)). */
		void computeHWSToNNPS(Math::Matrix4x4 &WSToPS, const bool considerPixelCenterOffset) const;

		/** todo */
		inline const Graphics::PinholeCamera &getCamera() const;

		/** todo */
		inline Graphics::PinholeCamera &getCamera();

		/** Returns this view's hopefully unique identifier. 
		@return Returns this view's hopefully unique identifier. */
		inline uint32 getID() const;

		/** Returns the color image which is associated to this view or NULL if there is none.
		@return Returns the color image which was taken from this view or NULL if there is none.*/
		const ColorImage *getColorImage() const;

		Math::Vector3 getPositionWS() const;

		///** Transforms a point in world space into a point in the pixel / image space of this view.
		//@param positionWS Set this to the world space coordinates of the point you want to transform into pixel / image space.
		//@param resolution Set this to the image width and height in pixels.
		//@return Returns the pixel coordinates of positionWS projected into this view's image plane. */
		//const Math::Vector2 getPixelCoordinates(const Math::Vector3 positionWS, const uint32 resolution[2]) const;

		///** todo */
		//void getProjection(Math::Vector2 projectedTriangle[3],
		//	const Math::Vector3 &v0, const Math::Vector3 &v1, const Math::Vector3 &v2, const uint32 resolution[2]) const;

		/** Returns the central view direction the projective capture device had when this View object was created.
		@return Returns the central view direction the projective capture device had when this View object was created. */
		const Math::Vector3 getViewDirection() const;

		/** todo */
		void saveToFile(Storage::File &file) const;

	private:
		/** todo */
		void loadFromFile(Storage::File &file, const Storage::Path &fileName);

	public:
		static const uint32 INVALID_ID;

	private:
		Graphics::PinholeCamera mCamera;/// Defines where the view is, its field of view and so on.
		Math::Matrix4x4 mWSToPS;		/// Cached matrix which transforms world coordinates to pixel coordinates
		uint32 mID;						/// Unique identifier.
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline const Graphics::PinholeCamera &View::getCamera() const
	{
		return mCamera;
	}

	inline Graphics::PinholeCamera &View::getCamera()
	{
		return mCamera;
	}

	inline uint32 View::getID() const
	{
		return mID;
	}

	inline Math::Vector3 View::getPositionWS() const
	{
		const Math::Vector4 &pHWS = mCamera.getPosition();
		return Math::Vector3(pHWS.x, pHWS.y, pHWS.z);
	}
}

#endif // _VIEW_H_

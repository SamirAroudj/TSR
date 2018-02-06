/*
 * Copyright (C) 2017 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
#ifndef _SCENE_H_
#define _SCENE_H_

#include <string>
#include "SurfaceReconstruction/Scene/IReconstructorObserver.h"
#include "Patterns/Singleton.h"
#include "Platform/Storage/Path.h"
#include "Utilities/Size2.h"

namespace SurfaceReconstruction
{
	class FlexibleMesh;
	class FSSFRefiner;
	class Mesh;
	class Occupancy;
	class PCSRefiner;
	class Samples;
	class StaticMesh;
	class Tree;
	class View;

	/// Represents a complete scene with its object, camera, noise descriptions and so on.
	class Scene : public IReconstructorObserver, public Patterns::Singleton<Scene>
	{
	public:
		/** todo */
		static std::string getIDString(const uint32 identifier);

		/** Returns the name of the image without any parent folders, e.g., depth-L2.mvei. */
		static const std::string getLocalImageName(const std::string &tag, const uint32 scale, const bool colorImage);

		/** todo */
		inline static const Tree *getTree();
		
		/** todo
		@return Returned file name starts with views folder.
		(For example: "view_0000.mve/undist-L2.png") */
		inline static Storage::Path getRelativeImageFileName(const uint32 viewID, 
			const std::string &tag, const uint32 scale, const bool colorImage);
		
		/** todo
		@return Returned file name starts with views folder.
		(For example: "view_0000.mve/depth-L1.mvei") */
		static Storage::Path getRelativeImageFileName(const std::string &viewID, 
			const std::string &tag, const uint32 scale, const bool colorImage);

		inline static Storage::Path getRelativeViewFolder(const uint32 viewIdx); 

		/** Returns the name of the folder in the getViewsFolder for the view viewID.
		@param viewID Identifies the view for which the relative folder is returned.
		@return Returns a relative path pointing to the folder for the view viewID.
		@see getIDString(); */
		static Storage::Path getRelativeViewFolder(const std::string &viewId); 

	public:
		/** Loads a scene from file from already processed and created data, such as views, reordered samples and Tree object.
		@param rootFolder todo*/
		Scene(const Storage::Path &rootFolder, const Storage::Path &FSSFReconstruction,
			const std::vector<IReconstructorObserver *> &observers);

		/** Frees all scene data, such as views. */
		virtual ~Scene();

		/** Updates samples, free space and scene tree.
		@param sampleOffsets todo*/
		void eraseSamples(const bool *inliers, const bool saveResults);

		/** Creates a Tree object to spatially organize the scene and reorder its samples according to the tree structure.
		@return Returns true if reconstruction is succesful.*/
		bool reconstruct();

		/** todo Returns path to folder wheher results are written to files.
			The scene name is additionally appended to the folder. */
		Storage::Path getFileBeginning() const;

		inline const FSSFRefiner *getFSSFRefiner() const;

		/** Returns the ground truth mesh from which samples were created if available.
		@return Returns the ground truth mesh from which samples were created if available or NULL otherwise. */
		inline const StaticMesh *getGroundTruth() const;

		inline uint32 getMinIsleSize() const;

		const FlexibleMesh *getMostRefinedReconstruction() const;

		/** todo */
		inline const Occupancy *getOccupancy() const;

		inline const PCSRefiner *getPCSRefiner() const;

		/** todo */
		inline const FlexibleMesh *getReconstruction(ReconstructionType type) const;

		/** Returns the complete path to the folder in which results are saved.
		@return Returns the absolute path to the folder in which results are saved. */
		Storage::Path getResultsFolder() const;

		/** todo */
		inline const Samples &getSamples() const;

		/** Returns the number of exisitng View objects belonging to this Scene instance.
		@return Returns the number of exisitng View objects belonging to this Scene instance. */
		inline uint32 getViewCount() const;

		inline Storage::Path getViewFolder(const std::string &viewIDString) const;

		/** Provides access to existing View objects.
		@return Returns all exisiting View objects of this Scene object.*/
		inline std::vector<View *> &getViews();

		/** Returns the path pointing to the directory containing all views folders of this scene. 
		@return Returns the folder which contains all view folders view0000, view0001, ... etc.*/
		inline Storage::Path getViewsFolder() const;

		/** Provides access to existing View objects.
		@return Returns all exisiting View objects of this Scene object.*/
		inline const std::vector<View *> &getViews() const;
		
		/** todo */
		inline bool isValidView(const uint32 viewIdx) const;
		
		/** todo */
		void refine(const ReconstructionType type);

		/** todo */
		void saveReconstructionToFiles(const ReconstructionType type, const std::string &nameAppendix,
			const bool saveAsPly, const bool saveAsMesh) const;

		///** todo */
		//void saveToFile() const;
		
		/** todo */
		void swapSamples(const uint32 index0, const uint32 index1);
		
		/** todo 
			Scene takes over the responsibility of the mesh and deletes it if necessary. */
		void takeOverReconstruction(FlexibleMesh *mesh, const ReconstructionType type);

		virtual bool onNewReconstruction(FlexibleMesh *mesh,
			const uint32 iteration, const std::string &text, const ReconstructionType type, const bool responsible);

	protected:
		/** Creates an empty scene with a default name.*/
		Scene(const std::vector<IReconstructorObserver *> &observers);

		/** Frees allocated memory and sets pointers to NULL. */
		void clear();

		/** Creates an FSSRefiner with the latest reconstruction as input mesh. */
		void createFSSFRefiner();

		/** Gets synthetic scene description from a parameters file.
		@param fileName Describes where to create the scene, what data to load, how to create the scene, etc.*/
		virtual bool getParameters(const Storage::Path &fileName);

		/** todo */
		void loadFromFile(const Storage::Path &rootFolder, const Storage::Path &FSSFReconstruction);

		/** todo */
		void loadViewsFromFile(const Storage::Path &fileName);

		/** todo */
		void saveViewsToFile(const Storage::Path &fileName) const;

		/** todo */
		void setRootFolder(const Storage::Path &rootFolder);

		void takeReconstructionFromOccupancy();

	private:
		/** Copy constructor is forbidden. Don't use it. */
		inline Scene(const Scene &other);

		/** Assignment operator is forbidden. Don't use it.*/
		inline Scene &operator =(const Scene &rhs);

	public:
		/// parameter names for identification in configuration files
		static const char *PARAMETER_NAME_RELATIVE_CAMERAS_FILE;
		static const char *PARAMETER_NAME_SCENE_FOLDER;
		static const char *PARAMETER_NAME_TRIANGLE_ISLE_SIZE_MINIMUM;

		// default parameter values in case there are no valid configurations given by the user
		static const char *PARAMETER_VALUE_RELATIVE_CAMERAS_FILE;	/// default value for PARAMETER_RELATIVE_CAMERAS_FILE_NAME
		static const uint32 PARAMETER_VALUE_TRIANGLE_ISLE_SIZE_MINIMUM;
		static const uint32 PARAMETER_VALUE_REFINEMENT_VIA_PHOTOS_MESH_OUTPUT_FREQUENCY;	/// todo

		static const uint32 VIEWS_FILE_VERSION;								/// Identifies the version of implementation of saving and loading of views for persistent storage.

	protected:
		StaticMesh *mGroundTruth;															/// Contains ground truth surfaces to be reconstructed.
		FlexibleMesh *mReconstructions[IReconstructorObserver::RECONSTRUCTION_TYPE_COUNT];	/// Contains different reconstruction types, see SurfaceReconstruction::MeshObserver::ReconstructionType.

		FSSFRefiner *mFSSFRefiner;			/// Does variational surface mesh refinement starting with an initial mesh to get a reconstruction which "fits to" input images (high photo consistency score).
		Occupancy *mOccupancy;				/// Represents how empty and full the space is.
		PCSRefiner *mPCSRefiner;			/// Refines a coarse extracted crust to fit to the scene input samples.
		Samples *mSamples;					/// Represents all scene samples.
		Tree *mTree;						/// This tree partitions this scene spatially and defines sampling positions for the implicit function used for reconstruction.

		std::vector<IReconstructorObserver *> mRefinerObservers;	/// get updates from mesh refiners
		std::vector<View *> mViews;									/// These represent projective capturing views measuring surfaces and creating samples.
		
		Storage::Path mFolder;				/// Defines the root scene folder. Contains scene data and is the parent folder of the folders like views containing sub folders for each view with their images. 
		Storage::Path mRelativeCamerasFile;	/// Defines the file with the data of all cameras, file name is relative to folder.
		uint32 mMinIsleSize;		/// Triangle isles (isolated, connected sets of triangles) smaller than this are removed.
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///   inline function definitions   ////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	inline Scene::Scene(const Scene &other)
	{
		assert(false);
	}

	inline Scene &Scene::operator =(const Scene &rhs)
	{
		assert(false);
		return *this;
	}

	inline const FSSFRefiner *Scene::getFSSFRefiner() const
	{
		return mFSSFRefiner;
	}

	inline const StaticMesh *Scene::getGroundTruth() const
	{
		return mGroundTruth;
	}

	inline uint32 Scene::getMinIsleSize() const
	{
		return mMinIsleSize;
	}

	inline const Occupancy *Scene::getOccupancy() const
	{
		return mOccupancy;
	}

	inline const PCSRefiner *Scene::getPCSRefiner() const
	{
		return mPCSRefiner;
	}

	inline const FlexibleMesh *Scene::getReconstruction(ReconstructionType type) const
	{
		if (type >= RECONSTRUCTION_TYPE_COUNT)
			return NULL;

		return mReconstructions[type];
	}

	inline Storage::Path Scene::getRelativeImageFileName(const uint32 viewID,
		const std::string &tag, const uint32 scale, const bool colorImage)
	{
		return getRelativeImageFileName(getIDString(viewID), tag, scale, colorImage);
	}

	inline Storage::Path Scene::getRelativeViewFolder(const uint32 viewIdx)
	{
		return Scene::getRelativeViewFolder(getIDString(viewIdx));
	}

	inline const Samples &Scene::getSamples() const
	{
		return *mSamples;
	}

	inline const Tree *Scene::getTree()
	{
		return Scene::getSingleton().mTree;
	}

	inline uint32 Scene::getViewCount() const
	{
		return (uint32) mViews.size();
	}

	inline Storage::Path Scene::getViewFolder(const std::string &viewIDString) const
	{
		return Storage::Path::appendChild(getViewsFolder(), getRelativeViewFolder(viewIDString));
	}

	inline std::vector<View *> &Scene::getViews()
	{
		return mViews;
	}

	inline Storage::Path Scene::getViewsFolder() const
	{
		return Storage::Path::appendChild(mFolder, "views");
	}

	inline const std::vector<View *> &Scene::getViews() const
	{
		return mViews;
	}

	inline bool Scene::isValidView(const uint32 viewIdx) const
	{
		return (viewIdx < mViews.size() && mViews[viewIdx]);
	}
}

#endif // _SCENE_H_

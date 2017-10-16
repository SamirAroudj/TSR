# TSR
3D surface reconstruction software: from point clouds to triangle meshes.

# Required tools and libraries
- CMake (for easier building under windows and linux, get also ccmake for linux): https://cmake.org/
- BaseProject (3-Clause BSD license) and all of its dependencies (tinyxml, glew, glut, etc.): https://github.com/SamirAroudj/BaseProject/
- Embree (Apache 2.0 license): https://embree.github.io/index.html
 (TSR expects a build embreeBuildDir/targetConfiguration for each target configuration you want to create for TSR, e.g., embreeBuildDir/Debug, embreeBuildDir/Release) 

# Building for Linux
- Do not use relative paths with CMake!
- download the source code with "git clone https://github.com/SamirAroudj/TSR.git" into some root folder "SomePath/TSR"
- create a build folder within the SurfaceReconstruction directory: SomePath/TSR/SurfaceReconstruction/Build
- create a sub build folder for each target you want to build, e.g., SomePath/TSR/SurfaceReconstruction/Build/Debug or SomePath/TSR/SurfaceReconstruction/Build/Release
- within the sub folder Build/target run "ccmake ../.."
- run configure
- set BASE_PROJECT_DIR to the root directory of Base project (e.g. SomePath/Repos/BaseProject/)
- set the CMAKE_BUILD_TYPE to your target configuration (e.g.: Debug, Release)
- set EMBREE_BUILD_DIR to the directory containing the embree builds (e.g.: SomePath/Repos/embree-2.16.1/Build/,
TSR expects a corresponding build for each target configuration you want to have, e.g., Build/Release, Build/Debug)
- set EMBREE_INCLUDE_DIR to the include directory containing the embree headers (e.g.: embree-2.16.1/include/)
- run configure
- set BASE_BUILD_DIR to the build directory of BaseProject you want to use. This is usually what you have set for BaseProject via BASE_BUILD_OUTPUT_DIR, e.g.: SomePath/Repos/BaseProject/Build. The option simply allows choosing specialized builds of BaseProject for different target architectures while refering to only one BaseProject root directory via BASE_PROJECT_DIR.)
- set the remaining BASE_someName thingies as described in https://github.com/SamirAroudj/BaseProject/
- run configure
- have fun building the code using your favourite tool chain! (e.g., simply "make -j" within the build target configuration directory, QtCreator or ...)

# Running TSR for Surface Reconstruction
TSR requires a configuration file for the app itself that contains general settings, usually named "App.cfg". The app configuration file name is always the first command line argument. Further, it expects a description of the input data in a second file called "InputData.txt" or "InputDataSynthetic.txt" describing what you want to reconstruct. The input data description file name is the second command line argument if you start a reconstruction from scratch. If you have already started a reconstruction you can continue with saved intermediate results by providing the scene directory as second command line argument. That is, there are three alternatives to run TSR (WorkingDirectory refers to the folder SomePath\TSR\SurfaceReconstruction/WorkingDirectory):

First option, run TSR on synthetic data: <br/>
<code>TSR.exe WorkingDirectory\Data\App.cfg WorkingDirectory\Data\ExampleInputDataDescriptions\InputDataSynthetic.txt</code></br>
Note that synthetic input data descriptions must be named InputDataSynthetic.txt!

Second option, run TSR on captured data: <br/>
<code>TSR.exe WorkingDirectory\Data\App.cfg WorkingDirectory\Data\ExampleInputDataDescriptions\InputData.txt</code></br>
Note that captured input data descriptions must be named InputData.txt!

Third option, run TSR on already computed intermediate results in some scene data root folder which is "C:\Dev\Scenes\WedgeComplex" in this example:</br>
<code>TSR.exe WorkingDirectory\Data\App.cfg C:\Dev\Scenes\WedgeComplex</code></br>
Note that TSR OVERWRITES INTERMEDIATE RESULTS in the SceneRoot\Results folder!
You can optionally start TSR with a particular mesh refinement result given that you also have the previous intermediate results (scene tree etc.).
The start mesh is defined via the third and optional command line argument. The mesh must be completely closed (watertight). See this example:<br/>
<code>TSR.exe WorkingDirectory\Data\App.cfg C:\Dev\Scenes\WedgeComplex C:\Dev\Scenes\WedgeComplex\Results\0006FSSFabsoluteErrors.ply</code><br>

TSR accesses app configuration data and input data descriptions.
See WorkingDirectory\Data\ExampleInputDataDescriptions for example files showing how to configure TSR and define input data.

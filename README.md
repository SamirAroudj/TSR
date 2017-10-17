# TSR
3D surface reconstruction software: from point clouds to triangle meshes.
See the official project web site https://www.gcc.tu-darmstadt.de/home/proj/tsr/tsr.en.jsp for the accompanying paper, supplemental material and further information.

# Required tools and libraries
- CMake (for easier building under windows and linux, get also ccmake for linux): https://cmake.org/
- BaseProject (3-Clause BSD license) and all of its dependencies (tinyxml, glew, glut, etc.): https://github.com/SamirAroudj/BaseProject/
(TSR expects a build BaseProjectBuilds/TargetConfiguration for each target configuration you want to create for TSR, e.g.,
BaseProject/Build/Release, BaseProject/Build/Debug)
- Embree (Apache 2.0 license): https://embree.github.io/index.html
 (TSR expects a build EmbreeBuildDir/TargetConfiguration for each target configuration you want to create for TSR, e.g., EmbreeBuildDir/Debug, EmbreeBuildDir/Release) 

# Building for Windows
- Do not use relative paths with CMake!
- download the source code with "git clone https://github.com/SamirAroudj/TSR.git" into some root folder "SomePath/TSR"
- create a build folder within the SurfaceReconstruction directory: SomePath/TSR/SurfaceReconstruction/Build
- run CMake configure within SomePath/TSR/SurfaceReconstruction/Build/ as Build or binaries directory and SomePath/TSR/SurfaceReconstruction/ as source directory
- run configure
- set BASE_PROJECT_DIR to the root directory of Base project (e.g. SomePath/Repos/BaseProject/)
- set EMBREE_BUILD_DIR (e.g.: C:/Dev/3rdParty/embree-2.16.1/Build)
- set EMBREE_INCLUDE_DIR (e.g.: C:/Dev/3rdParty/embree-2.16.1/include)
- run configure
- set BASE_PROJECT_BUILD_DIR to the build directory of BaseProject you want to use. This is usually what you have set for BaseProject via BASE_BUILD_OUTPUT_DIR, e.g.: SomePath/BaseProject/Build. The option simply allows choosing specialized builds of BaseProject for different target architectures while refering to only one BaseProject root directory via BASE_PROJECT_DIR.)
- set the remaining BASE_someName thingies as described in https://github.com/SamirAroudj/BaseProject/
- run generate
- have fun building the code with a toolchain of your liking! (e.g. visual studio and SomePath/TSR/SurfaceReconstruction/Build/SurfaceReconstruction.sln)

# Building for Linux
- Do not use relative paths with CMake!
- download the source code with "git clone https://github.com/SamirAroudj/TSR.git" into some root folder "SomePath/TSR"
- create a build folder within the SurfaceReconstruction directory: SomePath/TSR/SurfaceReconstruction/Build
- create a sub build folder for each target you want to build, e.g., SomePath/TSR/SurfaceReconstruction/Build/Debug or SomePath/TSR/SurfaceReconstruction/Build/Release
- within the sub folder Build/Target run "ccmake ../.."
- run configure
- set BASE_PROJECT_DIR to the root directory of Base project (e.g. SomePath/Repos/BaseProject/)
- set the CMAKE_BUILD_TYPE to your target configuration (e.g.: Debug, Release)
- set EMBREE_BUILD_DIR to the directory containing the embree builds (e.g.: SomePath/Repos/embree-2.16.1/Build/,
TSR expects a corresponding build for each target configuration you want to have, e.g., Build/Release, Build/Debug)
- set EMBREE_INCLUDE_DIR to the include directory containing the embree headers (e.g.: embree-2.16.1/include/)
- run configure
- set BASE_PROJECT_BUILD_DIR to the build directory of BaseProject you want to use. This is usually what you have set for BaseProject via BASE_BUILD_OUTPUT_DIR, e.g.: SomePath/Repos/BaseProject/Build. The option simply allows choosing specialized builds of BaseProject for different target architectures while refering to only one BaseProject root directory via BASE_PROJECT_DIR.)
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

# Creating Input Data
TSR expects either a simple .ply-file as synthetic ground truth object or input point clouds with visibility information.

For captured real world data, the input point clouds and visibility information can be generated using the MVE fork https://github.com/SamirAroudj/mve:
First, TSR requires the capture sensor positions as input (e.g., camera projection centers during capturing). If you save a MVE scene using the previously mentioned fork then a Cameras.txt file is saved in the MVE scene root directory containing all the required sensor data in the right format ready to be used by TSR.</br>
Second, TSR expects point clouds with samples linked to sensors which have seen them during capturing.
These can also be produced by the MVE fork by using the corresponding command line arguments during reconstruction.
During depth map reconstruction with <code>dmrecon</code>, the local view selections must be saved. These are the identifiers of views or capture devices that are used for reconstructing specific samples and thus should see the samples and can be linked to them.
To additionally store the view identifiers for each reconstructed depth map pixel, run <code>dmrecon</code> with the command line option <code>-V</code> or <code>--keep-views</code>, e.g., <code>dmrecon -s4 -V "mve scene root directory"</code>. Further, when you create sample point clouds from views using <code>scene2pset</code>, you must enable outputting the view identifiers that link the sample points to the views in the Cameras.txt file. The view ID links for sample points are enabled by the command line argument <code>-V</code> or <code>--viewmap=ARG</code>, e.g., <code>-Vviews-L2</code>, <code>--viewmap=views-L1</code>. This produces point clouds with view identifier properties as links to the sensor data (usually "property int viewID0" up to "property int viewID4"). Note that TSR further requires sample normals, confidences and scale values which are enabled by the three command line arguments <code>-n -c -s</code>. Here is an example call for producing sample points for TSR: <code>scene2pset -iundist-L2 -ddepth-L2 -Vviews-L2 -ncs -w2,4 "mve scene root directory" pointCloudL2W2,4.ply
</code>.</br>
See <code>SomePath\TSR\SurfaceReconstruction\WorkingDirectory\Data\ExampleInputDataDescriptions\InputData.txt</code> for describing TSR input data when having produced the point clouds and sensor position files using the MVE fork as stated above.

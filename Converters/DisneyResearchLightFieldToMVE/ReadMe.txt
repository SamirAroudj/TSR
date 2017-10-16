Converter for datasets from:
https://www.disneyresearch.com/project/lightfields/

Paper
@article{kim2013scene,
  title={Scene reconstruction from high spatio-angular resolution light fields.},
  author={Kim, Changil and Zimmer, Henning and Pritch, Yael and Sorkine-Hornung, Alexander and Gross, Markus H},
  journal={ACM Trans. Graph.},
  volume={32},
  number={4},
  pages={73--1},
  year={2013}
}



Acquisition
All images provided here have been captured using a Canon EOS 5D Mark II DSLR camera and a Canon EF 50mm f/1.4 USM lens,
with an exception of the Couch dataset which was captured with a Canon EF 50mm f/1.2 L USM lens.
A Zaber T-LST1500D motorized linear stage was used to drive the camera to shooting positions.
The focus and aperture settings vary between datasets, but were kept identical within each dataset.
The camera focal length is 50mm and the sensor size is 36x24mm for all datasets. 
PTLens was used to radially undistort the captured images,
and Voodoo Camera Tracker was used to estimate the camera poses for rectification.
See the paper for detail.
Additionally, the exposure variation was compensated additively for the Bikes and Statue datasets.

File Formats
All images are provided in JPEG format.
Depth maps are provided in two different formats.
Those in .dmap files contain the dense disparity values d as defined in Equation 1 of the paper.
The first and the second 32-bit unsigned integers respectively indicate the width and the height of the depth map.
The rest of the file is an uncompressed sequence of width x height 32-bit floats in row-major order. Little-endian is assumed for all 4-byte words.
The depth values z can be obtained by applying Equation 1 given the camera focal length f in pixels and the camera separation b for each dataset.
Additionally, depth maps are stored as grayscale images in PNG format for easier visual inspection.
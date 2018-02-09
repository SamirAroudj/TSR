/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

 uniform mat4 VP;

 attribute vec4 inPosition;
 attribute vec3 inNormal;
 attribute vec3 inColor;

 varying vec3 outNormal;
 varying vec3 outColor;

 void main()
 {
	gl_Position = VP * inPosition;
	outNormal = inNormal;
	outColor = inColor;
 }
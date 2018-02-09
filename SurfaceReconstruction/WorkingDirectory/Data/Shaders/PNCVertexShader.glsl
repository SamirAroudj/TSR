/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

 uniform mat4 VP;

 attribute vec3 inPositionWS;
 attribute vec3 inNormalWS;
 attribute vec3 inColor;

 varying vec3 positionWS;
 varying vec3 normalWS;
 varying vec3 color;

 void main()
 {
	gl_Position = VP * vec4(inPositionWS, 1.0);

	// lerp for: position, normal and color
	positionWS = inPositionWS;
	normalWS = inNormalWS;
	color = inColor;
 }
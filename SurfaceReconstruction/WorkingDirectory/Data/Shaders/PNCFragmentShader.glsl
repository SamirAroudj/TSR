/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */

 varying vec3 inNormal;
 varying vec3 inColor;

 void main()
 {
	vec3 normal = normalize(inNormal);
	gl_FragColor = vec4(1.0, 1.0, 1.0, 1.0);//vec4(inColor.x, inColor.y, inColor.z, 1.0);
 }
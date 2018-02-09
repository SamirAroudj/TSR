/*
 * Copyright (C) 2018 by Author: Aroudj, Samir
 * TU Darmstadt - Graphics, Capture and Massively Parallel Computing
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD 3-Clause license. See the License.txt file for details.
 */
 varying vec3 positionWS;
 varying vec3 normalWS;
 varying vec3 color;

 void main()
 {
	vec3 inLight0WS = vec3(1.0, 10.0, 1.0);

	// ambient color
	vec3 ambientLight = vec3(0.2, 0.2, 0.2);
	vec3 ambientRefl = color * ambientLight;

	// normal and light direction
	vec3 normal = normalize(normalWS);
	vec3 toLightWS = inLight0WS - positionWS;
	toLightWS = normalize(toLightWS);
	
	// diffuse lighting
	float cosAlpha = dot(toLightWS, normal);
	float diffuseFactor = clamp(cosAlpha, 0.0, 1.0);
	vec3 diffuseRefl = color * cosAlpha;

	// output final color
	vec3 finalColor = ambientRefl + diffuseRefl;
	gl_FragColor = vec4(finalColor, 1.0);
 }
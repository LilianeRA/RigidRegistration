#version 120

attribute vec3 vertexPosition_modelspace;
uniform vec3 vertexColor;

// Output data ; will be interpolated for each fragment.
varying vec3 fragmentColor;
// Values that stay constant for the whole mesh.
uniform mat4 MVP;

uniform vec3 offset;
uniform float radius;


void main()
{
	gl_Position = MVP * vec4((vertexPosition_modelspace * radius)+offset, 1.0);
	//gl_Position = MVP * vec4((vertexPosition_modelspace * radius), 1.0);

	// The color of each vertex will be interpolated
	// to produce the color of each fragment
	fragmentColor = vertexColor;
}

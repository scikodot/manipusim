/*
	OpenTK stores matrices in a row-major order (i.e., every _n_ consecutive values form a row), 
	while GLSL employs a column-major order (i.e., every _n_ consecutive values form a column).

	This means if _model_ receives the following matrix, produced by, say, Matrix4.CreateTranslation(x, y, z):
	1  0  0  0
	0  1  0  0
	0  0  1  0
	x  y  z  1

	it gets treated by GLSL in a transposed way:
	1  0  0  x
	0  1  0  y
	0  0  1  z
	0  0  0  1

	As such, uniform matrices are multiplied from the left.
*/

#version 330 core

layout(location = 0) in vec3 vPosition;
layout(location = 1) in vec3 vNormal;
layout(location = 2) in vec4 vColor;
layout(location = 3) in vec2 vTexCoords;

out VertexData
{
	vec3 Position;
	vec3 Normal;
	vec4 Color;
	vec2 TexCoords;
} vOut;

// TODO: this doesn't work for some reason; 
// the uniforms are recognized in the code (both app's and shader's), but they do not receive the values;
// maybe in OpenTK uniform block's members cannot be set by UniformMatrix4() but only with, say, UniformBlockBinding()?
//uniform Matrices
//{
//	mat4 model;
//	mat4 view;
//	mat4 projection;
//} mats;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
	vec4 vOutPosHg = model * vec4(vPosition, 1.0);

	vOut.Position = vec3(vOutPosHg);
	vOut.Normal = transpose(inverse(mat3(model))) * vNormal;  // TODO: transposition and inversion should be done on CPU, not GPU!
	vOut.Color = vColor;
	vOut.TexCoords = vTexCoords;
	
	gl_Position = projection * view * vOutPosHg;
}
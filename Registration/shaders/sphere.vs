#version 460 core
layout(std140, binding = 0) uniform PerFrameData
{
	uniform mat4 MVP;
};
in vec3 vPos;
in vec3 vNormal;
out vec3 color;
out vec3 Normal;
out vec3 FragPos;
uniform float u_radius;
uniform vec3 u_offset;
uniform vec3 u_color;
void main()
{
	FragPos = (vPos * u_radius) + u_offset;
    color = u_color;
	Normal = vNormal;
    gl_Position = MVP * vec4(FragPos, 1.0);
}




/*#version 460 core
layout(std140, binding = 0) uniform PerFrameData
{
	uniform mat4 MVP;
};
in vec3 vPos;
out vec3 color;
uniform float u_radius;
uniform vec3 u_offset;
uniform vec3 u_color;
void main()
{
    gl_Position = MVP * vec4((vPos* u_radius) +u_offset, 1.0);
    color = u_color;
}*/
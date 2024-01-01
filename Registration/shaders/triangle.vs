#version 460 core
layout(std140, binding = 0) uniform PerFrameData
{
	uniform mat4 MVP;
};
in vec3 vPos;
in vec3 vColor;
in vec3 vNormal;
out vec3 color;
out vec3 Normal;
out vec3 FragPos;
void main()
{
    color = vColor;
	Normal = vNormal;
	FragPos = vPos;
    gl_Position = MVP * vec4(vPos, 1.0);
}




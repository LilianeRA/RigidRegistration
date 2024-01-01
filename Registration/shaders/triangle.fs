#version 460 core
in vec3 color;
in vec3 Normal;  
in vec3 FragPos;
out vec4 out_FragColor;
uniform vec3 u_lightPos;
uniform vec3 u_lightColor;
uniform float u_transparency;
void main()
{
    // ambient
    float ambientStrength = 0.5;
    vec3 ambient = ambientStrength * u_lightColor;
  	
    // diffuse 
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(u_lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * u_lightColor;
            
    vec3 result = (ambient + diffuse) * color;
    out_FragColor = vec4(result, u_transparency);
	
	
    //out_FragColor = vec4(color, 1.0);
}
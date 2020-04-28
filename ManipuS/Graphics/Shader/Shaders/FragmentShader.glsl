#version 330 core
out vec4 FragColor;

struct Material {
	// lighting maps
	sampler2D diffuseTex;
	sampler2D specularTex;

	// lighting colors
	vec3 ambientCol;
	vec3 diffuseCol;
	vec3 specularCol;
	
	float shininess;
};

struct DirLight {
	vec3 direction;

	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};

struct PointLight {
	vec3 position;

	float constant;
	float linear;
	float quadratic;

	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};

struct SpotLight {
	vec3 position;
	vec3 direction;
	float cutOff;
	float outerCutOff;

	float constant;
	float linear;
	float quadratic;

	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
};

#define NR_DIR_LIGHTS 3

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;
in vec4 Color;  // vertex color is not used directly in model shader, hence it is discarded

// material components depending on current mode (textures/colors)
vec3 ambientComp;
vec3 diffuseComp;
vec3 specularComp;

// uniforms
uniform vec3 viewPos;
uniform DirLight dirLight[NR_DIR_LIGHTS];
uniform Material material;

uniform bool useMaterial;  // switch between vertex/material colors (for vertex colors no light is applied)
uniform bool useTextures;  // switch between material textures/colors

// function prototypes
vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir);
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir);
vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir);

void main()
{
    if (useMaterial)
    {
        // properties
        vec3 norm = normalize(Normal);
        vec3 viewDir = normalize(viewPos - FragPos);

        // material components
        ambientComp = useTextures ? vec3(texture(material.diffuseTex, TexCoords)) : material.ambientCol;
        diffuseComp = useTextures ? vec3(texture(material.diffuseTex, TexCoords)) : material.diffuseCol;
        specularComp = useTextures ? vec3(texture(material.specularTex, TexCoords)) : material.specularCol;

        // directional light
        vec3 result = vec3(0.0);
        for (int i = 0; i < NR_DIR_LIGHTS; i++)
            result += CalcDirLight(dirLight[i], norm, viewDir);

        FragColor = vec4(result, 1.0);
    }
    else
    {
		// pass input color if the material is not presented
        FragColor = Color;
    }
}

// calculates the color when using a directional light.
vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir)
{
    vec3 lightDir = normalize(-light.direction);

    // diffuse shading
    float diff = max(dot(normal, lightDir), 0.0);

    // specular shading
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);

    // combine results
    vec3 ambient = light.ambient * ambientComp;
    vec3 diffuse = light.diffuse * diff * diffuseComp;
    vec3 specular = light.specular * spec * specularComp;
    return (ambient + diffuse + specular);
}

// calculates the color when using a point light.
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
    vec3 lightDir = normalize(light.position - fragPos);

    // diffuse shading
    float diff = max(dot(normal, lightDir), 0.0);

    // specular shading
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);

    // attenuation
    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));
    
    // combine results
    vec3 ambient = light.ambient * ambientComp;
    vec3 diffuse = light.diffuse * diff * diffuseComp;
    vec3 specular = light.specular * spec * specularComp;
    ambient *= attenuation;
    diffuse *= attenuation;
    specular *= attenuation;
    return (ambient + diffuse + specular);
}

// calculates the color when using a spot light.
vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
    vec3 lightDir = normalize(light.position - fragPos);

    // diffuse shading
    float diff = max(dot(normal, lightDir), 0.0);

    // specular shading
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);

    // attenuation
    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));
    
    // spotlight intensity
    float theta = dot(lightDir, normalize(-light.direction)); 
    float epsilon = light.cutOff - light.outerCutOff;
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);

    // combine results
    vec3 ambient = light.ambient * ambientComp;
    vec3 diffuse = light.diffuse * diff * diffuseComp;
    vec3 specular = light.specular * spec * specularComp;
    ambient *= attenuation * intensity;
    diffuse *= attenuation * intensity;
    specular *= attenuation * intensity;
    return (ambient + diffuse + specular);
}
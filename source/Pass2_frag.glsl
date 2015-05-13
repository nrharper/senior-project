#version 120
uniform vec3 ka;
uniform vec3 kd;
uniform vec3 ks;
uniform vec3 lightPos; // in camera space
uniform sampler2D shadowMap;
uniform sampler2D texture;

varying vec2 fragTexCoords;
varying vec3 fragPos;
varying vec3 fragNor;
varying vec4 shadowClip;

vec2 poissonDisk[] = vec2[](
	vec2( -0.94201624, -0.39906216 ),
	vec2( 0.94558609, -0.76890725 ),
	vec2( -0.094184101, -0.92938870 ),
	vec2( 0.34495938, 0.29387760 ),
	vec2( -0.91588581, 0.45771432 ),
	vec2( -0.81544232, -0.87912464 ),
	vec2( -0.38277543, 0.27676845 ),
	vec2( 0.97484398, 0.75648379 ),
	vec2( 0.44323325, -0.97511554 ),
	vec2( 0.53742981, -0.47373420 ),
	vec2( -0.26496911, -0.41893023 ),
	vec2( 0.79197514, 0.19090188 ),
	vec2( -0.24188840, 0.99706507 ),
	vec2( -0.81409955, 0.91437590 ),
	vec2( 0.19984126, 0.78641367 ),
	vec2( 0.14383161, -0.14100790 )
);

void main()
{
	//////////////////////////////////////////////////////
	// Blinn-Phong
	//////////////////////////////////////////////////////
	
	float distToCam = length(fragPos); // distance to camera
	float distToLight = length(lightPos - fragPos); // distance to light
	vec3 n = normalize(fragNor);
	vec3 l = (lightPos - fragPos) / distToLight;
	vec3 e = -fragPos / distToCam;
	vec3 h = normalize(l + e);
	float cd = max(0.0, dot(n, l));
	float cs = pow(max(0.0, dot(n, h)), 100.0);

	vec3 texColor = texture2D(texture, fragTexCoords).rgb;
	texColor += kd * 0.1;

	float attenuation = 1.0;// / (1.0 + 0.02 * distToLight + 0.02 * distToLight * distToLight);
	vec3 ambient = ka * attenuation;
	vec3 diffuse = cd * texColor * attenuation;
	vec3 specular = cs * ks * attenuation;
	
	//////////////////////////////////////////////////////
	// Shadowing
	//////////////////////////////////////////////////////
	
	// Convert from clip space to NDC
	vec4 shadowCoords = shadowClip;
	shadowCoords = shadowCoords / shadowCoords.w;
	// Go from [-1,1] to [0,1]
	shadowCoords.xyz = 0.5 * shadowCoords.xyz + 0.5;
	
	// Sample the shadow map N times
	float bias = 0.0000001;
	//float bias = 0.00005 * tan(acos(dot(n, l)));
	//bias = clamp(bias, 0.0, 0.01);
	float blur = 0.001;
	float visibility = 1.0;
	if(shadowCoords.w > 0.0 &&
	   shadowCoords.x > 0.0 && shadowCoords.x < 1.0 &&
	   shadowCoords.y > 0.0 && shadowCoords.y < 1.0 &&
	   shadowCoords.z > 0.0 && shadowCoords.z < 1.0) {
		for(int i = 0; i < 4; i++) {
			// Get the distance to light stored in the shadow map.
			// This value is in NDC between 0 and 1.
			// (Note: distToLight we computed for Blinn-Phong is in camera space.)
			float distToLightStored = texture2D(shadowMap, shadowCoords.xy + poissonDisk[i]*blur).z;
			if(distToLightStored < shadowCoords.z + bias) {
				visibility -= 0.20;
			}
		}
	}
	
	// Final color
	gl_FragColor.rgb = ambient + visibility * (diffuse + specular);
}

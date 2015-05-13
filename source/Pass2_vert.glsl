#version 120

attribute vec4 vertPos;
attribute vec3 vertNor;
attribute vec2 vertTex;
uniform mat4 P;
uniform mat4 MV;
uniform mat4 lightMVP;
uniform mat3 Tscale;

varying vec2 fragTexCoords;
varying vec3 fragPos;
varying vec3 fragNor;
varying vec4 shadowClip;

void main()
{
   fragTexCoords = (vec3(vertTex, 1.0) * Tscale).xy;
	// vertex position in camera space
	vec4 vertPos_cam = MV * vertPos;
	fragPos = vertPos_cam.xyz;
	// vertex position in clip space
	gl_Position = P * vertPos_cam;
	// vertex normal in camera space
	fragNor = normalize((MV * vec4(vertNor, 0.0)).xyz);
	// vertex position in light clip place
	shadowClip = lightMVP * vertPos;
}

attribute vec4 vertPos; // in object space
attribute vec3 vertNor; // in object space
attribute vec2 vertTex;
uniform mat4 P;
uniform mat4 MV;
uniform mat3 T1;

varying vec2 fragTexCoords;
varying vec3 fragPos;
varying vec3 norm;

void main()
{
   fragTexCoords = (vec3(vertTex, 1.0) * T1).xy;
   norm = (MV * vec4(vertNor, 0.0)).xyz;
   fragPos = (MV * vertPos).xyz;
	gl_Position = P * MV * vertPos;
}

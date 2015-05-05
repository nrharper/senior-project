uniform vec3 ka;
uniform vec3 kd;
uniform vec3 ks;
uniform float s;
uniform float intensity;
uniform vec3 lightPos;
uniform sampler2D texture;

varying vec2 fragTexCoords;
varying vec3 fragPos;
varying vec3 norm;

void main()
{
	vec3 n = normalize(norm);
   vec3 l = normalize(lightPos - fragPos);
   vec3 e = normalize(vec3(0.0f, 0.0f, 0.0f) - fragPos);
   vec3 h = normalize(l + e);

   vec3 texKd = texture2D(texture, fragTexCoords).rgb;
   vec3 texKa = vec3(texture2D(texture, fragTexCoords).rgb) * 0.5;
   vec3 color = texKa * ka + (texKd + (0.1 * kd)) * max(dot(l, n), 0.0) + (ks * 0.15) * pow(max(dot(h, n), 0), s);
   vec3 colorFinal = intensity * color;

   gl_FragColor = vec4(colorFinal.r, colorFinal.g, colorFinal.b, 1.0);
}

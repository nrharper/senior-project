#version 120

attribute vec4 vertPos;
uniform mat4 MVP;

void main()
{
	gl_Position = MVP * vertPos;
}

#ifndef __Texture__
#define __Texture__

#ifdef __APPLE__
#include <GLUT/glut.h>
#endif
#ifdef __unix__
#include <GL/glut.h>
#endif
#include <string>

class Texture
{
public:
	Texture();
	virtual ~Texture();
	void setFilename(const std::string &f) { filename = f; }
	void init();
	void bind(GLint handle, GLint unit);
	void unbind(GLint unit);
	
private:
	std::string filename;
	int width;
	int height;
	GLuint tid;
	
};

#endif

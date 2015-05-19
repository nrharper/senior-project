#include "../libraries/KartRacer.h"

float timePrev;

Eigen::Vector2f mouse;
bool keyDown[256] = {false};

Program prog_pass1;
Program prog_pass2;
Camera camera;
Scene scene;

GLuint shadowmap_width = 4096;
GLuint shadowmap_height = 4096;

GLuint framebufferID;
GLuint shadowMap;

bool cull = true;
bool line = false;
int centerWidth, centerHeight;


void loadScene()
{
	// time
	timePrev = 0.0f;

	scene.load("../materials/myscene.txt");
	scene.bindCamera(&camera);

	prog_pass1.setShaderNames("../source/Pass1_vert.glsl", "../source/Pass1_frag.glsl");
	prog_pass2.setShaderNames("../source/Pass2_vert.glsl", "../source/Pass2_frag.glsl");
}

void initGL()
{
	//////////////////////////////////////////////////////
	// Initialize GL for the whole scene
	//////////////////////////////////////////////////////
	
	// Set background color
	glClearColor(0.5f, 0.5f, 1.0f, 1.0f);
	// Enable z-buffer test
	glEnable(GL_DEPTH_TEST);
	
	glGenFramebuffers(1, &framebufferID);
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferID);
	glGenTextures(1, &shadowMap);
	glBindTexture(GL_TEXTURE_2D, shadowMap);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, shadowmap_width, shadowmap_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowMap, 0);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		std::cerr << "Framebuffer is not ok" << std::endl;
	}
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

   //////////////////////////////////////////////////////
	// Initialize the geometry
	//////////////////////////////////////////////////////

	scene.init();

	//////////////////////////////////////////////////////
	// Intialize the shaders
	//////////////////////////////////////////////////////
	
	prog_pass1.init();
	prog_pass1.addUniform("MVP");
	prog_pass1.addAttribute("vertPos");

	prog_pass2.init();
	prog_pass2.addUniform("P");
	prog_pass2.addUniform("MV");
	prog_pass2.addUniform("lightMVP");
	prog_pass2.addUniform("Tscale");
	prog_pass2.addAttribute("vertPos");
	prog_pass2.addAttribute("vertNor");
	prog_pass2.addAttribute("vertTex");
	prog_pass2.addUniform("shadowMap");
	prog_pass2.addUniform("lightPos");
	prog_pass2.addUniform("ka");
	prog_pass2.addUniform("kd");
	prog_pass2.addUniform("ks");
	prog_pass2.addUniform("texture");
}

void reshapeGL(int w, int h)
{
	// Set view size
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	// Set center of screen
	centerWidth = w / 2;
	centerHeight = h / 2;
	// Set camera aspect ratio
	camera.setWindowSize(w, h);
}

void drawGL()
{
	// Enable backface culling
	if(cull) {
		glEnable(GL_CULL_FACE);
	} else {
		glDisable(GL_CULL_FACE);
	}
	if(line) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	} else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);

	// Pass 1: get depth from light source
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferID);
	glViewport(0, 0, shadowmap_width, shadowmap_height);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glCullFace(GL_FRONT);
	prog_pass1.bind();

	scene.draw(keyDown, &prog_pass1, true);

	prog_pass1.unbind();

	// Pass 2: draw with shadow info
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glViewport(0, 0, centerWidth * 2, centerHeight * 2);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.5, 0.5, 0.5, 1);//gray color, same as fog color
	glClearDepth(1);
	glEnable(GL_DEPTH_TEST);
	glCullFace(GL_BACK);
	prog_pass2.bind();
	glUniform3fv(prog_pass2.getUniform("ka"),  1, Eigen::Vector3f(0.3f, 0.3f, 0.3f).data());
	glUniform3fv(prog_pass2.getUniform("kd"),  1, Eigen::Vector3f(0.8f, 0.7f, 0.7f).data());
	glUniform3fv(prog_pass2.getUniform("ks"), 1, Eigen::Vector3f(1.0f, 0.9f, 0.8f).data());

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, shadowMap);
	glUniform1i(prog_pass2.getUniform("shadowMap"), 0);

	scene.draw(keyDown, &prog_pass2, false);

	prog_pass2.unbind();

	// Double buffer
	glutSwapBuffers();
}


void passiveMotionGL(int x, int y)
{
	mouse(0) = x;
	mouse(1) = y;
}

void keyboardGL(unsigned char key, int x, int y)
{
	keyDown[key] = true;
	Eigen::Vector3f lightPos = scene.getLight().getPosition();
	switch(key) {
		case 27:
			// ESCAPE
			exit(0);
			break;
		case 'c':
			cull = !cull;
			break;
		case 'l':
			line = !line;
			break;
      case ' ':
         break;
		case 'x':
			lightPos(0) += 0.1;
			scene.getLight().setPosition(lightPos);
			std::cout << "Light: (" << lightPos(0) << ", " << lightPos(1) << ", " << lightPos(2) << ")" << std::endl;
			break;
		case 'X':
			lightPos(0) -= 0.1;
			scene.getLight().setPosition(lightPos);
			std::cout << "Light: (" << lightPos(0) << ", " << lightPos(1) << ", " << lightPos(2) << ")" << std::endl;
			break;
		case 'y':
			lightPos(1) += 0.1;
			scene.getLight().setPosition(lightPos);
			std::cout << "Light: (" << lightPos(0) << ", " << lightPos(1) << ", " << lightPos(2) << ")" << std::endl;
			break;
		case 'Y':
			lightPos(1) -= 0.1;
			scene.getLight().setPosition(lightPos);
			std::cout << "Light: (" << lightPos(0) << ", " << lightPos(1) << ", " << lightPos(2) << ")" << std::endl;
			break;
		case 'z':
			lightPos(2) += 0.1;
			scene.getLight().setPosition(lightPos);
			std::cout << "Light: (" << lightPos(0) << ", " << lightPos(1) << ", " << lightPos(2) << ")" << std::endl;
			break;
		case 'Z':
			lightPos(2) -= 0.1;
			scene.getLight().setPosition(lightPos);
			std::cout << "Light: (" << lightPos(0) << ", " << lightPos(1) << ", " << lightPos(2) << ")" << std::endl;
			break;
		case '=':
			lightPos << 0.0f, 500.0f, 0.0f;
			scene.getLight().setPosition(lightPos);
			break;
	}
}

void keyboardUpGL(unsigned char key, int x, int y)
{
	keyDown[key] = false;
}

void timerGL(int value)
{
	// time step calculation
	float timeCurr = (float)glutGet(GLUT_ELAPSED_TIME)/1000;
	float dt = timeCurr - timePrev;
	timePrev = timeCurr;

	// update everything
	scene.update(keyDown, mouse, dt);
	Vehicle vehicle = scene.getVehicle();
	camera.update(vehicle.getYaw(), vehicle.getPitch(), vehicle.getPosition());

	glutWarpPointer(centerWidth, centerHeight);
	glutPostRedisplay();
	glutTimerFunc(20, timerGL, 0);
}

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitWindowSize(600, 600);
	centerWidth = centerHeight = 300;
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutCreateWindow("Noah Harper");
	if (argc == 1) {
		glutFullScreen();
	}
	glutSetCursor(GLUT_CURSOR_NONE);
	glutWarpPointer(centerWidth, centerHeight);
	glutPassiveMotionFunc(passiveMotionGL);
	glutKeyboardFunc(keyboardGL);
	glutKeyboardUpFunc(keyboardUpGL);
	glutReshapeFunc(reshapeGL);
	glutDisplayFunc(drawGL);
	glutTimerFunc(20, timerGL, 0);
	loadScene();
	initGL();
	glutMainLoop();
	return 0;
}

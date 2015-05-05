#include "../libraries/KartRacer.h"

using namespace std;

float timePrev;
float h = 0.1f;
float t = 0.0f;
float tireSpin = 0.0f;
Eigen::Vector3f g(0.0f, 0.0f, 0.0f);
Eigen::Vector2f mouse;
bool keyDown[256] = {false};

Program prog;
Camera camera;
Scene scene;

bool cull = false;
bool line = false;
int centerWidth, centerHeight;
// Vertex buffer data. This data is populated on the CPU and sent to the GPU.
vector<float> positionBuf;
vector<unsigned int> indexBuf;
// Vertex buffer object IDs. These are used by OpenGL to reference the buffers.
GLuint positionBufID;
GLuint indexBufID;

// Model matrix for the plane
Eigen::Matrix4f T;
Eigen::Matrix3f T1 = Eigen::Matrix3f::Identity();
Eigen::Vector3f lightPosCam;

void loadScene()
{
	// time
	timePrev = 0.0f;
	t = 0.0f;

	scene.load("../materials/myscene.txt");

	prog.setShaderNames("../source/simple_vert.glsl", "../source/simple_frag.glsl");
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
	
   //////////////////////////////////////////////////////
	// Initialize the geometry
	//////////////////////////////////////////////////////

	scene.init();

	//////////////////////////////////////////////////////
	// Intialize the shaders
	//////////////////////////////////////////////////////
	
	prog.init();
	prog.addUniform("P");
	prog.addUniform("MV");
	prog.addUniform("T1");
	prog.addAttribute("vertPos");
	prog.addAttribute("vertNor");
	prog.addAttribute("vertTex");
	prog.addUniform("lightPos");
	prog.addUniform("intensity");
	prog.addUniform("ka");
	prog.addUniform("kd");
	prog.addUniform("ks");
	prog.addUniform("s");
	prog.addUniform("texture");
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
	// Clear buffers
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

	// Create matrix stacks
	MatrixStack P, MV;
	// Apply camera transforms
	P.pushMatrix();
	camera.applyProjectionMatrix(&P);
	MV.pushMatrix();
	camera.applyViewMatrix(&MV);


	// Get light position in Camera space
	Eigen::Vector4f lightPos = MV.topMatrix() * Eigen::Vector4f(-500.0f, 200.0f, -500.0f, 1.0f);

	// Bind the program
	prog.bind();

	// Send shader info to the GPU
	glUniform3fv(prog.getUniform("lightPos"), 1, lightPos.data());
	glUniform1f(prog.getUniform("intensity"), 1.0f);
	glUniform3fv(prog.getUniform("ka"),  1, Eigen::Vector3f(0.2f, 0.2f, 0.2f).data());
	glUniform3fv(prog.getUniform("kd"),  1, Eigen::Vector3f(0.8f, 0.7f, 0.7f).data());
	glUniform3fv(prog.getUniform("ks"), 1, Eigen::Vector3f(1.0f, 0.9f, 0.8f).data());
	glUniform1f(prog.getUniform("s"), 200.0f);
	glUniformMatrix4fv(prog.getUniform("P"), 1, GL_FALSE, P.topMatrix().data());

	/////
	// Draw shapes
	scene.draw(keyDown, MV, &prog);

	// Unbind
	glUseProgram(0);

	// Pop stacks
	MV.popMatrix();
	P.popMatrix();

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
	}
}

void keyboardUpGL(unsigned char key, int x, int y)
{
	keyDown[key] = false;
}

void timerGL(int value)
{
	float timeCurr = (float)glutGet(GLUT_ELAPSED_TIME)/1000;
	float dt = timeCurr - timePrev;
	timePrev = timeCurr;
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

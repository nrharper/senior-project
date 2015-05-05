#ifdef __APPLE__
#include <GLUT/glut.h>
#endif
#ifdef __unix__
#include <GL/glut.h>
#endif
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include "GLSL.h"
#include "Vehicle.h"
#include "Camera.h"
#include "Shape.h"
#include "MatrixStack.h"
#include "CollisionBox.h"
#include <stdio.h>
#include "WorldObject.h"
#include "Image.h"
#include "Program.h"
#include "Texture.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Scene.h"
#include "Track.h"

using namespace std;
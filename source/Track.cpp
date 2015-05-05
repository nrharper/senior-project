#include "../libraries/Track.h"
#include <Eigen/Geometry>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>

using namespace std;
// PQP

Track::Track()
{
}

Track::~Track()
{
}

void pushMeshPoint(std::vector<float> &mesh, Eigen::Vector3f &point) {
	mesh.push_back(point(0));
   mesh.push_back(point(1));
   mesh.push_back(point(2));
}

void pushTriangleIndices(std::vector<unsigned int> &indices, int a, int b, int c) {
	indices.push_back(a);
   indices.push_back(b);
   indices.push_back(c);
}

// Builds a spline curve and constructs the objects for it
void Track::buildGeometry() {
   int ncps = (int)cps.size();
   Eigen::MatrixXf G(3,ncps);
   Eigen::MatrixXf Gn(3,ncps);
   Eigen::MatrixXf Gk(3,4);
   Eigen::MatrixXf Gnk(3,4);
   for(int i = 0; i < ncps; ++i) {
      G.block<3,1>(0,i) = cps[i].cp;
   }
   for(int i = 0; i < ncps; ++i) {
      Gn.block<3,1>(0,i) = cps[i].n;
   }
   float ds = 1.0f;
	int atEnd = 0;
   float smax = usTable.back().second, s = 0.0f; // spline length
   while(s < smax) {
		//if (s > smax)
		//	s = smax;
		MeshSegment mesh;
		for (int i = 0; i < 10 && s < smax; i++) {
			//if (s > smax)
			//	s = smax;
		   // Convert from s to (concatenated) u
		   float uu = s2u(s);
		   // Convert from concatenated u to the usual u between 0 and 1.
		   float kfloat;
		   float u = std::modf(uu, &kfloat);
		   // k is the index of the starting control point
		   int k = (int)std::floor(kfloat);
		   // Gk is the 3x4 block starting at column k
			Gk = G.block<3,4>(0,k);
			Gnk = Gn.block<3,4>(0,k);
		   // Compute spline point at u

		   Eigen::Vector4f uVec(1.0f, u, u*u, u*u*u);
			Eigen::Vector4f uPrime(0.0f, 1.0f, 2.0f * u, 3.0f * u * u);
			Eigen::Vector4f uPrimePrime(0.0f, 0.0f, 2.0f, 6.0f * u);
		   Eigen::Vector3f P = Gk * B * uVec;
			if (P(1) < 0)
				P(1) = 0;

			Eigen::Vector3f tracknormal = Gnk * B * uVec;
			Eigen::Vector3f pPrime = Gk * B * uPrime;
			Eigen::Vector3f pPrimePrime = Gk * B * uPrimePrime;
			Eigen::Vector3f tangent = pPrime / pPrime.norm();
			tangent.normalize();
			Eigen::Vector3f binormal = tracknormal.cross(tangent);
			binormal.normalize();

		   P = P + 5.0f * binormal;
			pushMeshPoint(mesh.positions, P);

		   Eigen::Vector3f P2 = P - 5.0f * binormal - 5.0f * binormal;
			pushMeshPoint(mesh.positions, P2);

			mesh.texcoords.push_back(0.0f);
			mesh.texcoords.push_back((float)i * 0.1f);
			mesh.texcoords.push_back(1.0f);
			mesh.texcoords.push_back((float)i * 0.1f);

			if (atEnd == 1) {
				pushMeshPoint(topMeshes.back().positions, P);
				pushMeshPoint(topMeshes.back().positions, P2);
				topMeshes.back().texcoords.push_back(0.0f);
				topMeshes.back().texcoords.push_back(1.0f);
				topMeshes.back().texcoords.push_back(1.0f);
				topMeshes.back().texcoords.push_back(1.0f);
				atEnd = 0;
			}

			s += ds;
		}
		atEnd = 1;
		topMeshes.push_back(mesh);
   }

	// Build the lower mesh and set the normals for both upper and lower meshes
	for (auto &mesh : topMeshes) {
		MeshSegment bottomMesh;
		for (int i = 0; i < (int)mesh.positions.size() / 3; i += 2) {
			Eigen::Vector3f normal1, normal2;
			Eigen::Vector3f pos1, pos2, pos3, pos4;
			int ind1 = 3 * i, ind2 = 3 * (i + 1), ind3, ind4;
			pos1 << mesh.positions.at(ind1), mesh.positions.at(ind1 + 1), mesh.positions.at(ind1 + 2);
			pos2 << mesh.positions.at(ind2), mesh.positions.at(ind2 + 1), mesh.positions.at(ind2 + 2);

			if (i != (int)mesh.positions.size() / 3 - 2) {
				ind3 = 3 * (i + 2);
				ind4 = 3 * (i + 3);
			}
			else {
				ind3 = 3 * (i - 2);
				ind4 = 3 * (i - 1);
			}

			pos3 << mesh.positions.at(ind3), mesh.positions.at(ind3 + 1), mesh.positions.at(ind3 + 2);
			pos4 << mesh.positions.at(ind4), mesh.positions.at(ind4 + 1), mesh.positions.at(ind4 + 2);

			Eigen::Vector3f pv1, pv2, pv3, pv4;
			pv1 = pos2 - pos1;
			pv2 = pos3 - pos1;
			//pv1.normalize();
			//pv2.normalize();
			pv3 = pos1 - pos2;
			pv4 = pos4 - pos2;
			//pv3.normalize();
			//pv4.normalize();

			if (i == (int)mesh.positions.size() / 3 - 2) {
				normal1 = pv2.cross(pv1);
				normal2 = pv3.cross(pv4);
			}
			else {
				normal1 = pv1.cross(pv2);
				normal2 = pv4.cross(pv3);
			}
			normal1.normalize();
			normal2.normalize();

			pushMeshPoint(mesh.normals, normal1);
			pushMeshPoint(mesh.normals, normal2);

			///
			// Bottom mesh positions and normals
			if (i == 0) { // Use the first vertices of each segment as a point on the collisionMesh
				pushMeshPoint(collisionMesh.positions, pos1);
				pushMeshPoint(collisionMesh.positions, pos2);
			}
			pushMeshPoint(bottomMesh.positions, pos1);
			pushMeshPoint(bottomMesh.positions, pos2);

			bottomMesh.texcoords.push_back(0.0f);
			bottomMesh.texcoords.push_back((float)(i / 2) * 0.1f);
			bottomMesh.texcoords.push_back(1.0f);
			bottomMesh.texcoords.push_back((float)(i / 2) * 0.1f);

			// Get the normals of the sides
			Eigen::Vector3f sideNorm1, sideNorm2;
			sideNorm1 = normal1.cross(pv2);
			sideNorm2 = -normal2.cross(pv4);

			if (i == (int)mesh.positions.size() / 3 - 2) {
				sideNorm1 = pv2.cross(normal1);
				sideNorm2 = -pv4.cross(normal2);
			}
			else {
				sideNorm1 = normal1.cross(pv2);
				sideNorm2 = -normal2.cross(pv4);
			}

			pushMeshPoint(bottomMesh.normals, sideNorm1);
			pushMeshPoint(bottomMesh.normals, sideNorm2);

			pos1 = pos1 - 0.5f * normal1;
			pos2 = pos2 - 0.5f * normal2;
			if (i == 0) { // Use the first vertices of each segment as a point on the collisionMesh
				pushMeshPoint(collisionMesh.positions, pos1);
				pushMeshPoint(collisionMesh.positions, pos2);
			}
			pushMeshPoint(bottomMesh.positions, pos1);
			pushMeshPoint(bottomMesh.positions, pos2);

			bottomMesh.texcoords.push_back(0.0f  + 1.0f/12.0f);
			bottomMesh.texcoords.push_back((float)(i / 2) * 0.1f);
			bottomMesh.texcoords.push_back(1.0f  - 1.0f/12.0f);
			bottomMesh.texcoords.push_back((float)(i / 2) * 0.1f);

			pushMeshPoint(bottomMesh.positions, pos1);
			pushMeshPoint(bottomMesh.positions, pos2);

			bottomMesh.texcoords.push_back(0.0f  + 1.0f/12.0f);
			bottomMesh.texcoords.push_back((float)(i / 2) * 0.1f);
			bottomMesh.texcoords.push_back(1.0f  - 1.0f/12.0f);
			bottomMesh.texcoords.push_back((float)(i / 2) * 0.1f);

			Eigen::Vector3f reverseNorm1 = -normal1;
			Eigen::Vector3f reverseNorm2 = -normal2;
			pushMeshPoint(bottomMesh.normals, reverseNorm1);
			pushMeshPoint(bottomMesh.normals, reverseNorm2);

			pushMeshPoint(bottomMesh.normals, sideNorm1);
			pushMeshPoint(bottomMesh.normals, sideNorm2);
		}
		bottomMeshes.push_back(bottomMesh);
	}


	// Set up the triangle indices for the upper mesh
	for (auto &mesh : topMeshes) {
		for (int i = 0; i < (int)mesh.positions.size() / 6 - 1; i++) {
			//Top mesh triangles
		   pushTriangleIndices(mesh.indices, i * 2, i * 2 + 1, i * 2 + 3);
		   pushTriangleIndices(mesh.indices, i * 2, i * 2 + 3, i * 2 + 2);
		}
	}

	// Set up the triangle indices for the lower mesh
	for (auto &mesh : bottomMeshes) {
		for (int i = 0; i < (int)mesh.positions.size() / 18 - 1; i++) {
			// Left side mesh triangles
			pushTriangleIndices(mesh.indices, i * 6, i * 6 + 10, i * 6 + 4);
		   pushTriangleIndices(mesh.indices, i * 6, i * 6 + 6, i * 6 + 10);

			// Bottom mesh triangles
			pushTriangleIndices(mesh.indices, i * 6 + 2, i * 6 + 9, i * 6 + 3);
		   pushTriangleIndices(mesh.indices, i * 6 + 2, i * 6 + 8, i * 6 + 9);

			// Right side mesh triangles
			pushTriangleIndices(mesh.indices, i * 6 + 1, i * 6 + 11, i * 6 + 7);
		   pushTriangleIndices(mesh.indices, i * 6 + 1, i * 6 + 5, i * 6 + 11);
		}
	}

	for (int i = 0; i < (int)collisionMesh.positions.size() / 12 - 1; i++) {
		// Top mesh triangles 
		pushTriangleIndices(collisionMesh.indices, i * 4, i * 4 + 5, i * 4 + 4);
		pushTriangleIndices(collisionMesh.indices, i * 4, i * 4 + 1, i * 4 + 5);

		// Left side mesh triangles
		pushTriangleIndices(collisionMesh.indices, i * 4, i * 4 + 6, i * 4 + 2);
		pushTriangleIndices(collisionMesh.indices, i * 4, i * 4 + 4, i * 4 + 6);

		// Bottom mesh triangles
		pushTriangleIndices(collisionMesh.indices, i * 4 + 2, i * 4 + 7, i * 4 + 3);
		pushTriangleIndices(collisionMesh.indices, i * 4 + 2, i * 4 + 6, i * 4 + 7);

		// Right side mesh triangles
		pushTriangleIndices(collisionMesh.indices, i * 4 + 1, i * 4 + 7, i * 4 + 5);
		pushTriangleIndices(collisionMesh.indices, i * 4 + 1, i * 4 + 3, i * 4 + 7);
	}
}

// Builds the lookup table for arclength parameterization
void Track::buildTable() {
	usTable.clear();
	int ncps = (int)cps.size();
	if(ncps >= 4) {
      usTable.push_back(make_pair(0.0f, 0.0f));
      
		Eigen::MatrixXf G(3,ncps);
		Eigen::MatrixXf Gk(3,4);
		for(int i = 0; i < ncps; ++i) {
			G.block<3,1>(0,i) = cps[i].cp;
		}

      for (int k = 0; k < ncps - 3; k++) {
         for (float u = 0.2f; u <= 1.0f; u += 0.2f) {
			   Gk = G.block<3,4>(0,k);

            float tmp = -0.1f * sqrt(0.6f) + u - 0.1f;
			   Eigen::Vector4f uVec(0.0f, 1.0f, 2.0f * tmp, 3.0f * tmp * tmp);
            Eigen::Vector3f P = Gk * B * uVec;
            float s = (5.0f / 9.0f) * P.norm();

            tmp = u - 0.1f;
            uVec(2) = 2.0f * tmp;
            uVec(3) = 3.0f * tmp * tmp;
            P = Gk * B * uVec;
            s += (8.0f / 9.0f) * P.norm();

            tmp = 0.1f * sqrt(0.6f) + u - 0.1f;
            uVec(2) = 2.0f * tmp;
            uVec(3) = 3.0f * tmp * tmp;
            P = Gk * B * uVec;
            s += (5.0f / 9.0f) * P.norm();

            usTable.push_back(make_pair(u + k, usTable.back().second + s * 0.1f));
         }
	   }
   }

	curveLength = (float)usTable.back().second;
}

// Arclength Parameterization
float Track::s2u(float s) {
   float alpha;
   int i;
   for (i = 0; i < (int)usTable.size(); i++) {
      if (s < (float)usTable[i].second)
         break;
   }
   alpha = (s - usTable[i - 1].second) / (usTable[i].second - usTable[i - 1].second);

   return (1 - alpha) * usTable[i - 1].first + alpha * usTable[i].first;
}

// Read in a set of points from a file and build a track
//   Includes a direction vector representing the direction the top faces
void Track::load(const char *filename) {
   B << 0.0f, -1.0f,  2.0f, -1.0f,
         2.0f,  0.0f, -5.0f,  3.0f,
         0.0f,  1.0f,  4.0f, -3.0f,
         0.0f,  0.0f, -1.0f,  1.0f;
   B *= 0.5;
	cps.clear();


	ifstream in;
	in.open(filename);
	if(!in.good()) {
		std::cout << "Cannot read " << filename << endl;
		exit(1);
	}

	string line;
	
	int ncps;
	in >> ncps;
	getline(in, line);

	KeyFrame frame;
	Eigen::Vector3f cp;
	Eigen::Vector3f axis;
	Eigen::Vector3f n;
	// get each control point
	for (int i = 0; i < ncps; i++) {
		getline(in, line);
		getline(in, line);
		stringstream ss(line);
		float px, py, pz; // First get translation
		ss >> px;
		ss >> py;
		ss >> pz;
		cp << px, py, pz;
		frame.cp = cp;

		getline(in, line);
		stringstream ss0(line);
		float nx, ny, nz; // Next get scale
		ss0 >> nx;
		ss0 >> ny;
		ss0 >> nz;
		n << nx, ny, nz;
		n.normalize();
		frame.n = n;

		cps.push_back(frame);
	}

	while(1) {
		getline(in, line);
		if(in.eof()) {
			break;
		}
		// Skip empty lines
		if(line.size() < 2) {
			continue;
		}
		// Skip comments
		if(line.at(0) == '#') {
			continue;
		}
	}
	in.close();

	toptex.setFilename("../materials/road.jpg");
	bottomtex.setFilename("../materials/diffuse.bmp");
   buildTable();
   buildGeometry();
}

// Initialize the geometry for the lower and upper meshes
void Track::init() {
	for (auto &mesh : topMeshes) {
		initMesh(mesh);
	}
	for (auto &mesh : bottomMeshes) {
		initMesh(mesh);
	}
	initMesh(collisionMesh);
	toptex.init();
	bottomtex.init();
}

// OpenGL init handler
void Track::initMesh(MeshSegment &mesh) {
	// Send the position array to the GPU
   const vector<float> &posBuf = mesh.positions;
   glGenBuffers(1, &mesh.posBufID);
   glBindBuffer(GL_ARRAY_BUFFER, mesh.posBufID);
   glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_STATIC_DRAW);
   
   // Send the normal array (if it exists) to the GPU
   const vector<float> &norBuf = mesh.normals;
   if(!norBuf.empty()) {
      glGenBuffers(1, &mesh.norBufID);
      glBindBuffer(GL_ARRAY_BUFFER, mesh.norBufID);
      glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_STATIC_DRAW);
   } else {
      mesh.norBufID = 0;
   }
   
   // Send the texture coordinates array (if it exists) to the GPU
   const vector<float> &texBuf = mesh.texcoords;
   if(!texBuf.empty()) {
      glGenBuffers(1, &mesh.texBufID);
      glBindBuffer(GL_ARRAY_BUFFER, mesh.texBufID);
      glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
   } else {
      mesh.texBufID = 0;
   }
   
   // Send the index array to the GPU
   const vector<unsigned int> &indBuf = mesh.indices;
   glGenBuffers(1, &mesh.indBufID);
   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.indBufID);
   glBufferData(GL_ELEMENT_ARRAY_BUFFER, indBuf.size()*sizeof(unsigned int), &indBuf[0], GL_STATIC_DRAW);
   
   // Unbind the arrays
   glBindBuffer(GL_ARRAY_BUFFER, 0);
   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
   
   assert(glGetError() == GL_NO_ERROR);
}

// Draws the track. Binds a seperate texture for lower and upper parts
void Track::draw(MatrixStack &MV, Program *prog) {
   glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, MV.topMatrix().data());
	Eigen::Matrix3f T1 = Eigen::Matrix3f::Identity();
	glUniformMatrix3fv(prog->getUniform("T1"), 1, GL_TRUE, T1.data());

	toptex.bind(prog->getUniform("texture"), 2);
	for (auto &mesh : topMeshes) {
		drawMesh(prog, mesh);
	}
	toptex.unbind(2);
	bottomtex.bind(prog->getUniform("texture"), 3);
	for (auto &mesh : bottomMeshes) {
		drawMesh(prog, mesh);
	}
	drawMesh(prog, collisionMesh);
	bottomtex.unbind(3);
}

// OpenGL draw hander
void Track::drawMesh(Program *prog, MeshSegment &mesh) {
   int h_pos = prog->getAttribute("vertPos"), h_nor = prog->getAttribute("vertNor"), h_tex = prog->getAttribute("vertTex");
	// Enable and bind position array for drawing
   GLSL::enableVertexAttribArray(h_pos);
   glBindBuffer(GL_ARRAY_BUFFER, mesh.posBufID);
   glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, 0);
   
   // Enable and bind normal array (if it exists) for drawing
   if(mesh.norBufID && h_nor >= 0) {
      GLSL::enableVertexAttribArray(h_nor);
      glBindBuffer(GL_ARRAY_BUFFER, mesh.norBufID);
      glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, 0);
   }
   
   // Enable and bind texcoord array (if it exists) for drawing
   if(mesh.texBufID && h_tex >= 0) {
      GLSL::enableVertexAttribArray(h_tex);
      glBindBuffer(GL_ARRAY_BUFFER, mesh.texBufID);
      glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, 0);
   }
   
   // Bind index array for drawing
   int nIndices = (int)mesh.indices.size();
   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.indBufID);
   
   // Draw
   glDrawElements(GL_TRIANGLES, nIndices, GL_UNSIGNED_INT, 0);
   
   // Disable and unbind
   if(mesh.texBufID && h_tex >= 0) {
      GLSL::disableVertexAttribArray(h_tex);
   }
   if(mesh.norBufID && h_nor >= 0) {
      GLSL::disableVertexAttribArray(h_nor);
   }
   GLSL::disableVertexAttribArray(h_pos);
   glBindBuffer(GL_ARRAY_BUFFER, 0);
   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


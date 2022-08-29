#include "Mesh.h"

using namespace std;

void Mesh::load( const char* filename )
{
	// 2.1.1. load() should populate bindVertices, currentVertices, and faces

	// Add your code here.
	ifstream fin(filename, std::ifstream::in);

	string type;
	float x, y, z;
	int i, j, k;

	while(fin >> type) {
		if(type == "v" && (fin >> x >> y >> z)) {
			// vertice
			bindVertices.push_back(Vector3f(x, y, z ));
		} else if(type == "f" && (fin >> i >> j >> k)) {
			// face
			Tuple3u tp(i,j,k);
			faces.push_back(tp);
		} else {
			cerr << "unknown type " << type << '\n';
		}
	}
	fin.close();

	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	// Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".

	glBegin(GL_TRIANGLES);
	for(int i = 0, nfaces = faces.size(); i < nfaces; ++i) {

		int i1 = faces[i][0]-1, i2 = faces[i][1]-1, i3 = faces[i][2]-1;

		assert(i1 >= 0 && i2 >= 0 && i3 >= 0);
		Vector3f v1 = currentVertices[i2] - currentVertices[i1];
		Vector3f v2 = currentVertices[i3] - currentVertices[i1];
		Vector3f norm = Vector3f::cross(v1, v2).normalized();

		
		glNormal3d(norm[0], norm[1], norm[2]);
		glVertex3d(currentVertices[i1][0], currentVertices[i1][1], currentVertices[i1][2]);
		glVertex3d(currentVertices[i2][0], currentVertices[i2][1], currentVertices[i2][2]);
		glVertex3d(currentVertices[i3][0], currentVertices[i3][1], currentVertices[i3][2]);
		
	}
	glEnd();
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 2.2. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments

	// each line contains 17 values
	ifstream fin(filename, std::ifstream::in);

	string line;


	char buf[16];
	char *ch;
	float w;
	int idx;
	while(getline(fin, line)) {
		// chop line
		idx = 0;
		std::vector< float > weights(17);
		weights[0] = 0;
		ch = buf;
		int i = 0, m = line.size()-1;
		while(i < m) {
			while(i < m && line[i] != ' ')
				*ch++ = line[i++];
			*ch = 0;
			w = atof(buf);
			ch = buf;
			weights[idx++] = w;
			i++;
		}
		// cerr<< "m = " << m << endl;
		// for(int j=0; j<=idx; j++)
		// 	cerr << j << ' ' << weights[j] << ", ";
		// cerr << endl;
		assert(idx == 17);

		attachments.push_back(weights);

	}

	fin.close();
	assert(currentVertices.size() == attachments.size());

}
